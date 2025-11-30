"""
Main Control Script
Combines all modules to provide robot stabilization
"""

import cv2
import logging
import signal
import sys
import time
from datetime import datetime

try:
    from .pixhawk_connection import PixhawkConnection
    from .image_processor import ImageProcessor
    from .forward_controller import ForwardController
    from .yaw_controller import YawController
    from .lateral_controller import LateralController
    from .throttle_controller import ThrottleController
except ImportError:
    # Absolute import fallback (for direct script execution)
    from pixhawk_connection import PixhawkConnection
    from image_processor import ImageProcessor
    from forward_controller import ForwardController
    from yaw_controller import YawController
    from lateral_controller import LateralController
    from throttle_controller import ThrottleController


class UUVControlSystem:
    """Main robot control system"""
    
    # Global reference for signal handler
    _instance = None
    
    def __init__(self, connection_string='udp:127.0.0.1:14551', 
                 camera_index=0, frame_width=640, frame_height=480,
                 marker_size=0.1, show_video=False):
        """
        Initialize control system
        
        Args:
            connection_string: Pixhawk connection string
            camera_index: Camera index
            frame_width: Image width
            frame_height: Image height
            marker_size: ArUco marker size in meters (default: 0.1m = 10cm)
            show_video: Whether to show video window (default: False - terminal only mode)
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.show_video = show_video
        
        # Pixhawk connection
        self.pixhawk = PixhawkConnection(connection_string)
        
        # Image processing with pose estimation
        self.image_processor = ImageProcessor(marker_size=marker_size)
        
        # Controllers
        self.forward_controller = ForwardController(target_area=20000)
        self.yaw_controller = YawController()
        self.lateral_controller = LateralController(frame_width, frame_height)
        self.throttle_controller = ThrottleController(frame_width, frame_height)
        
        # Camera
        self.camera_index = camera_index
        self.cap = None
        
        # Running state
        self.running = False
        self.shutting_down = False
        
        # Pose information for terminal output
        self.last_pose_info = None
        
        # Set global instance for signal handler
        UUVControlSystem._instance = self
        
        # Logging setup
        self.setup_logging()
    
    def setup_logging(self):
        """Setup logging system"""
        log_filename = f"uuv_control_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S',
            handlers=[
                logging.FileHandler(log_filename, encoding='utf-8'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
        logging.info("=" * 60)
        logging.info("UUV POSITION STABILIZATION CONTROL SYSTEM")
        logging.info(f"Log file: {log_filename}")
        logging.info(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("=" * 60)
    
    def initialize_camera(self):
        """Initialize camera"""
        logging.info("Initializing camera...")
        
        for idx in [self.camera_index, 0, 1, 2, 3]:
            cap = cv2.VideoCapture(idx)
            
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                cap.set(cv2.CAP_PROP_FPS, 30)
                
                ret, frame = cap.read()
                if ret and frame is not None:
                    logging.info(f"Camera {idx} opened successfully - "
                               f"Resolution: {frame.shape[1]}x{frame.shape[0]}")
                    self.cap = cap
                    return True
                else:
                    cap.release()
        
        logging.error("No camera could be opened!")
        return False
    
    def connect_pixhawk(self):
        """Connect to Pixhawk and arm"""
        logging.info("Starting Pixhawk connection...")
        
        if not self.pixhawk.connect():
            logging.error("Pixhawk connection failed!")
            return False
        
        logging.info("Arming Pixhawk...")
        if not self.pixhawk.arm():
            logging.warning("Pixhawk could not be armed, but continuing...")
        
        return True
    
    def draw_control_info(self, frame, corners, ids, marker_info):
        """Draw control information on image"""
        # Target center lines
        cv2.line(frame, (self.frame_width//2, 0), 
                (self.frame_width//2, self.frame_height), (0, 255, 255), 1)
        cv2.line(frame, (0, self.frame_height//2), 
                (self.frame_width, self.frame_height//2), (0, 255, 255), 1)
        cv2.circle(frame, (self.frame_width//2, self.frame_height//2), 5, (0, 255, 255), -1)
        
        # Marker drawing
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            if marker_info:
                info = marker_info[0]
                center = tuple(info['center'])
                
                # Draw center point
                cv2.circle(frame, center, 8, (255, 0, 0), -1)
                
                # ID and area information
                cv2.putText(frame, f"ID: {ids[0][0]}", 
                          (center[0] - 30, center[1] - 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Area: {info['area']:.0f}px", 
                          (center[0] - 40, center[1] + 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Control information
        forward_status = self.forward_controller.get_status()
        yaw_status = self.yaw_controller.get_status()
        lateral_status = self.lateral_controller.get_status()
        throttle_status = self.throttle_controller.get_status()
        
        control_text = [
            "=== UUV CONTROL SYSTEM ===",
            "",
            f"FORWARD (Ch5): PWM={forward_status['pwm']} | {forward_status['direction']}",
            f"YAW (Ch4): PWM={yaw_status['pwm']} | {yaw_status['direction']}",
            f"LATERAL (Ch6): PWM={lateral_status['pwm']} | {lateral_status['direction']}",
            f"THROTTLE (Ch3): PWM={throttle_status['pwm']} | {throttle_status['direction']}",
            "",
            f"Pixhawk: {'ARMED' if self.pixhawk.armed else 'DISARMED'}",
            f"Marker: {'DETECTED' if marker_info else 'NOT DETECTED'}"
        ]
        
        # Add pose information if available
        if self.last_pose_info:
            pose = self.last_pose_info
            control_text.append("")
            control_text.append(f"POSE: X:{pose['x']:.3f}m Y:{pose['y']:.3f}m Z:{pose['z']:.3f}m")
            control_text.append(f"      Roll:{pose['roll']:.1f}° Pitch:{pose['pitch']:.1f}° Yaw:{pose['yaw']:.1f}°")
        
        y_offset = 30
        for i, text in enumerate(control_text):
            color = (255, 255, 255)
            if i == 0:
                color = (255, 255, 0)
            elif 'FORWARD' in text:
                color = (0, 255, 255) if forward_status['pwm'] != 1500 else (255, 255, 255)
            elif 'YAW' in text:
                color = (0, 255, 255) if yaw_status['pwm'] != 1500 else (255, 255, 255)
            elif 'LATERAL' in text:
                color = (0, 255, 255) if lateral_status['pwm'] != 1500 else (255, 255, 255)
            elif 'THROTTLE' in text:
                color = (0, 255, 255) if throttle_status['pwm'] != 1500 else (255, 255, 255)
            elif 'ARMED' in text:
                color = (0, 255, 0) if self.pixhawk.armed else (0, 0, 255)
            elif 'DETECTED' in text:
                color = (0, 255, 0) if marker_info else (0, 0, 255)
            elif 'POSE' in text:
                color = (0, 255, 255)
            
            cv2.putText(frame, text, (10, y_offset + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # FPS information
        fps = self.image_processor.calculate_fps()
        cv2.putText(frame, f"FPS: {fps:.1f}", 
                   (self.frame_width - 120, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return frame
    
    def run(self):
        """Main control loop"""
        logging.info("Starting control system...")
        
        # Initialize camera
        if not self.initialize_camera():
            logging.error("Camera initialization failed!")
            return
        
        # Connect to Pixhawk
        if not self.connect_pixhawk():
            logging.error("Pixhawk connection failed!")
            return
        
        logging.info("=" * 60)
        logging.info("CONTROL SYSTEM READY!")
        logging.info("Channel Assignments:")
        logging.info("  Channel 3 (Throttle): Z axis (Up/Down)")
        logging.info("  Channel 4 (Yaw): Yaw control (Left/Right)")
        logging.info("  Channel 5 (Forward): X axis (Forward/Backward)")
        logging.info("  Channel 6 (Lateral): Y axis (Right/Left)")
        logging.info("=" * 60)
        logging.info("ArUco Pose Estimation: ACTIVE")
        logging.info("Terminal output: x, y, z (meters), roll, pitch, yaw (degrees)")
        if self.show_video:
            logging.info("Video window: ENABLED")
            logging.info("Press 'q' key or Ctrl+C to exit")
        else:
            logging.info("Video window: DISABLED (Terminal only mode - no GUI)")
            logging.info("Press Ctrl+C to exit")
        logging.info("=" * 60)
        logging.info("")
        
        # Detection statistics
        frame_count = 0
        detection_count = 0
        
        self.running = True
        
        # Detection statistics
        frame_count = 0
        detection_count = 0
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logging.warning("Frame could not be read!")
                    continue
                
                frame_count += 1
                
                # Marker detection
                corners, ids = self.image_processor.detect_markers(frame)
                
                # Pose estimation (x, y, z, roll, pitch, yaw)
                pose_info = self.image_processor.estimate_pose(corners, ids)
                
                # Legacy marker info for backward compatibility with controllers
                marker_info = self.image_processor.calculate_marker_info(corners)
                
                # Print pose information to terminal (ArUco pose estimation output)
                # This is the main output - no video window needed, just terminal data
                # Similar to dualaruco.py approach: terminal-only mode
                if pose_info:
                    detection_count += 1
                    for pose in pose_info:
                        self.last_pose_info = pose
                        # Print pose data (similar format to dualaruco.py)
                        # Clear line and print pose data
                        print(f"\r[ArUco Pose] ID:{pose['id']:2d} | "
                              f"X:{pose['x']:7.3f}m | Y:{pose['y']:7.3f}m | Z:{pose['z']:7.3f}m | "
                              f"Roll:{pose['roll']:7.2f}° | Pitch:{pose['pitch']:7.2f}° | Yaw:{pose['yaw']:7.2f}°", 
                              end='', flush=True)
                    
                    # Log detailed info every 30 frames (~1 second at 30fps)
                    if frame_count % 30 == 0:
                        detection_rate = (detection_count / frame_count) * 100
                        logging.info(f"📍 Detection rate: {detection_rate:.1f}% | "
                                   f"Markers: {[p['id'] for p in pose_info]} | "
                                   f"Pos: ({pose_info[0]['x']:.3f}, {pose_info[0]['y']:.3f}, {pose_info[0]['z']:.3f})m | "
                                   f"RPY: ({pose_info[0]['roll']:.1f}°, {pose_info[0]['pitch']:.1f}°, {pose_info[0]['yaw']:.1f}°)")
                else:
                    print("\r[ArUco Pose] No marker detected" + " " * 60, end='', flush=True)
                
                # Calculate control signals
                forward_pwm = self.forward_controller.calculate_control(marker_info)
                yaw_pwm = self.yaw_controller.calculate_control(marker_info)
                lateral_pwm = self.lateral_controller.calculate_control(marker_info)
                throttle_pwm = self.throttle_controller.calculate_control(marker_info)
                
                # Send PWM signals to Pixhawk
                # Channel mapping:
                # Ch1: Roll (0 = ignore)
                # Ch2: Pitch (0 = ignore)
                # Ch3: Throttle (dive/surface)
                # Ch4: Yaw
                # Ch5: Forward (forward/backward)
                # Ch6: Lateral (right/left)
                # Ch7: Mode (0 = ignore)
                # Ch8: (0 = ignore)
                self.pixhawk.send_rc_override([
                    0,              # Ch1: Roll (ignore)
                    0,              # Ch2: Pitch (ignore)
                    throttle_pwm,   # Ch3: Throttle
                    yaw_pwm,        # Ch4: Yaw
                    forward_pwm,    # Ch5: Forward
                    lateral_pwm,    # Ch6: Lateral
                    0,              # Ch7: Mode (ignore)
                    0               # Ch8: (ignore)
                ])
                
                # Visualization (if enabled)
                # Note: When show_video=False, only terminal output is shown (no GUI window)
                if self.show_video:
                    result_frame = self.draw_control_info(frame, corners, ids, marker_info)
                    # Draw pose axis on frame
                    if pose_info:
                        for pose in pose_info:
                            # Draw coordinate axes
                            rvec = pose['rvec']
                            tvec = pose['tvec']
                            axis_length = 0.05  # 5cm axis length
                            axis_points = np.float32([
                                [0, 0, 0],
                                [axis_length, 0, 0],
                                [0, axis_length, 0],
                                [0, 0, -axis_length]
                            ]).reshape(-1, 3)
                            img_points, _ = cv2.projectPoints(
                                axis_points, rvec, tvec, 
                                self.image_processor.camera_matrix, 
                                self.image_processor.dist_coeffs
                            )
                            img_points = np.int32(img_points).reshape(-1, 2)
                            
                            # Draw axes (X: red, Y: green, Z: blue)
                            cv2.line(result_frame, tuple(img_points[0]), tuple(img_points[1]), (0, 0, 255), 3)  # X - Red
                            cv2.line(result_frame, tuple(img_points[0]), tuple(img_points[2]), (0, 255, 0), 3)  # Y - Green
                            cv2.line(result_frame, tuple(img_points[0]), tuple(img_points[3]), (255, 0, 0), 3)  # Z - Blue
                            
                            # Add pose text
                            center = corners[0][0].mean(axis=0).astype(int)
                            pose_text = f"X:{pose['x']:.2f}m Y:{pose['y']:.2f}m Z:{pose['z']:.2f}m"
                            cv2.putText(result_frame, pose_text, 
                                      (center[0] - 100, center[1] + 50),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            rpy_text = f"R:{pose['roll']:.1f}° P:{pose['pitch']:.1f}° Y:{pose['yaw']:.1f}°"
                            cv2.putText(result_frame, rpy_text, 
                                      (center[0] - 100, center[1] + 70),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    
                    cv2.imshow("UUV Control System", result_frame)
                    
                    # Exit control
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        logging.info("\nUser exit request...")
                        break
                
                # Short delay (for performance)
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logging.info("Stopped with Ctrl+C")
        except Exception as e:
            logging.error(f"Unexpected error: {e}", exc_info=True)
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Shutdown system safely"""
        # Prevent multiple shutdown calls
        if self.shutting_down:
            return
        
        self.shutting_down = True
        self.running = False
        
        logging.info("=" * 60)
        logging.info("SAFE SHUTDOWN INITIATED")
        logging.info("=" * 60)
        
        try:
            # Step 1: Reset all PWM signals to neutral (safety first)
            logging.info("[SHUTDOWN] Step 1: Resetting all PWM channels to neutral...")
            if self.pixhawk.connected:
                self.pixhawk.send_rc_override([0] * 8)
                logging.info("[SHUTDOWN] All PWM channels reset to 0 (ignore)")
            
            # Step 2: Disarm Pixhawk
            logging.info("[SHUTDOWN] Step 2: Disarming Pixhawk...")
            if self.pixhawk.connected and self.pixhawk.armed:
                if self.pixhawk.disarm():
                    logging.info("[SHUTDOWN] Pixhawk disarmed successfully")
                else:
                    logging.warning("[SHUTDOWN] Pixhawk disarm may have failed")
            
            # Step 3: Close Pixhawk connection
            logging.info("[SHUTDOWN] Step 3: Closing Pixhawk connection...")
            if self.pixhawk.connected:
                self.pixhawk.disconnect()
                logging.info("[SHUTDOWN] Pixhawk connection closed")
            
            # Step 4: Release camera
            logging.info("[SHUTDOWN] Step 4: Releasing camera...")
            if self.cap is not None:
                self.cap.release()
                logging.info("[SHUTDOWN] Camera released")
            
            # Step 5: Close OpenCV windows (if video was enabled)
            if self.show_video:
                logging.info("[SHUTDOWN] Step 5: Closing OpenCV windows...")
                cv2.destroyAllWindows()
                logging.info("[SHUTDOWN] OpenCV windows closed")
            
            logging.info("=" * 60)
            logging.info("SAFE SHUTDOWN COMPLETED")
            logging.info("=" * 60)
            
        except Exception as e:
            logging.error(f"[SHUTDOWN] Error during shutdown: {e}", exc_info=True)
            # Still try to close critical resources
            try:
                if self.pixhawk.connected:
                    self.pixhawk.send_rc_override([0] * 8)
                if self.cap is not None:
                    self.cap.release()
                cv2.destroyAllWindows()
            except:
                pass


def signal_handler(sig, frame):
    """Signal handler for graceful shutdown"""
    logging.info("=" * 60)
    logging.info("SIGINT (Ctrl+C) received - Initiating safe shutdown...")
    logging.info("=" * 60)
    
    # Get the control system instance and shutdown safely
    if UUVControlSystem._instance is not None:
        UUVControlSystem._instance.shutdown()
    
    logging.info("Program terminated safely.")
    sys.exit(0)


def main():
    """Main function"""
    # Signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Connection string (can be changed if needed)
    # Examples:
    # - 'udp:127.0.0.1:14551' (SITL simulation)
    # - 'tcp:192.168.1.100:5760' (TCP connection)
    # - '/dev/ttyUSB0' (USB serial connection - Linux)
    # - 'COM3' (Windows serial connection)
    connection_string = 'udp:127.0.0.1:14551'
    
    # ArUco marker size in meters (adjust according to your marker size)
    # Common sizes: 0.05m (5cm), 0.1m (10cm), 0.2m (20cm)
    marker_size = 0.1  # 10cm default
    
    # Initialize control system
    control_system = UUVControlSystem(
        connection_string=connection_string,
        camera_index=0,
        frame_width=640,
        frame_height=480,
        marker_size=marker_size,
        show_video=False  # Set to True to enable video window, False for terminal only
    )
    
    # Run
    control_system.run()


if __name__ == "__main__":
    main()
