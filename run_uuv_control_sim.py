"""
UUV Control System - Simulation Mode (No Pixhawk)
This script runs the control system in simulation mode for testing
Pixhawk commands are commented out and replaced with print statements
"""

import cv2
import logging
import signal
import sys
import os
import time
from datetime import datetime

# Add project root directory to path
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# Import modules (without Pixhawk)
try:
    from uuv_control.image_processor import ImageProcessor
    from uuv_control.forward_controller import ForwardController
    from uuv_control.yaw_controller import YawController
    from uuv_control.lateral_controller import LateralController
    from uuv_control.throttle_controller import ThrottleController
except ImportError:
    # Absolute import fallback
    from image_processor import ImageProcessor
    from forward_controller import ForwardController
    from yaw_controller import YawController
    from lateral_controller import LateralController
    from throttle_controller import ThrottleController


class UUVControlSystemSim:
    """Main robot control system - Simulation Mode (No Pixhawk)"""
    
    # Global reference for signal handler
    _instance = None
    
    def __init__(self, connection_string='udp:127.0.0.1:14551', 
                 camera_index=0, frame_width=640, frame_height=480):
        """
        Initialize control system in simulation mode
        
        Args:
            connection_string: Pixhawk connection string (not used in sim mode)
            camera_index: Camera index
            frame_width: Image width
            frame_height: Image height
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Pixhawk connection - SIMULATED
        # self.pixhawk = PixhawkConnection(connection_string)
        self.pixhawk_connected = False
        self.pixhawk_armed = False
        print(f"[SIM] Pixhawk connection string: {connection_string} (SIMULATED)")
        
        # Image processing
        self.image_processor = ImageProcessor()
        
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
        
        # Set global instance for signal handler
        UUVControlSystemSim._instance = self
        
        # Logging setup
        self.setup_logging()
    
    def setup_logging(self):
        """Setup logging system"""
        log_filename = f"uuv_control_sim_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        
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
        logging.info("UUV POSITION STABILIZATION CONTROL SYSTEM - SIMULATION MODE")
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
        """Connect to Pixhawk and arm - SIMULATED"""
        logging.info("Starting Pixhawk connection...")
        print("[SIM] ========================================")
        print("[SIM] PIXHAWK CONNECTION SIMULATION")
        print("[SIM] ========================================")
        
        # SIMULATED: Pixhawk connection
        # if not self.pixhawk.connect():
        #     logging.error("Pixhawk connection failed!")
        #     return False
        print("[SIM] Pixhawk connection: SUCCESS (SIMULATED)")
        self.pixhawk_connected = True
        
        # SIMULATED: Pixhawk arm
        # logging.info("Arming Pixhawk...")
        # if not self.pixhawk.arm():
        #     logging.warning("Pixhawk could not be armed, but continuing...")
        print("[SIM] Pixhawk arm: SUCCESS (SIMULATED)")
        self.pixhawk_armed = True
        print("[SIM] ========================================")
        
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
            "=== UUV CONTROL SYSTEM (SIMULATION MODE) ===",
            "",
            f"FORWARD (Ch5): PWM={forward_status['pwm']} | {forward_status['direction']}",
            f"YAW (Ch4): PWM={yaw_status['pwm']} | {yaw_status['direction']}",
            f"LATERAL (Ch6): PWM={lateral_status['pwm']} | {lateral_status['direction']}",
            f"THROTTLE (Ch3): PWM={throttle_status['pwm']} | {throttle_status['direction']}",
            "",
            f"Pixhawk: {'ARMED' if self.pixhawk_armed else 'DISARMED'} (SIMULATED)",
            f"Marker: {'DETECTED' if marker_info else 'NOT DETECTED'}"
        ]
        
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
                color = (0, 255, 0) if self.pixhawk_armed else (0, 0, 255)
            elif 'DETECTED' in text:
                color = (0, 255, 0) if marker_info else (0, 0, 255)
            
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
        logging.info("Starting control system in SIMULATION MODE...")
        
        # Initialize camera
        if not self.initialize_camera():
            logging.error("Camera initialization failed!")
            return
        
        # Connect to Pixhawk (SIMULATED)
        if not self.connect_pixhawk():
            logging.error("Pixhawk connection failed!")
            return
        
        logging.info("=" * 60)
        logging.info("CONTROL SYSTEM READY! (SIMULATION MODE)")
        logging.info("Channel Assignments:")
        logging.info("  Channel 3 (Throttle): Z axis (Up/Down)")
        logging.info("  Channel 4 (Yaw): Yaw control (Left/Right)")
        logging.info("  Channel 5 (Forward): X axis (Forward/Backward)")
        logging.info("  Channel 6 (Lateral): Y axis (Right/Left)")
        logging.info("=" * 60)
        logging.info("Press 'q' key or Ctrl+C to exit")
        logging.info("")
        
        self.running = True
        frame_count = 0
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logging.warning("Frame could not be read!")
                    continue
                
                # Marker detection
                corners, ids = self.image_processor.detect_markers(frame)
                marker_info = self.image_processor.calculate_marker_info(corners)
                
                # Calculate control signals
                forward_pwm = self.forward_controller.calculate_control(marker_info)
                yaw_pwm = self.yaw_controller.calculate_control(marker_info)
                lateral_pwm = self.lateral_controller.calculate_control(marker_info)
                throttle_pwm = self.throttle_controller.calculate_control(marker_info)
                
                # Send PWM signals to Pixhawk - SIMULATED
                # Channel mapping:
                # Ch1: Roll (0 = ignore)
                # Ch2: Pitch (0 = ignore)
                # Ch3: Throttle (dive/surface)
                # Ch4: Yaw
                # Ch5: Forward (forward/backward)
                # Ch6: Lateral (right/left)
                # Ch7: Mode (0 = ignore)
                # Ch8: (0 = ignore)
                
                # ORIGINAL CODE (COMMENTED):
                # self.pixhawk.send_rc_override([
                #     0,              # Ch1: Roll (ignore)
                #     0,              # Ch2: Pitch (ignore)
                #     throttle_pwm,   # Ch3: Throttle
                #     yaw_pwm,        # Ch4: Yaw
                #     forward_pwm,    # Ch5: Forward
                #     lateral_pwm,    # Ch6: Lateral
                #     0,              # Ch7: Mode (ignore)
                #     0               # Ch8: (ignore)
                # ])
                
                # SIMULATED: Print PWM values instead of sending
                frame_count += 1
                if frame_count % 30 == 0:  # Print every 30 frames (~1 second at 30fps)
                    print("\n" + "=" * 60)
                    print("[SIM] PWM VALUES (Would be sent to Pixhawk):")
                    print(f"[SIM]   Ch1 (Roll):     0 (ignore)")
                    print(f"[SIM]   Ch2 (Pitch):    0 (ignore)")
                    print(f"[SIM]   Ch3 (Throttle): {throttle_pwm} ({'UP' if throttle_pwm > 1500 else 'DOWN' if throttle_pwm < 1500 else 'CENTER'})")
                    print(f"[SIM]   Ch4 (Yaw):      {yaw_pwm} ({'RIGHT' if yaw_pwm > 1500 else 'LEFT' if yaw_pwm < 1500 else 'STRAIGHT'})")
                    print(f"[SIM]   Ch5 (Forward):  {forward_pwm} ({'FORWARD' if forward_pwm > 1500 else 'BACKWARD' if forward_pwm < 1500 else 'NEUTRAL'})")
                    print(f"[SIM]   Ch6 (Lateral):  {lateral_pwm} ({'RIGHT' if lateral_pwm > 1500 else 'LEFT' if lateral_pwm < 1500 else 'CENTER'})")
                    print(f"[SIM]   Ch7 (Mode):     0 (ignore)")
                    print(f"[SIM]   Ch8:            0 (ignore)")
                    print("=" * 60)
                
                # Visualization
                result_frame = self.draw_control_info(frame, corners, ids, marker_info)
                cv2.imshow("UUV Control System (SIMULATION MODE)", result_frame)
                
                # Exit control
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logging.info("User exit request...")
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
        logging.info("SAFE SHUTDOWN INITIATED (SIMULATION MODE)")
        logging.info("=" * 60)
        
        try:
            # Step 1: Reset all PWM signals to neutral (safety first) - SIMULATED
            logging.info("[SHUTDOWN] Step 1: Resetting all PWM channels to neutral...")
            print("[SIM] [SHUTDOWN] Step 1: Resetting all PWM channels to neutral...")
            # if self.pixhawk.connected:
            #     self.pixhawk.send_rc_override([0] * 8)
            print("[SIM] [SHUTDOWN] All PWM channels reset to 0 (ignore) - SIMULATED")
            logging.info("[SHUTDOWN] All PWM channels reset to 0 (ignore) - SIMULATED")
            
            # Step 2: Disarm Pixhawk - SIMULATED
            logging.info("[SHUTDOWN] Step 2: Disarming Pixhawk...")
            print("[SIM] [SHUTDOWN] Step 2: Disarming Pixhawk...")
            # if self.pixhawk.connected and self.pixhawk.armed:
            #     if self.pixhawk.disarm():
            #         logging.info("[SHUTDOWN] Pixhawk disarmed successfully")
            #     else:
            #         logging.warning("[SHUTDOWN] Pixhawk disarm may have failed")
            if self.pixhawk_connected and self.pixhawk_armed:
                print("[SIM] [SHUTDOWN] Pixhawk disarmed successfully - SIMULATED")
                logging.info("[SHUTDOWN] Pixhawk disarmed successfully - SIMULATED")
                self.pixhawk_armed = False
            
            # Step 3: Close Pixhawk connection - SIMULATED
            logging.info("[SHUTDOWN] Step 3: Closing Pixhawk connection...")
            print("[SIM] [SHUTDOWN] Step 3: Closing Pixhawk connection...")
            # if self.pixhawk.connected:
            #     self.pixhawk.disconnect()
            if self.pixhawk_connected:
                print("[SIM] [SHUTDOWN] Pixhawk connection closed - SIMULATED")
                logging.info("[SHUTDOWN] Pixhawk connection closed - SIMULATED")
                self.pixhawk_connected = False
            
            # Step 4: Release camera
            logging.info("[SHUTDOWN] Step 4: Releasing camera...")
            print("[SIM] [SHUTDOWN] Step 4: Releasing camera...")
            if self.cap is not None:
                self.cap.release()
                logging.info("[SHUTDOWN] Camera released")
                print("[SIM] [SHUTDOWN] Camera released")
            
            # Step 5: Close OpenCV windows
            logging.info("[SHUTDOWN] Step 5: Closing OpenCV windows...")
            print("[SIM] [SHUTDOWN] Step 5: Closing OpenCV windows...")
            cv2.destroyAllWindows()
            logging.info("[SHUTDOWN] OpenCV windows closed")
            print("[SIM] [SHUTDOWN] OpenCV windows closed")
            
            logging.info("=" * 60)
            logging.info("SAFE SHUTDOWN COMPLETED (SIMULATION MODE)")
            logging.info("=" * 60)
            print("[SIM] ========================================")
            print("[SIM] SAFE SHUTDOWN COMPLETED (SIMULATION MODE)")
            print("[SIM] ========================================")
            
        except Exception as e:
            logging.error(f"[SHUTDOWN] Error during shutdown: {e}", exc_info=True)
            print(f"[SIM] [SHUTDOWN] Error during shutdown: {e}")
            # Still try to close critical resources
            try:
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
    print("\n[SIM] ========================================")
    print("[SIM] SIGINT (Ctrl+C) received - Initiating safe shutdown...")
    print("[SIM] ========================================")
    
    # Get the control system instance and shutdown safely
    if UUVControlSystemSim._instance is not None:
        UUVControlSystemSim._instance.shutdown()
    
    logging.info("Program terminated safely.")
    print("[SIM] Program terminated safely.")
    sys.exit(0)


def main():
    """Main function"""
    # Signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("=" * 60)
    print("UUV CONTROL SYSTEM - SIMULATION MODE")
    print("Pixhawk commands are commented out and simulated")
    print("=" * 60)
    
    # Connection string (not used in sim mode, but kept for reference)
    connection_string = 'udp:127.0.0.1:14551'
    
    # Initialize control system
    control_system = UUVControlSystemSim(
        connection_string=connection_string,
        camera_index=0,
        frame_width=640,
        frame_height=480
    )
    
    # Run
    control_system.run()


if __name__ == "__main__":
    main()

