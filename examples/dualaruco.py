#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import time
import math


class Kalman1D:
    def __init__(self, q=0.01, r=0.1):
        self.q = q
        self.r = r
        self.x = 0.0
        self.p = 1.0

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x


def rvec_tvec_to_transform(rvec, tvec):
    """Convert rvec, tvec (OpenCV) -> 4x4 homogeneous transform"""
    rot_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = rot_mat
    T[:3, 3] = tvec.flatten()
    return T


def invert_transform(T):
    """Invert a 4x4 homogeneous transform"""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def main():
    rospy.init_node('aruco_multi_pose_kalman_node')
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    detect_pub = rospy.Publisher('/aruco_detect', Int8, queue_size=10)
    pose_pub = rospy.Publisher('/aruco/pose_xyz_yaw', Vector3, queue_size=10)

    # Optional: enable visualization for debugging (set to True if you have a display)
    ENABLE_DISPLAY = rospy.get_param('~enable_display', False)
    
    # ✅ NEW: Add diagnostic mode to see what markers are actually detected
    DIAGNOSTIC_MODE = rospy.get_param('~diagnostic_mode', True)

    # Log system info
    rospy.loginfo(f"🤖 Running on ROS Noetic | Ubuntu 20.04 | Python {rospy.get_param('/rospy/python_version', '3.8')}")
    rospy.loginfo(f"📦 OpenCV version: {cv2.__version__}")

    # --- Camera setup for Raspberry Pi 4 + Ubuntu 20.04 (USB camera with V4L2) ---
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    
    # ✅ IMPROVED: Better camera settings for ArUco detection
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus if available
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Enable auto exposure
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    time.sleep(1.5)  # allow camera to warm up and adjust exposure

    if not cap.isOpened():
        rospy.logerr("❌ Could not open USB camera (/dev/video0)")
        return

    # Verify actual camera settings
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    rospy.loginfo(f"✅ Camera opened: {int(actual_width)}x{int(actual_height)} @ {int(actual_fps)} FPS")

    bridge = CvBridge()
    rate = rospy.Rate(30)

    # ✅ ArUco setup - Optimized for OpenCV 4.2.0 (ROS Noetic default)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    
    # Optimized detection parameters for OpenCV 4.2.0
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.adaptiveThreshConstant = 7
    parameters.minMarkerPerimeterRate = 0.03
    parameters.maxMarkerPerimeterRate = 4.0
    parameters.polygonalApproxAccuracyRate = 0.05
    parameters.minCornerDistanceRate = 0.05
    parameters.minDistanceToBorder = 3
    parameters.markerBorderBits = 1
    parameters.minOtsuStdDev = 5.0
    
    rospy.loginfo("✅ ArUco detector parameters configured for OpenCV 4.2.0")

    # Camera intrinsics scaled from 800x600 to 640x360
    camera_matrix = np.array([
        [258.08721534, 0.0, 319.75089206],
        [0.0, 194.59767625, 180.1979814],
        [0.0, 0.0, 1.0]
    ])
    dist_coeffs = np.array([-0.09758372, 0.01368855, -0.00415818, -0.00602915, -0.00266812])
    marker_length = 0.096  # 96mm markers

    rospy.loginfo(f"📷 Camera matrix (640x360):\n{camera_matrix}")
    rospy.loginfo(f"📏 Marker size: {marker_length*1000:.0f}mm")

    # Kalman filters - tuned for better response
    kalman_x = Kalman1D(q=0.005, r=0.05)
    kalman_y = Kalman1D(q=0.005, r=0.05)
    kalman_yaw = Kalman1D(q=0.01, r=0.1)

    # Define world poses of markers
    marker_world_poses = {
        100: np.eye(4),
        0: np.array([[0, -1, 0, 1.0], [1, 0, 0, 0.0], [0, 0, 1, 0.0], [0, 0, 0, 1]]),
        7: np.array([[-1, 0, 0, 1.0], [0, -1, 0, 1.0], [0, 0, 1, 0.0], [0, 0, 0, 1]]),
        23: np.array([[0, 1, 0, 0.0], [-1, 0, 0, 1.0], [0, 0, 1, 0.0], [0, 0, 0, 1]])
    }

    target_ids = [100, 0, 7, 23]
    rospy.loginfo(f"🎯 Tracking markers: {target_ids}")

    # ✅ Track all detected marker IDs for diagnostics
    all_detected_ids = set()
    marker_detection_counts = {}

    # Detection monitoring
    frame_count = 0
    detection_count = 0
    last_diagnostic_log = 0

    rospy.loginfo("🚀 Starting ArUco detection loop...")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("⚠️ Failed to grab frame from USB camera")
            continue

        frame_count += 1

        # ✅ IMPROVED: Apply preprocessing for better detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        
        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        detected = False
        positions = []
        yaws = []
        detected_ids = []

        # ✅ Diagnostic - log ALL detected markers
        if DIAGNOSTIC_MODE and ids is not None:
            for marker_id in ids.flatten():
                all_detected_ids.add(int(marker_id))
                marker_detection_counts[int(marker_id)] = marker_detection_counts.get(int(marker_id), 0) + 1

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in target_ids:
                    detected = True
                    detected_ids.append(marker_id)
                    
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        [corners[i]], marker_length, camera_matrix, dist_coeffs
                    )
                    rvec = rvec[0][0]
                    tvec = tvec[0][0]

                    # Camera relative to marker
                    T_cam_marker = rvec_tvec_to_transform(rvec, tvec)
                    T_marker_cam = invert_transform(T_cam_marker)

                    # Camera in world
                    T_marker_world = marker_world_poses[marker_id]
                    T_cam_world = T_marker_world @ T_marker_cam

                    cam_pos = T_cam_world[:3, 3]
                    yaw = math.atan2(T_cam_world[1, 0], T_cam_world[0, 0])

                    positions.append(cam_pos[:2])
                    yaws.append(yaw)

                    # Draw on frame for debugging
                    if ENABLE_DISPLAY:
                        cv2.aruco.drawDetectedMarkers(frame, [corners[i]], np.array([marker_id]))
                        cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            if positions:
                detection_count += 1
                
                # Average position
                avg_pos = np.mean(positions, axis=0)
                # Average yaw with sin/cos
                avg_yaw = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))

                # Apply Kalman
                filtered_x = kalman_x.update(avg_pos[0])
                filtered_y = kalman_y.update(avg_pos[1])
                filtered_yaw = kalman_yaw.update(avg_yaw)

                pose_msg = Vector3(filtered_x, filtered_y, filtered_yaw)
                pose_pub.publish(pose_msg)

                # Log position every 30 frames (~1 second)
                if frame_count % 30 == 0:
                    rospy.loginfo(f"📍 Markers {detected_ids} | Pos: ({filtered_x:.3f}, {filtered_y:.3f}) | Yaw: {math.degrees(filtered_yaw):.1f}°")

        detect_pub.publish(Int8(1 if detected else 0))

        # ✅ Detailed diagnostic logging every 5 seconds
        if DIAGNOSTIC_MODE and (frame_count - last_diagnostic_log) >= 150:
            detection_rate = (detection_count / frame_count) * 100
            rospy.loginfo(f"📊 Detection rate: {detection_rate:.1f}% ({detection_count}/{frame_count} frames)")
            
            if all_detected_ids:
                rospy.loginfo(f"🔍 All markers ever seen: {sorted(all_detected_ids)}")
                
                # Show which target markers are being detected
                target_detected = [mid for mid in target_ids if mid in all_detected_ids]
                target_missing = [mid for mid in target_ids if mid not in all_detected_ids]
                
                if target_detected:
                    rospy.loginfo(f"✅ Target markers detected: {target_detected}")
                if target_missing:
                    rospy.logwarn(f"❌ Target markers NOT detected: {target_missing}")
                    
                # Show detection counts
                target_counts = {k: v for k, v in marker_detection_counts.items() if k in target_ids}
                if target_counts:
                    rospy.loginfo(f"📈 Target marker counts: {target_counts}")
            else:
                rospy.logwarn("⚠️ NO MARKERS DETECTED AT ALL!")
                rospy.logwarn("   Troubleshooting checklist:")
                rospy.logwarn("   1. Are ArUco markers visible to camera?")
                rospy.logwarn("   2. Are they 4x4 markers from DICT_4X4_50?")
                rospy.logwarn("   3. Is lighting adequate (not too dark/bright)?")
                rospy.logwarn("   4. Are markers printed clearly (not blurry)?")
                rospy.logwarn("   5. Is camera focused properly?")
                rospy.logwarn("   6. Are markers large enough in frame?")
            
            # Show rejected candidates count
            if rejected is not None and len(rejected) > 0:
                rospy.loginfo(f"⚠️ Rejected candidates: {len(rejected)} (close but not valid markers)")
            
            last_diagnostic_log = frame_count

        # Publish ROS image
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_pub.publish(ros_image)

        # Optional display for debugging
        if ENABLE_DISPLAY:
            # Draw detection info on frame
            info_y = 30
            if ids is not None:
                cv2.putText(frame, f"Detected: {len(ids)} markers", (10, info_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                info_y += 30
                for marker_id in ids.flatten():
                    color = (0, 255, 0) if marker_id in target_ids else (0, 165, 255)
                    cv2.putText(frame, f"ID: {marker_id}", (10, info_y),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    info_y += 25
            else:
                cv2.putText(frame, "No markers detected", (10, info_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Show detection rate
            detection_rate = (detection_count / frame_count) * 100 if frame_count > 0 else 0
            cv2.putText(frame, f"Rate: {detection_rate:.1f}%", (10, frame.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("ArUco Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rate.sleep()

    cap.release()
    if ENABLE_DISPLAY:
        cv2.destroyAllWindows()
    
    final_rate = (detection_count/frame_count)*100 if frame_count > 0 else 0
    rospy.loginfo(f"🏁 Node shutdown. Final detection rate: {final_rate:.1f}%")
    if DIAGNOSTIC_MODE and all_detected_ids:
        rospy.loginfo(f"🔍 Summary - All markers ever detected: {sorted(all_detected_ids)}")
        rospy.loginfo(f"📊 Final detection counts: {marker_detection_counts}")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass