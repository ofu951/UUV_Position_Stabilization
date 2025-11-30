"""
Image Processing and QR Code Detection Module
ArUco marker detection and information extraction with pose estimation
"""

import cv2
import numpy as np
import time
import logging


class ImageProcessor:
    """Image processing and marker detection class with pose estimation"""
    
    def __init__(self, aruco_dict_type=cv2.aruco.DICT_4X4_50, 
                 marker_size=0.1, camera_matrix=None, dist_coeffs=None):
        """
        Initialize image processor
        
        Args:
            aruco_dict_type: ArUco dictionary type
            marker_size: Marker size in meters (default: 0.1m = 10cm)
            camera_matrix: Camera calibration matrix (3x3). If None, uses default values
            dist_coeffs: Distortion coefficients. If None, uses default values
        """
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Marker size in meters
        self.marker_size = marker_size
        
        # Camera calibration parameters
        # Default values for a typical webcam (can be calibrated for better accuracy)
        if camera_matrix is None:
            # Default camera matrix (assumes 640x480 resolution)
            # These values should be calibrated for your specific camera
            self.camera_matrix = np.array([
                [800.0, 0.0, 320.0],
                [0.0, 800.0, 240.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)
        else:
            self.camera_matrix = camera_matrix
        
        if dist_coeffs is None:
            # Default distortion coefficients (no distortion)
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        else:
            self.dist_coeffs = dist_coeffs
        
        # FPS calculation
        self.frame_count = 0
        self.fps = 0
        self.start_time = time.time()
        
        logging.info(f"[IMAGE_PROCESSOR] Initialized with marker_size={marker_size}m")
    
    def detect_markers(self, image):
        """
        Detect markers in image
        
        Args:
            image: BGR image
            
        Returns:
            tuple: (corners, ids) or (None, None)
        """
        if image is None:
            return None, None
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        return corners, ids
    
    def estimate_pose(self, corners, ids):
        """
        Estimate pose (position and orientation) of ArUco markers
        
        Args:
            corners: Marker corner points
            ids: Marker IDs
            
        Returns:
            list: List of pose dictionaries with x, y, z, roll, pitch, yaw or None
        """
        if corners is None or len(corners) == 0:
            return None
        
        poses = []
        
        for i, corner in enumerate(corners):
            # Estimate pose for single marker
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                corner, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            # Extract translation (position)
            tvec = tvec[0][0]  # Shape: (3,)
            x = float(tvec[0])
            y = float(tvec[1])
            z = float(tvec[2])
            
            # Extract rotation
            rvec = rvec[0][0]  # Shape: (3,)
            
            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            
            # Convert rotation matrix to Euler angles (roll, pitch, yaw)
            # Using ZYX convention (yaw-pitch-roll)
            sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
            
            singular = sy < 1e-6
            
            if not singular:
                roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            else:
                roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                yaw = 0
            
            # Convert to degrees
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)
            
            marker_id = int(ids[i][0]) if ids is not None and i < len(ids) else -1
            
            poses.append({
                'id': marker_id,
                'x': x,
                'y': y,
                'z': z,
                'roll': roll_deg,
                'pitch': pitch_deg,
                'yaw': yaw_deg,
                'roll_rad': roll,
                'pitch_rad': pitch,
                'yaw_rad': yaw,
                'rvec': rvec,
                'tvec': tvec,
                'rotation_matrix': rotation_matrix
            })
        
        return poses
    
    def calculate_marker_info(self, corners):
        """
        Calculate marker information (2D image-based info)
        
        Args:
            corners: Marker corner points
            
        Returns:
            list: Marker information list or None
        """
        if corners is None or len(corners) == 0:
            return None
        
        box_info = []
        
        for corner in corners:
            points = corner[0].astype(np.float32)
            
            # Calculate area
            area = cv2.contourArea(points)
            
            # Calculate edge lengths
            edge_lengths = {}
            edge_names = ['UST', 'SAG', 'ALT', 'SOL']
            
            for j in range(4):
                p1 = points[j]
                p2 = points[(j + 1) % 4]
                distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                edge_lengths[edge_names[j]] = distance
            
            # Center point
            center = points.mean(axis=0).astype(int)
            
            box_info.append({
                'area': area,
                'edge_lengths': edge_lengths,
                'center': center,
                'points': points
            })
        
        return box_info
    
    def calculate_fps(self):
        """Calculate FPS"""
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time > 1.0:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = current_time
        
        return self.fps
    
    def draw_detection(self, frame, corners, ids, marker_info=None):
        """
        Draw detection results on image
        
        Args:
            frame: Image
            corners: Marker corners
            ids: Marker IDs
            marker_info: Marker information
            
        Returns:
            frame: Drawn image
        """
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i, corner in enumerate(corners):
                center = corner[0].mean(axis=0).astype(int)
                
                # ID information
                cv2.putText(frame, f"ID: {ids[i][0]}", 
                          (center[0] - 30, center[1] - 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if marker_info and i < len(marker_info):
                    info = marker_info[i]
                    # Area information
                    area_text = f"Area: {info['area']:.0f}px"
                    cv2.putText(frame, area_text, 
                              (center[0] - 40, center[1] + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame
