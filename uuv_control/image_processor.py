"""
Image Processing and QR Code Detection Module
ArUco marker detection and information extraction
"""

import cv2
import numpy as np
import time
import logging


class ImageProcessor:
    """Image processing and marker detection class"""
    
    def __init__(self, aruco_dict_type=cv2.aruco.DICT_4X4_50):
        """
        Initialize image processor
        
        Args:
            aruco_dict_type: ArUco dictionary type
        """
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # FPS calculation
        self.frame_count = 0
        self.fps = 0
        self.start_time = time.time()
    
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
    
    def calculate_marker_info(self, corners):
        """
        Calculate marker information
        
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
