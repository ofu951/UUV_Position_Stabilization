"""
Görüntü İşleme ve QR Kod Tespit Modülü
ArUco marker tespiti ve bilgi çıkarımı
"""

import cv2
import numpy as np
import time
import logging


class ImageProcessor:
    """Görüntü işleme ve marker tespit sınıfı"""
    
    def __init__(self, aruco_dict_type=cv2.aruco.DICT_4X4_50):
        """
        Görüntü işlemci başlat
        
        Args:
            aruco_dict_type: ArUco dictionary tipi
        """
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # FPS hesaplama
        self.frame_count = 0
        self.fps = 0
        self.start_time = time.time()
    
    def detect_markers(self, image):
        """
        Görüntüde marker tespit et
        
        Args:
            image: BGR görüntü
            
        Returns:
            tuple: (corners, ids) veya (None, None)
        """
        if image is None:
            return None, None
        
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        return corners, ids
    
    def calculate_marker_info(self, corners):
        """
        Marker bilgilerini hesapla
        
        Args:
            corners: Marker köşe noktaları
            
        Returns:
            list: Marker bilgileri listesi veya None
        """
        if corners is None or len(corners) == 0:
            return None
        
        box_info = []
        
        for corner in corners:
            points = corner[0].astype(np.float32)
            
            # Alan hesapla
            area = cv2.contourArea(points)
            
            # Kenar uzunluklarını hesapla
            edge_lengths = {}
            edge_names = ['UST', 'SAG', 'ALT', 'SOL']
            
            for j in range(4):
                p1 = points[j]
                p2 = points[(j + 1) % 4]
                distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                edge_lengths[edge_names[j]] = distance
            
            # Merkez noktası
            center = points.mean(axis=0).astype(int)
            
            box_info.append({
                'area': area,
                'edge_lengths': edge_lengths,
                'center': center,
                'points': points
            })
        
        return box_info
    
    def calculate_fps(self):
        """FPS hesapla"""
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
        Tespit sonuçlarını görüntü üzerine çiz
        
        Args:
            frame: Görüntü
            corners: Marker köşeleri
            ids: Marker ID'leri
            marker_info: Marker bilgileri
            
        Returns:
            frame: Çizilmiş görüntü
        """
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i, corner in enumerate(corners):
                center = corner[0].mean(axis=0).astype(int)
                
                # ID bilgisi
                cv2.putText(frame, f"ID: {ids[i][0]}", 
                          (center[0] - 30, center[1] - 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if marker_info and i < len(marker_info):
                    info = marker_info[i]
                    # Alan bilgisi
                    area_text = f"Area: {info['area']:.0f}px"
                    cv2.putText(frame, area_text, 
                              (center[0] - 40, center[1] + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame

