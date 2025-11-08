import cv2
import numpy as np
import time
import logging
import signal
import sys
import os
from datetime import datetime

class PIDController:
    def __init__(self, kp, ki, kd, max_output=200, min_output=-200, deadband=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.deadband = deadband
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return 0
        
        # Deadband uygulaması
        if abs(error) <= self.deadband:
            error = 0
            self.integral = 0  # Deadband içinde integrali sıfırla
            
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Anti-windup
        if output > self.max_output:
            output = self.max_output
            self.integral -= error * dt
        elif output < self.min_output:
            output = self.min_output
            self.integral -= error * dt
        
        self.previous_error = error
        self.last_time = current_time
        
        return output

class YawController:
    def __init__(self):
        # Yaw kontrolü için PID katsayıları
        self.yaw_pid = PIDController(
            kp=5.0,            # ÇOK DAHA YÜKSEK oransal - kenar farkına hassas
            ki=0.025,           # Daha yüksek integral
            kd=1,             # Yüksek türev
            max_output=200, 
            min_output=-200,
            deadband=5         # 2px deadband
        )
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.yaw_pwm = 1500
        self.in_deadband = False
        
    def calculate_yaw_control(self, box_info):
        if not box_info:
            self.yaw_pwm = self.neutral_pwm
            self.in_deadband = False
            logging.warning("YAW - Marker tespit edilemedi - Nötr pozisyon")
            return
        
        info = box_info[0]
        edge_lengths = info['edge_lengths']
        
        # Sol ve sağ kenar uzunluklarını al
        left_edge = edge_lengths.get('SOL', 0)
        right_edge = edge_lengths.get('SAG', 0)
        
        # Yaw hatası: Sol ve sağ kenar farkı
        yaw_error = left_edge - right_edge
        
        # Deadband kontrolü
        was_in_deadband = self.in_deadband
        self.in_deadband = abs(yaw_error) <= self.yaw_pid.deadband
        
        # Deadband değişimini logla
        if was_in_deadband != self.in_deadband:
            if self.in_deadband:
                logging.info(f"YAW - DEADBAND AKTIF | Kenar Farkı: {yaw_error:.2f}px")
            else:
                logging.info(f"YAW - DEADBAND PASIF | Kenar Farkı: {yaw_error:.2f}px")
        
        yaw_output = self.yaw_pid.compute(yaw_error)
        self.yaw_pwm = self.neutral_pwm + yaw_output
        self.yaw_pwm = max(self.min_pwm, min(self.max_pwm, self.yaw_pwm))
        
        # Detaylı log
        direction = 'SAG' if self.yaw_pwm > 1500 else 'SOL' if self.yaw_pwm < 1500 else 'DUZ'
        logging.info(f"YAW - SOL:{left_edge:.1f}px, SAG:{right_edge:.1f}px, Fark:{yaw_error:.1f}px, "
                    f"PWM:{self.yaw_pwm}, Çıkış:{yaw_output:.1f}, Yön:{direction}, "
                    f"Deadband:{'EVET' if self.in_deadband else 'HAYIR'}")

class XYPositionController:
    def __init__(self, frame_width=640, frame_height=480):
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Hedef merkez noktası (ekranın tam ortası)
        self.target_center_x = frame_width // 2
        self.target_center_y = frame_height // 2
        
        # X ekseni kontrolü (Sağ/Sol hareket)
        self.x_pid = PIDController(
            kp=2,            # X pozisyonu için hassasiyet
            ki=0.02,          # Düşük integral
            kd=0.4,           # Düşük türev
            max_output=200,
            min_output=-200,
            deadband=15        # 15px deadband - merkeze yakın bölge
        )
        
        # Y ekseni kontrolü (Yukarı/Aşağı hareket)  
        self.y_pid = PIDController(
            kp=2,            # X pozisyonu için hassasiyet
            ki=0.02,          # Düşük integral
            kd=0.4,          # Düşük türev
            max_output=200,
            min_output=-200,
            deadband=15        # 15px deadband - merkeze yakın bölge
        )
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        
        # Mevcut PWM değerleri
        self.x_pwm = 1500  # Sağ/Sol hareketi
        self.y_pwm = 1500  # Yukarı/Aşağı hareketi
        
        # Kontrol durumu
        self.x_in_deadband = False
        self.y_in_deadband = False
        
    def calculate_xy_control(self, box_info):
        if not box_info:
            self.x_pwm = self.neutral_pwm
            self.y_pwm = self.neutral_pwm
            self.x_in_deadband = False
            self.y_in_deadband = False
            logging.warning("XY - Marker tespit edilemedi - Nötr pozisyon")
            return
        
        info = box_info[0]
        center_x, center_y = info['center']
        
        # X ekseni hatası (Sağ/Sol)
        x_error = center_x - self.target_center_x
        
        # Y ekseni hatası (Yukarı/Aşağı)
        y_error = center_y - self.target_center_y
        
        # Deadband kontrolü
        was_x_deadband = self.x_in_deadband
        was_y_deadband = self.y_in_deadband
        
        self.x_in_deadband = abs(x_error) <= self.x_pid.deadband
        self.y_in_deadband = abs(y_error) <= self.y_pid.deadband
        
        # Deadband değişimlerini logla
        if was_x_deadband != self.x_in_deadband:
            status = "AKTIF" if self.x_in_deadband else "PASIF"
            logging.info(f"XY-X - DEADBAND {status} | X Hatası: {x_error:.1f}px")
            
        if was_y_deadband != self.y_in_deadband:
            status = "AKTIF" if self.y_in_deadband else "PASIF"
            logging.info(f"XY-Y - DEADBAND {status} | Y Hatası: {y_error:.1f}px")
        
        # PID çıkışlarını hesapla
        x_output = self.x_pid.compute(x_error)
        y_output = self.y_pid.compute(y_error)
        
        # PWM değerlerini güncelle
        self.x_pwm = self.neutral_pwm - x_output  # Ters yönde çalışıyor
        self.y_pwm = self.neutral_pwm + y_output
        
        # PWM değerlerini sınırla
        self.x_pwm = max(self.min_pwm, min(self.max_pwm, self.x_pwm))
        self.y_pwm = max(self.min_pwm, min(self.max_pwm, self.y_pwm))
        
        # Detaylı log
        x_direction = 'SOL' if self.x_pwm < 1500 else 'SAG' if self.x_pwm > 1500 else 'ORTA'
        y_direction = 'YUKARI' if self.y_pwm > 1500 else 'ASAGI' if self.y_pwm < 1500 else 'ORTA'
        
        logging.info(f"XY - Merkez:({center_x},{center_y}), Hedef:({self.target_center_x},{self.target_center_y}), "
                    f"Hata:X:{x_error:.1f}px,Y:{y_error:.1f}px, "
                    f"PWM:X:{self.x_pwm}({x_direction}),Y:{self.y_pwm}({y_direction}), "
                    f"Çıkış:X:{x_output:.1f},Y:{y_output:.1f}, "
                    f"Deadband:X:{'EVET' if self.x_in_deadband else 'HAYIR'},Y:{'EVET' if self.y_in_deadband else 'HAYIR'}")

class CameraArucoDetector:
    def __init__(self, frame_width=640, frame_height=480):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.frame_count = 0
        self.fps = 0
        self.start_time = time.time()
        
    def detect_markers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        return corners, ids
    
    def calculate_bounding_box_info(self, corners):
        if corners is None or len(corners) == 0:
            return None
        
        box_info = []
        
        for i, corner in enumerate(corners):
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
    
    def draw_detection_results(self, frame, corners, ids, yaw_controller, xy_controller):
        # Hedef merkez çizgilerini çiz
        cv2.line(frame, (self.frame_width//2, 0), (self.frame_width//2, self.frame_height), (0, 255, 255), 1)
        cv2.line(frame, (0, self.frame_height//2), (self.frame_width, self.frame_height//2), (0, 255, 255), 1)
        cv2.circle(frame, (self.frame_width//2, self.frame_height//2), 5, (0, 255, 255), -1)
        
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            box_info = self.calculate_bounding_box_info(corners)
            
            for i, corner in enumerate(corners):
                center = corner[0].mean(axis=0).astype(int)
                
                # ID bilgisi
                cv2.putText(frame, f"ID: {ids[i][0]}", 
                          (center[0] - 30, center[1] - 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if box_info and i < len(box_info):
                    info = box_info[i]
                    edge_lengths = info['edge_lengths']
                    left_edge = edge_lengths.get('SOL', 0)
                    right_edge = edge_lengths.get('SAG', 0)
                    edge_diff = left_edge - right_edge
                    
                    # Merkez noktasını çiz
                    cv2.circle(frame, tuple(info['center']), 8, (255, 0, 0), -1)
                    
                    # Merkez koordinatları
                    center_x, center_y = info['center']
                    coord_text = f"Merkez: ({center_x}, {center_y})"
                    cv2.putText(frame, coord_text, 
                              (center[0] - 60, center[1] + 100),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)
                    
                    # Kontrol bilgileri
                    control_text = [
                        f"=== YAW KONTROL ===",
                        f"Kenar Farkı: {edge_diff:.1f}px",
                        f"Yaw PWM: {yaw_controller.yaw_pwm}",
                        f"Yaw Yön: {'SAG' if yaw_controller.yaw_pwm > 1500 else 'SOL' if yaw_controller.yaw_pwm < 1500 else 'DUZ'}",
                        f"Deadband: {'AKTIF' if yaw_controller.in_deadband else 'PASIF'}",
                        f"",
                        f"=== XY POZISYON KONTROL ===",
                        f"Merkez: ({center_x}, {center_y})",
                        f"Hedef: ({xy_controller.target_center_x}, {xy_controller.target_center_y})",
                        f"X PWM: {xy_controller.x_pwm} ({'SOL' if xy_controller.x_pwm < 1500 else 'SAG' if xy_controller.x_pwm > 1500 else 'ORTA'})",
                        f"Y PWM: {xy_controller.y_pwm} ({'YUKARI' if xy_controller.y_pwm > 1500 else 'ASAGI' if xy_controller.y_pwm < 1500 else 'ORTA'})",
                        f"X Deadband: {'AKTIF' if xy_controller.x_in_deadband else 'PASIF'}",
                        f"Y Deadband: {'AKTIF' if xy_controller.y_in_deadband else 'PASIF'}"
                    ]
                    
                    for k, text in enumerate(control_text):
                        color = (255, 255, 0)
                        if k in [1, 6, 7, 8]:  # Önemli değerler
                            color = (0, 255, 255)
                        elif k in [4, 9, 10]:  # Deadband durumu
                            color = (0, 255, 0) if 'AKTIF' in text else (0, 165, 255)
                            
                        cv2.putText(frame, text, (10, 400 + k * 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        else:
            # Marker yoksa bilgi göster
            control_text = [
                f"=== YAW KONTROL ===",
                f"Marker tespit edilemedi",
                f"Yaw PWM: {yaw_controller.yaw_pwm} (DUZ)",
                f"",
                f"=== XY POZISYON KONTROL ===",
                f"Marker tespit edilemedi",
                f"X PWM: {xy_controller.x_pwm} (ORTA)",
                f"Y PWM: {xy_controller.y_pwm} (ORTA)"
            ]
            
            for k, text in enumerate(control_text):
                cv2.putText(frame, text, (10, 400 + k * 25),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return frame
    
    def calculate_fps(self):
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time > 1.0:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = current_time
        
        return self.fps

def setup_logging():
    """Kapsamlı loglama sistemi kur"""
    # Log dosyası ismini zaman damgası ile oluştur
    log_filename = f"robot_control_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    
    # Log formatı
    log_format = '%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s'
    date_format = '%Y-%m-%d %H:%M:%S'
    
    # Log seviyesi
    logging.basicConfig(
        level=logging.INFO,
        format=log_format,
        datefmt=date_format,
        handlers=[
            logging.FileHandler(log_filename, encoding='utf-8'),
            logging.StreamHandler(sys.stdout)
        ]
    )
    
    logging.info(f"=== ROBOT KONTROL SISTEMI BASLATILDI ===")
    logging.info(f"Log dosyasi: {log_filename}")
    logging.info(f"Baslangic zamani: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logging.info("=" * 60)

def initialize_camera():
    logging.info("Kamera baslatiliyor...")
    
    for camera_index in [0, 1, 2, 3]:
        cap = cv2.VideoCapture(camera_index)
        
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            ret, frame = cap.read()
            if ret and frame is not None:
                logging.info(f"Kamera {camera_index} basariyla acildi - Cozunurluk: {frame.shape[1]}x{frame.shape[0]}")
                return cap
            else:
                cap.release()
                logging.warning(f"Kamera {camera_index} acildi ancak frame okunamadi")
    
    logging.error("Hicbir kamera acilamadi!")
    return None

def signal_handler(sig, frame):
    logging.info("Program sonlandiriliyor...")
    sys.exit(0)

def main():
    # Loglama sistemini kur
    setup_logging()
    
    # Signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    logging.info("3 EKSENLI ROBOT KONTROL SISTEMI BASLATILIYOR...")
    
    # Kontrolcüleri başlat
    yaw_controller = YawController()
    xy_controller = XYPositionController()
    detector = CameraArucoDetector()
    
    cap = initialize_camera()
    if cap is None:
        logging.error("Kamera baslatilamadi. Program sonlandiriliyor.")
        return
    
    logging.info("SISTEM BASARIYLA BASLATILDI!")
    logging.info("KONTROL EKSENLERI:")
    logging.info("1. YAW (Donus) - Sol/Sag kenar farki")
    logging.info("2. X (Sag/Sol) - Merkezin X koordinati") 
    logging.info("3. Y (Yukari/Asagi) - Merkezin Y koordinati")
    logging.info("HEDEF: Marker'i ekranin tam ortasina getirmek")
    logging.info("=" * 60)
    
    try:
        frame_counter = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                logging.error("Kameradan frame alinamadi!")
                break
            
            if frame is None:
                logging.warning("Bos frame alindi!")
                continue
            
            # Marker tespiti
            corners, ids = detector.detect_markers(frame)
            box_info = detector.calculate_bounding_box_info(corners)
            
            # Her 10 frame'de bir log (performans için)
            frame_counter += 1
            if frame_counter % 10 == 0:
                if box_info:
                    info = box_info[0]
                    center_x, center_y = info['center']
                    logging.debug(f"Frame {frame_counter}: Marker merkezi ({center_x}, {center_y})")
                else:
                    logging.debug(f"Frame {frame_counter}: Marker tespit edilemedi")
            
            # Kontrol hesaplamaları
            yaw_controller.calculate_yaw_control(box_info)
            xy_controller.calculate_xy_control(box_info)
            
            # Görselleştirme
            result_frame = detector.draw_detection_results(frame, corners, ids, yaw_controller, xy_controller)
            
            # FPS bilgisi
            fps = detector.calculate_fps()
            cv2.putText(result_frame, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow("3 EKSENLI ROBOT KONTROL SISTEMI", result_frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                logging.info("Kullanici cikis talebi...")
                break
                
    except KeyboardInterrupt:
        logging.info("Kullanici tarafindan durduruldu (Ctrl+C)")
    except Exception as e:
        logging.error(f"Beklenmeyen hata: {e}", exc_info=True)
    finally:
        logging.info("Kaynaklar temizleniyor...")
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()
        logging.info("=== PROGRAM SONLANDIRILDI ===")

if __name__ == "__main__":
    main()
