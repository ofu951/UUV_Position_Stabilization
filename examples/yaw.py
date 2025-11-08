import cv2
import numpy as np
import time
import logging
import signal
import sys

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
        # Yaw kontrolü için GÜÇLENDİRİLMİŞ PID katsayıları
        self.yaw_pid = PIDController(
            kp=5.0,            # ÇOK DAHA YÜKSEK oransal - kenar farkına hassas
            ki=0.025,           # Daha yüksek integral
            kd=1,            # Daha yüksek türev
            max_output=200, 
            min_output=-200,
            deadband=2         # SADECE 2px deadband - çok hassas
        )
        
        # Hedef değer (sol ve sağ kenarlar eşit olmalı)
        self.target_balance = 0
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        
        # Mevcut PWM değerleri
        self.yaw_pwm = 1500
        
        # Kontrol durumu
        self.in_deadband = False
        
        # Loglama ayarları
        self.setup_logging()
        
    def setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('yaw_control.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
    def calculate_yaw_control(self, box_info):
        if not box_info:
            self.yaw_pwm = self.neutral_pwm
            self.in_deadband = False
            logging.warning("Marker tespit edilemedi - Yaw nötr")
            return
        
        info = box_info[0]
        edge_lengths = info['edge_lengths']
        
        # Sol ve sağ kenar uzunluklarını al
        left_edge = edge_lengths.get('SOL', 0)
        right_edge = edge_lengths.get('SAG', 0)
        
        # Yaw hatası: Sol ve sağ kenar farkı
        # Pozitif: Sol kenar daha uzun (sağa dön)
        # Negatif: Sağ kenar daha uzun (sola dön)
        yaw_error = left_edge - right_edge
        
        # Deadband durumunu kontrol et
        was_in_deadband = self.in_deadband
        self.in_deadband = abs(yaw_error) <= self.yaw_pid.deadband
        
        # Deadband'a giriş/çıkış logları
        if was_in_deadband != self.in_deadband:
            if self.in_deadband:
                logging.info(f"YAW DEADBAND AKTIF - Kenar Farkı: {yaw_error:.2f}px")
            else:
                logging.info(f"YAW DEADBAND INACTIVE - Kenar Farkı: {yaw_error:.2f}px")
        
        yaw_output = self.yaw_pid.compute(yaw_error)
        self.yaw_pwm = self.neutral_pwm + yaw_output
        
        # PWM değerini sınırla
        self.yaw_pwm = max(self.min_pwm, min(self.max_pwm, self.yaw_pwm))
        
        # Log bilgileri
        self.log_control_info(left_edge, right_edge, yaw_error)
    
    def log_control_info(self, left_edge, right_edge, yaw_error):
        deadband_status = "DEADBAND" if self.in_deadband else "ACTIVE"
        direction = 'SAG' if self.yaw_pwm > 1500 else 'SOL' if self.yaw_pwm < 1500 else 'DUZ'
        pid_output = self.yaw_pwm - 1500
        
        log_message = (
            f"YAW CONTROL - "
            f"SOL: {left_edge:.2f}px, SAG: {right_edge:.2f}px | "
            f"Fark: {yaw_error:.2f}px | "
            f"Yaw PWM: {self.yaw_pwm} | "
            f"PID Output: {pid_output:.1f} | "
            f"Status: {deadband_status} | "
            f"Direction: {direction}"
        )
        logging.info(log_message)

class CameraArucoDetector:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
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
            
            # Alan hesapla (sadece bilgi için)
            area = cv2.contourArea(points)
            
            # Kenar uzunluklarını hesapla - YAW kontrolü için
            edge_lengths = {}
            edge_names = ['UST', 'SAG', 'ALT', 'SOL']
            
            for j in range(4):
                p1 = points[j]
                p2 = points[(j + 1) % 4]
                distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                edge_lengths[edge_names[j]] = distance
            
            box_info.append({
                'area': area,
                'edge_lengths': edge_lengths,
                'center': points.mean(axis=0).astype(int),
                'points': points
            })
        
        return box_info
    
    def draw_detection_results(self, frame, corners, ids, yaw_controller):
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
                    
                    # Alan bilgisi (sadece info)
                    area_text = f"Area: {info['area']:.0f}px"
                    cv2.putText(frame, area_text, 
                              (center[0] - 40, center[1] + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Kenar uzunlukları bilgisi - DETAYLI
                    edge_text_y = center[1] + 45
                    edge_texts = [
                        f"SOL: {left_edge:.2f}px",
                        f"SAG: {right_edge:.2f}px", 
                        f"FARK: {edge_diff:.2f}px"
                    ]
                    
                    for j, text in enumerate(edge_texts):
                        color = (255, 200, 0)
                        if j == 2:  # Fark satırı
                            color = (0, 255, 0) if abs(edge_diff) <= 2 else (0, 165, 255)
                        cv2.putText(frame, text, 
                                  (center[0] - 50, edge_text_y + j * 20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    # Yaw kontrol bilgisi - AYRINTILI
                    yaw_deadband_status = "DEADBAND" if abs(edge_diff) <= 2 else "ACTIVE"
                    yaw_direction = 'SAG' if yaw_controller.yaw_pwm > 1500 else 'SOL' if yaw_controller.yaw_pwm < 1500 else 'DUZ'
                    pid_output = yaw_controller.yaw_pwm - 1500
                    
                    control_text = [
                        f"=== YAW KONTROL SISTEMI ===",
                        f"Kenar Farkı: {edge_diff:.2f}px",
                        f"Deadband: {yaw_deadband_status} (±2px)",
                        f"PID Çıkış: {pid_output:.1f}",
                        f"Yaw PWM: {yaw_controller.yaw_pwm}",
                        f"Yön: {yaw_direction}",
                        f"",
                        f"PID Katsayıları:",
                        f"Kp=2.0, Ki=0.01, Kd=0.5"
                    ]
                    
                    for k, text in enumerate(control_text):
                        color = (255, 255, 0)  # Default color
                        
                        if k == 1:  # Kenar Farkı
                            color = (0, 255, 0) if abs(edge_diff) <= 2 else (0, 165, 255)
                        elif k == 2:  # Deadband
                            color = (0, 255, 0) if yaw_deadband_status == "DEADBAND" else (0, 165, 255)
                        elif k == 3:  # PID Çıkış
                            color = (0, 255, 255) if abs(pid_output) > 0 else (255, 255, 255)
                        elif k == 4:  # Yaw PWM
                            color = (0, 255, 0) if yaw_controller.yaw_pwm == 1500 else (0, 165, 255)
                        elif k == 5:  # Yön
                            color = (0, 255, 0) if yaw_direction == 'DUZ' else (0, 165, 255)
                            
                        cv2.putText(frame, text, (10, 400 + k * 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Marker yoksa bilgi göster
        else:
            control_text = [
                f"=== YAW KONTROL SISTEMI ===",
                f"Marker tespit edilemedi",
                f"Yaw PWM: {yaw_controller.yaw_pwm} (DUZ)",
                f"Status: WAITING FOR MARKER",
                f"",
                f"PID Katsayıları:",
                f"Kp=2.0, Ki=0.01, Kd=0.5"
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

def initialize_camera():
    print("Kamera başlatılıyor...")
    
    # Farklı kamera index'lerini dene
    for camera_index in [0, 1, 2, 3]:
        cap = cv2.VideoCapture(camera_index)
        
        if cap.isOpened():
            # Kamera özelliklerini ayarla
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 30)
            
            # Kameranın çalıştığını test et
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"Kamera {camera_index} başarıyla açıldı")
                print(f"Çerçeve boyutu: {frame.shape[1]}x{frame.shape[0]}")
                return cap
            else:
                cap.release()
        else:
            print(f"Kamera {camera_index} açılamadı")
    
    print("Hata: Hiçbir kamera açılamadı!")
    return None

def signal_handler(sig, frame):
    print("\nProgram sonlandırılıyor...")
    sys.exit(0)

def main():
    # Signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    print("YAW KONTROL SİSTEMİ BAŞLATILIYOR...")
    
    # Yaw kontrolcüsünü başlat
    yaw_controller = YawController()
    detector = CameraArucoDetector()
    
    # Kamerayı başlat
    cap = initialize_camera()
    if cap is None:
        print("Kamera başlatılamadı. Program sonlandırılıyor.")
        return
    
    print("\nYAW KONTROL SİSTEMİ BAŞARIYLA BAŞLATILDI!")
    print("SİSTEM ÖZELLİKLERİ:")
    print("- Kontrol Tipi: Yaw (Sağ/Sol Dönüş)")
    print("- Sensör: Sol ve Sağ kenar uzunluk farkı")
    print("- Hedef: Kenar farkı = 0 (SOL = SAG)")
    print("")
    print("PID KATSAYILARI (GÜÇLENDİRİLMİŞ):")
    print("- Kp: 2.0     (Yüksek hassasiyet)")
    print("- Ki: 0.01    (Orta integral)") 
    print("- Kd: 0.5     (Yüksek türev)")
    print("- Deadband: ±2px (Çok hassas)")
    print("")
    print("PWM AYARLARI:")
    print("- PWM Aralığı: 1100-1900")
    print("- Nötr PWM: 1500 (Düz)")
    print("- Sol PWM: 1100-1499 (Sola dönüş)")
    print("- Sağ PWM: 1501-1900 (Sağa dönüş)")
    print("")
    print("Çıkış için 'q' tuşuna basın veya Ctrl+C")
    print("=" * 60)
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Kameradan frame alınamadı!")
                break
            
            # Frame'i kontrol et
            if frame is None:
                print("Boş frame alındı!")
                continue
            
            # Marker tespiti
            corners, ids = detector.detect_markers(frame)
            box_info = detector.calculate_bounding_box_info(corners)
            
            # Yaw kontrolü
            yaw_controller.calculate_yaw_control(box_info)
            
            # Görselleştirme
            result_frame = detector.draw_detection_results(frame, corners, ids, yaw_controller)
            
            # FPS bilgisi
            fps = detector.calculate_fps()
            cv2.putText(result_frame, f"FPS: {fps:.1f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Durum bilgisi
            status = f"Markers: {len(ids) if ids is not None else 0}"
            cv2.putText(result_frame, status, 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                       (0, 255, 0) if ids is not None else (0, 0, 255), 2)
            
            # Sistem başlığı
            cv2.putText(result_frame, "YAW KONTROL SISTEMI - SOL/SAG KENAR FARKI", 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.imshow("YAW KONTROL SISTEMI - Sol/Sag Kenar Farki", result_frame)
            
            # Çıkış kontrolü
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Kullanıcı çıkış talebi...")
                break
                
    except KeyboardInterrupt:
        print("\nKullanıcı tarafından durduruldu (Ctrl+C)")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Kaynaklar temizleniyor...")
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()
        print("Program sonlandırıldı.")

if __name__ == "__main__":
    main()
