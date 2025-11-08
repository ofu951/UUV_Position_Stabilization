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

class ForwardController:
    def __init__(self):
        # Revize edilmiş PID katsayıları - daha yumuşak ve deadband'lı
        self.area_pid = PIDController(
            kp=0.02,           # Daha düşük oransal
            ki=0.0005,         # Daha düşük integral
            kd=0.01,           # Daha düşük türev
            max_output=200, 
            min_output=-200,
            deadband=200       # 200px deadband
        )
        
        # Hedef değerler
        self.target_area = 20000
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        
        # Mevcut PWM değerleri
        self.forward_pwm = 1500
        
        # Kontrol durumu
        self.in_deadband = False
        
        # Loglama ayarları
        self.setup_logging()
        
    def setup_logging(self):
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('forward_control.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger()
    
    def calculate_forward_control(self, box_info):
        if not box_info:
            self.forward_pwm = self.neutral_pwm
            self.logger.warning("Marker tespit edilemedi - Forward nötr")
            self.in_deadband = False
            return
        
        info = box_info[0]
        
        # Alan kontrolü (Forward/Backward)
        area_error = self.target_area - info['area']
        
        # Deadband durumunu kontrol et
        was_in_deadband = self.in_deadband
        self.in_deadband = abs(area_error) <= self.area_pid.deadband
        
        # Deadband'a giriş/çıkış logları
        if was_in_deadband != self.in_deadband:
            if self.in_deadband:
                self.logger.info(f"DEADBAND AKTIF - Hata: {area_error:.0f}px")
            else:
                self.logger.info(f"DEADBAND INACTIVE - Hata: {area_error:.0f}px")
        
        forward_output = self.area_pid.compute(area_error)
        self.forward_pwm = self.neutral_pwm + forward_output
        
        # PWM değerini sınırla
        self.forward_pwm = max(self.min_pwm, min(self.max_pwm, self.forward_pwm))
        
        # Log bilgileri
        self.log_control_info(info, area_error)
    
    def log_control_info(self, info, area_error):
        deadband_status = "DEADBAND" if self.in_deadband else "ACTIVE"
        direction = 'ILERI' if self.forward_pwm > 1500 else 'GERI' if self.forward_pwm < 1500 else 'NÖTR'
        
        log_message = (
            f"FORWARD CONTROL - "
            f"Area: {info['area']:.0f}px (err: {area_error:.0f}) | "
            f"Target: 20000px | "
            f"Forward PWM: {self.forward_pwm:.0f} | "
            f"PID Output: {self.forward_pwm - 1500:.1f} | "
            f"Status: {deadband_status} | "
            f"Direction: {direction}"
        )
        self.logger.info(log_message)

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
            
            box_info.append({
                'area': area,
                'edge_lengths': edge_lengths,
                'center': points.mean(axis=0).astype(int),
                'points': points
            })
        
        return box_info
    
    def draw_detection_results(self, frame, corners, ids, controller):
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
                    
                    # Alan bilgisi
                    area_text = f"Area: {info['area']:.0f}/20000"
                    cv2.putText(frame, area_text, 
                              (center[0] - 40, center[1] + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Hata ve deadband bilgisi
                    error = 20000 - info['area']
                    deadband_status = "DEADBAND" if abs(error) <= 200 else "ACTIVE"
                    error_color = (0, 255, 0) if abs(error) <= 200 else (0, 165, 255)
                    
                    # Forward kontrol bilgisi
                    control_text = [
                        f"Forward Control - ADIM 1",
                        f"Area Error: {error:.0f}px",
                        f"Deadband: {deadband_status}",
                        f"Forward PWM: {controller.forward_pwm}",
                        f"Direction: {'ILERI' if controller.forward_pwm > 1500 else 'GERI' if controller.forward_pwm < 1500 else 'NÖTR'}"
                    ]
                    
                    for k, text in enumerate(control_text):
                        color = (255, 255, 0)
                        if k == 2:  # Deadband satırı
                            color = (0, 255, 0) if deadband_status == "DEADBAND" else (0, 165, 255)
                        elif k == 1:  # Error satırı
                            color = error_color
                            
                        cv2.putText(frame, text, (10, 400 + k * 25),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Marker yoksa bilgi göster
        else:
            control_text = [
                f"Forward Control - ADIM 1",
                f"Marker tespit edilemedi",
                f"Forward PWM: {controller.forward_pwm} (NÖTR)",
                f"Status: WAITING FOR MARKER"
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
    
    print("ADIM 1: Forward Kontrol Sistemi Başlatılıyor...")
    
    # Kontrolcü ve dedektörü başlat
    controller = ForwardController()
    detector = CameraArucoDetector()
    
    # Kamerayı başlat
    cap = initialize_camera()
    if cap is None:
        print("Kamera başlatılamadı. Program sonlandırılıyor.")
        return
    
    print("\nSistem başarıyla başlatıldı!")
    print("Hedef Alan: 20000 px²")
    print("Kontrol Bilgileri:")
    print("- PWM Aralığı: 1100-1900")
    print("- Nötr PWM: 1500")
    print("- PID: Kp=0.02, Ki=0.0005, Kd=0.01")
    print("- Deadband: ±200px")
    print("- Status: Deadband içinde PWM değişimi DURUR")
    print("Çıkış için 'q' tuşuna basın veya Ctrl+C")
    print("=" * 50)
    
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
            
            # Forward kontrolü
            controller.calculate_forward_control(box_info)
            
            # Görselleştirme
            result_frame = detector.draw_detection_results(frame, corners, ids, controller)
            
            # FPS bilgisi
            fps = detector.calculate_fps()
            cv2.putText(result_frame, f"FPS: {fps:.1f}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Durum bilgisi
            status = f"Markers: {len(ids) if ids is not None else 0}"
            cv2.putText(result_frame, status, 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                       (0, 255, 0) if ids is not None else (0, 0, 255), 2)
            
            # PID ve Deadband bilgisi
            pid_info = f"PID: Kp=0.02, Deadband=±200px"
            cv2.putText(result_frame, pid_info, 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            cv2.imshow("ADIM 1 - Forward Control (Deadband Active)", result_frame)
            
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
