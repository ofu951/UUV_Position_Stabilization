"""
Ana Kontrol Scripti
Tüm modülleri birleştirerek robot stabilizasyonu sağlar
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
    """Ana robot kontrol sistemi"""
    
    def __init__(self, connection_string='udp:127.0.0.1:14551', 
                 camera_index=0, frame_width=640, frame_height=480):
        """
        Kontrol sistemi başlat
        
        Args:
            connection_string: Pixhawk bağlantı string'i
            camera_index: Kamera index'i
            frame_width: Görüntü genişliği
            frame_height: Görüntü yüksekliği
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Pixhawk bağlantısı
        self.pixhawk = PixhawkConnection(connection_string)
        
        # Görüntü işleme
        self.image_processor = ImageProcessor()
        
        # Kontrolcüler
        self.forward_controller = ForwardController(target_area=20000)
        self.yaw_controller = YawController()
        self.lateral_controller = LateralController(frame_width, frame_height)
        self.throttle_controller = ThrottleController(frame_width, frame_height)
        
        # Kamera
        self.camera_index = camera_index
        self.cap = None
        
        # Çalışma durumu
        self.running = False
        
        # Loglama ayarları
        self.setup_logging()
    
    def setup_logging(self):
        """Loglama sistemini kur"""
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
        logging.info(f"Log dosyası: {log_filename}")
        logging.info(f"Başlangıç zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        logging.info("=" * 60)
    
    def initialize_camera(self):
        """Kamerayı başlat"""
        logging.info("Kamera başlatılıyor...")
        
        for idx in [self.camera_index, 0, 1, 2, 3]:
            cap = cv2.VideoCapture(idx)
            
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                cap.set(cv2.CAP_PROP_FPS, 30)
                
                ret, frame = cap.read()
                if ret and frame is not None:
                    logging.info(f"Kamera {idx} başarıyla açıldı - "
                               f"Çözünürlük: {frame.shape[1]}x{frame.shape[0]}")
                    self.cap = cap
                    return True
                else:
                    cap.release()
        
        logging.error("Hiçbir kamera açılamadı!")
        return False
    
    def connect_pixhawk(self):
        """Pixhawk'a bağlan ve arm et"""
        logging.info("Pixhawk bağlantısı başlatılıyor...")
        
        if not self.pixhawk.connect():
            logging.error("Pixhawk bağlantısı başarısız!")
            return False
        
        logging.info("Pixhawk arm ediliyor...")
        if not self.pixhawk.arm():
            logging.warning("Pixhawk arm edilemedi, ancak devam ediliyor...")
        
        return True
    
    def draw_control_info(self, frame, corners, ids, marker_info):
        """Kontrol bilgilerini görüntü üzerine çiz"""
        # Hedef merkez çizgileri
        cv2.line(frame, (self.frame_width//2, 0), 
                (self.frame_width//2, self.frame_height), (0, 255, 255), 1)
        cv2.line(frame, (0, self.frame_height//2), 
                (self.frame_width, self.frame_height//2), (0, 255, 255), 1)
        cv2.circle(frame, (self.frame_width//2, self.frame_height//2), 5, (0, 255, 255), -1)
        
        # Marker çizimi
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            if marker_info:
                info = marker_info[0]
                center = tuple(info['center'])
                
                # Merkez noktasını çiz
                cv2.circle(frame, center, 8, (255, 0, 0), -1)
                
                # ID ve alan bilgisi
                cv2.putText(frame, f"ID: {ids[0][0]}", 
                          (center[0] - 30, center[1] - 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Area: {info['area']:.0f}px", 
                          (center[0] - 40, center[1] + 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Kontrol bilgileri
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
            
            cv2.putText(frame, text, (10, y_offset + i * 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # FPS bilgisi
        fps = self.image_processor.calculate_fps()
        cv2.putText(frame, f"FPS: {fps:.1f}", 
                   (self.frame_width - 120, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return frame
    
    def run(self):
        """Ana kontrol döngüsü"""
        logging.info("Kontrol sistemi başlatılıyor...")
        
        # Kamerayı başlat
        if not self.initialize_camera():
            logging.error("Kamera başlatılamadı!")
            return
        
        # Pixhawk'a bağlan
        if not self.connect_pixhawk():
            logging.error("Pixhawk bağlantısı başarısız!")
            return
        
        logging.info("=" * 60)
        logging.info("KONTROL SİSTEMİ HAZIR!")
        logging.info("Kanal Atamaları:")
        logging.info("  Channel 3 (Throttle): Z ekseni (Yukarı/Aşağı)")
        logging.info("  Channel 4 (Yaw): Yaw kontrolü (Sol/Sağ)")
        logging.info("  Channel 5 (Forward): X ekseni (İleri/Geri)")
        logging.info("  Channel 6 (Lateral): Y ekseni (Sağ/Sol)")
        logging.info("=" * 60)
        logging.info("Çıkış için 'q' tuşuna basın veya Ctrl+C")
        logging.info("")
        
        self.running = True
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logging.warning("Frame okunamadı!")
                    continue
                
                # Marker tespiti
                corners, ids = self.image_processor.detect_markers(frame)
                marker_info = self.image_processor.calculate_marker_info(corners)
                
                # Kontrol sinyallerini hesapla
                forward_pwm = self.forward_controller.calculate_control(marker_info)
                yaw_pwm = self.yaw_controller.calculate_control(marker_info)
                lateral_pwm = self.lateral_controller.calculate_control(marker_info)
                throttle_pwm = self.throttle_controller.calculate_control(marker_info)
                
                # Pixhawk'a PWM sinyalleri gönder
                # Channel mapping:
                # Ch1: Roll (0 = ignore)
                # Ch2: Pitch (0 = ignore)
                # Ch3: Throttle (batma/çıkma)
                # Ch4: Yaw
                # Ch5: Forward (ileri/geri)
                # Ch6: Lateral (sağ/sol)
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
                
                # Görselleştirme
                result_frame = self.draw_control_info(frame, corners, ids, marker_info)
                cv2.imshow("UUV Control System", result_frame)
                
                # Çıkış kontrolü
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logging.info("Kullanıcı çıkış talebi...")
                    break
                
                # Kısa bekleme (performans için)
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logging.info("Ctrl+C ile durduruldu")
        except Exception as e:
            logging.error(f"Beklenmeyen hata: {e}", exc_info=True)
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Sistemi kapat"""
        logging.info("Sistem kapatılıyor...")
        self.running = False
        
        # RC override'ı sıfırla
        if self.pixhawk.connected:
            self.pixhawk.send_rc_override([0] * 8)
            self.pixhawk.disarm()
            self.pixhawk.disconnect()
        
        # Kamerayı kapat
        if self.cap is not None:
            self.cap.release()
        
        cv2.destroyAllWindows()
        logging.info("Sistem kapatıldı.")


def signal_handler(sig, frame):
    """Signal handler"""
    logging.info("Program sonlandırılıyor...")
    sys.exit(0)


def main():
    """Ana fonksiyon"""
    # Signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Bağlantı string'i (gerekirse değiştirilebilir)
    # Örnekler:
    # - 'udp:127.0.0.1:14551' (SITL simülasyonu)
    # - 'tcp:192.168.1.100:5760' (TCP bağlantı)
    # - '/dev/ttyUSB0' (USB seri bağlantı - Linux)
    # - 'COM3' (Windows seri bağlantı)
    connection_string = 'udp:127.0.0.1:14551'
    
    # Kontrol sistemini başlat
    control_system = UUVControlSystem(
        connection_string=connection_string,
        camera_index=0,
        frame_width=640,
        frame_height=480
    )
    
    # Çalıştır
    control_system.run()


if __name__ == "__main__":
    main()

