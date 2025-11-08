"""
Throttle Kontrol Modülü
Z ekseni (yukarı/aşağı) hareket için throttle kontrolü (Channel 3)
"""

import logging
from .pid_controller import PIDController


class ThrottleController:
    """Throttle (batma/çıkma) kontrol sınıfı"""
    
    def __init__(self, frame_width=640, frame_height=480):
        """
        Throttle kontrolcü başlat
        
        Args:
            frame_width: Görüntü genişliği
            frame_height: Görüntü yüksekliği
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Hedef merkez noktası (ekranın ortası)
        self.target_center_y = frame_height // 2
        
        # PID katsayıları (center.py'den alındı - Y ekseni)
        self.y_pid = PIDController(
            kp=2.0,            # Oransal katsayı
            ki=0.02,           # İntegral katsayı
            kd=0.4,            # Türev katsayı
            max_output=200,
            min_output=-200,
            deadband=15        # 15px deadband
        )
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.throttle_pwm = 1500
        
        logging.info(f"[THROTTLE] Kontrolcü başlatıldı - Hedef Y: {self.target_center_y}px")
        logging.info(f"[THROTTLE] PID: Kp={self.y_pid.kp}, Ki={self.y_pid.ki}, "
                    f"Kd={self.y_pid.kd}, Deadband={self.y_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Throttle kontrol sinyali hesapla
        
        Args:
            marker_info: Marker bilgileri (image_processor'dan)
            
        Returns:
            int: PWM değeri (1100-1900)
        """
        if not marker_info:
            self.throttle_pwm = self.neutral_pwm
            logging.debug("[THROTTLE] Marker yok - Nötr pozisyon")
            return self.throttle_pwm
        
        info = marker_info[0]
        center_x, center_y = info['center']
        
        # Y ekseni hatası: center_y - target_center_y
        # Pozitif: Marker aşağıda -> Yukarı çık (PWM > 1500)
        # Negatif: Marker yukarıda -> Aşağı in (PWM < 1500)
        y_error = center_y - self.target_center_y
        
        # PID çıkışı
        pid_output = self.y_pid.compute(y_error)
        
        # PWM değerine çevir
        # Pozitif hata -> Yukarı çık -> PWM > 1500
        self.throttle_pwm = self.neutral_pwm + pid_output
        
        # PWM sınırları
        self.throttle_pwm = max(self.min_pwm, min(self.max_pwm, int(self.throttle_pwm)))
        
        # Log (sadece önemli değişikliklerde)
        if not self.y_pid.in_deadband:
            direction = 'YUKARI' if self.throttle_pwm > 1500 else 'ASAGI' if self.throttle_pwm < 1500 else 'ORTA'
            logging.debug(f"[THROTTLE] Merkez Y: {center_y}px, Hedef: {self.target_center_y}px, "
                         f"Hata: {y_error:.1f}px, PWM: {self.throttle_pwm}, Yön: {direction}")
        
        return self.throttle_pwm
    
    def get_status(self):
        """Kontrol durumu bilgisi"""
        direction = 'YUKARI' if self.throttle_pwm > 1500 else 'ASAGI' if self.throttle_pwm < 1500 else 'ORTA'
        return {
            'pwm': self.throttle_pwm,
            'direction': direction,
            'in_deadband': self.y_pid.in_deadband
        }

