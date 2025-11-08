"""
Lateral Kontrol Modülü
Y ekseni merkezleme için sağa/sola hareket (Channel 6)
"""

import logging
from .pid_controller import PIDController


class LateralController:
    """Lateral (sağ/sol) kontrol sınıfı"""
    
    def __init__(self, frame_width=640, frame_height=480):
        """
        Lateral kontrolcü başlat
        
        Args:
            frame_width: Görüntü genişliği
            frame_height: Görüntü yüksekliği
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Hedef merkez noktası (ekranın ortası)
        self.target_center_x = frame_width // 2
        
        # PID katsayıları (center.py'den alındı - X ekseni)
        self.x_pid = PIDController(
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
        self.lateral_pwm = 1500
        
        logging.info(f"[LATERAL] Kontrolcü başlatıldı - Hedef X: {self.target_center_x}px")
        logging.info(f"[LATERAL] PID: Kp={self.x_pid.kp}, Ki={self.x_pid.ki}, "
                    f"Kd={self.x_pid.kd}, Deadband={self.x_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Lateral kontrol sinyali hesapla
        
        Args:
            marker_info: Marker bilgileri (image_processor'dan)
            
        Returns:
            int: PWM değeri (1100-1900)
        """
        if not marker_info:
            self.lateral_pwm = self.neutral_pwm
            logging.debug("[LATERAL] Marker yok - Nötr pozisyon")
            return self.lateral_pwm
        
        info = marker_info[0]
        center_x, center_y = info['center']
        
        # X ekseni hatası: center_x - target_center_x
        # Pozitif: Marker sağda -> Sola git (PWM < 1500)
        # Negatif: Marker solda -> Sağa git (PWM > 1500)
        x_error = center_x - self.target_center_x
        
        # PID çıkışı
        pid_output = self.x_pid.compute(x_error)
        
        # PWM değerine çevir (ters yönde çalışıyor)
        # Pozitif hata -> Sola git -> PWM < 1500
        self.lateral_pwm = self.neutral_pwm - pid_output
        
        # PWM sınırları
        self.lateral_pwm = max(self.min_pwm, min(self.max_pwm, int(self.lateral_pwm)))
        
        # Log (sadece önemli değişikliklerde)
        if not self.x_pid.in_deadband:
            direction = 'SOL' if self.lateral_pwm < 1500 else 'SAG' if self.lateral_pwm > 1500 else 'ORTA'
            logging.debug(f"[LATERAL] Merkez X: {center_x}px, Hedef: {self.target_center_x}px, "
                         f"Hata: {x_error:.1f}px, PWM: {self.lateral_pwm}, Yön: {direction}")
        
        return self.lateral_pwm
    
    def get_status(self):
        """Kontrol durumu bilgisi"""
        direction = 'SOL' if self.lateral_pwm < 1500 else 'SAG' if self.lateral_pwm > 1500 else 'ORTA'
        return {
            'pwm': self.lateral_pwm,
            'direction': direction,
            'in_deadband': self.x_pid.in_deadband
        }

