"""
Yaw Kontrol Modülü
Sol/sağ kenar farkına göre yaw kontrolü (Channel 4)
"""

import logging
from .pid_controller import PIDController


class YawController:
    """Yaw kontrol sınıfı"""
    
    def __init__(self):
        """Yaw kontrolcü başlat"""
        # PID katsayıları (yaw.py ve center.py'den alındı)
        self.yaw_pid = PIDController(
            kp=5.0,            # Yüksek oransal katsayı
            ki=0.025,          # İntegral katsayı
            kd=1.0,            # Türev katsayı
            max_output=200,
            min_output=-200,
            deadband=2         # 2px deadband (çok hassas)
        )
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.yaw_pwm = 1500
        
        logging.info("[YAW] Kontrolcü başlatıldı")
        logging.info(f"[YAW] PID: Kp={self.yaw_pid.kp}, Ki={self.yaw_pid.ki}, "
                    f"Kd={self.yaw_pid.kd}, Deadband={self.yaw_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Yaw kontrol sinyali hesapla
        
        Args:
            marker_info: Marker bilgileri (image_processor'dan)
            
        Returns:
            int: PWM değeri (1100-1900)
        """
        if not marker_info:
            self.yaw_pwm = self.neutral_pwm
            logging.debug("[YAW] Marker yok - Nötr pozisyon")
            return self.yaw_pwm
        
        info = marker_info[0]
        edge_lengths = info['edge_lengths']
        
        # Sol ve sağ kenar uzunlukları
        left_edge = edge_lengths.get('SOL', 0)
        right_edge = edge_lengths.get('SAG', 0)
        
        # Yaw hatası: Sol - Sağ kenar farkı
        # Pozitif: Sol kenar daha uzun -> Sağa dön (PWM > 1500)
        # Negatif: Sağ kenar daha uzun -> Sola dön (PWM < 1500)
        yaw_error = left_edge - right_edge
        
        # PID çıkışı
        pid_output = self.yaw_pid.compute(yaw_error)
        
        # PWM değerine çevir
        self.yaw_pwm = self.neutral_pwm + pid_output
        
        # PWM sınırları
        self.yaw_pwm = max(self.min_pwm, min(self.max_pwm, int(self.yaw_pwm)))
        
        # Log (sadece önemli değişikliklerde)
        if not self.yaw_pid.in_deadband:
            direction = 'SAG' if self.yaw_pwm > 1500 else 'SOL' if self.yaw_pwm < 1500 else 'DUZ'
            logging.debug(f"[YAW] Sol: {left_edge:.1f}px, Sağ: {right_edge:.1f}px, "
                         f"Fark: {yaw_error:.1f}px, PWM: {self.yaw_pwm}, Yön: {direction}")
        
        return self.yaw_pwm
    
    def get_status(self):
        """Kontrol durumu bilgisi"""
        direction = 'SAG' if self.yaw_pwm > 1500 else 'SOL' if self.yaw_pwm < 1500 else 'DUZ'
        return {
            'pwm': self.yaw_pwm,
            'direction': direction,
            'in_deadband': self.yaw_pid.in_deadband
        }

