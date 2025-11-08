"""
Forward/Backward Kontrol Modülü
QR kod alanına göre ileri/geri hareket (Channel 5, X ekseni)
"""

import logging
from .pid_controller import PIDController


class ForwardController:
    """Forward/Backward kontrol sınıfı"""
    
    def __init__(self, target_area=20000):
        """
        Forward kontrolcü başlat
        
        Args:
            target_area: Hedef pixel alanı (default: 20000)
        """
        # PID katsayıları (fwd_bwd.py'den alındı)
        self.area_pid = PIDController(
            kp=0.02,           # Oransal katsayı
            ki=0.0005,         # İntegral katsayı
            kd=0.01,           # Türev katsayı
            max_output=200,
            min_output=-200,
            deadband=200       # 200px deadband
        )
        
        self.target_area = target_area
        
        # PWM değerleri
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.forward_pwm = 1500
        
        logging.info(f"[FORWARD] Kontrolcü başlatıldı - Hedef alan: {target_area}px")
        logging.info(f"[FORWARD] PID: Kp={self.area_pid.kp}, Ki={self.area_pid.ki}, "
                    f"Kd={self.area_pid.kd}, Deadband={self.area_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Forward/Backward kontrol sinyali hesapla
        
        Args:
            marker_info: Marker bilgileri (image_processor'dan)
            
        Returns:
            int: PWM değeri (1100-1900)
        """
        if not marker_info:
            self.forward_pwm = self.neutral_pwm
            logging.debug("[FORWARD] Marker yok - Nötr pozisyon")
            return self.forward_pwm
        
        info = marker_info[0]
        current_area = info['area']
        
        # Hata hesapla: target_area - current_area
        # Pozitif hata: alan küçük, ileri git (forward)
        # Negatif hata: alan büyük, geri git (backward)
        area_error = self.target_area - current_area
        
        # PID çıkışı
        pid_output = self.area_pid.compute(area_error)
        
        # PWM değerine çevir
        # PID çıkışı pozitif ise ileri (PWM > 1500)
        # PID çıkışı negatif ise geri (PWM < 1500)
        self.forward_pwm = self.neutral_pwm + pid_output
        
        # PWM sınırları
        self.forward_pwm = max(self.min_pwm, min(self.max_pwm, int(self.forward_pwm)))
        
        # Log (sadece önemli değişikliklerde)
        if not self.area_pid.in_deadband:
            direction = 'ILERI' if self.forward_pwm > 1500 else 'GERI' if self.forward_pwm < 1500 else 'NÖTR'
            logging.debug(f"[FORWARD] Alan: {current_area:.0f}px, Hata: {area_error:.0f}px, "
                         f"PWM: {self.forward_pwm}, Yön: {direction}")
        
        return self.forward_pwm
    
    def get_status(self):
        """Kontrol durumu bilgisi"""
        direction = 'ILERI' if self.forward_pwm > 1500 else 'GERI' if self.forward_pwm < 1500 else 'NÖTR'
        return {
            'pwm': self.forward_pwm,
            'direction': direction,
            'in_deadband': self.area_pid.in_deadband
        }

