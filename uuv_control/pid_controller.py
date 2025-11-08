"""
PID Controller Modülü
Tüm PID kontrolcüleri için temel sınıf
"""

import time
import logging


class PIDController:
    """Genel amaçlı PID kontrolcü"""
    
    def __init__(self, kp, ki, kd, max_output=200, min_output=-200, deadband=0):
        """
        PID kontrolcü başlat
        
        Args:
            kp: Oransal katsayı
            ki: İntegral katsayı
            kd: Türev katsayı
            max_output: Maksimum çıkış değeri
            min_output: Minimum çıkış değeri
            deadband: Deadband değeri (bu aralıkta hata 0 kabul edilir)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.deadband = deadband
        
        # PID durumu
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.in_deadband = False
    
    def compute(self, error):
        """
        PID çıkışını hesapla
        
        Args:
            error: Hata değeri
            
        Returns:
            float: PID çıkış değeri
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.0001  # Minimum delta time
        
        # Deadband kontrolü
        was_in_deadband = self.in_deadband
        if abs(error) <= self.deadband:
            error = 0
            self.integral = 0  # Deadband içinde integrali sıfırla
            self.in_deadband = True
        else:
            self.in_deadband = False
        
        # Deadband durumu değiştiyse logla
        if was_in_deadband != self.in_deadband:
            status = "AKTIF" if self.in_deadband else "PASIF"
            logging.debug(f"PID Deadband {status} - Hata: {error:.2f}")
        
        # İntegral hesapla
        self.integral += error * dt
        
        # Türev hesapla
        derivative = (error - self.previous_error) / dt
        
        # PID çıkışı
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Anti-windup: Çıkış sınırlarını aşarsa integrali düzelt
        if output > self.max_output:
            output = self.max_output
            self.integral -= error * dt  # Integral windup önleme
        elif output < self.min_output:
            output = self.min_output
            self.integral -= error * dt  # Integral windup önleme
        
        # Durumu güncelle
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID durumunu sıfırla"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.in_deadband = False

