"""
Throttle Control Module
Throttle control for Z axis (up/down) movement (Channel 3)
"""

import logging
from .pid_controller import PIDController


class ThrottleController:
    """Throttle (dive/surface) control class"""
    
    def __init__(self, frame_width=640, frame_height=480):
        """
        Initialize throttle controller
        
        Args:
            frame_width: Image width
            frame_height: Image height
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Target center point (screen center)
        self.target_center_y = frame_height // 2
        
        # PID coefficients (from center.py - Y axis)
        self.y_pid = PIDController(
            kp=2.0,            # Proportional coefficient
            ki=0.02,           # Integral coefficient
            kd=0.4,            # Derivative coefficient
            max_output=200,
            min_output=-200,
            deadband=15        # 15px deadband
        )
        
        # PWM values
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.throttle_pwm = 1500
        
        logging.info(f"[THROTTLE] Controller initialized - Target Y: {self.target_center_y}px")
        logging.info(f"[THROTTLE] PID: Kp={self.y_pid.kp}, Ki={self.y_pid.ki}, "
                    f"Kd={self.y_pid.kd}, Deadband={self.y_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Calculate throttle control signal
        
        Args:
            marker_info: Marker information (from image_processor)
            
        Returns:
            int: PWM value (1100-1900)
        """
        if not marker_info:
            self.throttle_pwm = self.neutral_pwm
            logging.debug("[THROTTLE] No marker - Neutral position")
            return self.throttle_pwm
        
        info = marker_info[0]
        center_x, center_y = info['center']
        
        # Y axis error: center_y - target_center_y
        # Positive: Marker below -> Move down (PWM < 1500)
        # Negative: Marker above -> Move up (PWM > 1500)
        y_error = center_y - self.target_center_y
        
        # PID output
        pid_output = self.y_pid.compute(y_error)
        
        # Convert to PWM value
        # Positive error -> Move down -> PWM < 1500
        # Negative error -> Move up -> PWM > 1500
        self.throttle_pwm = self.neutral_pwm - pid_output
        
        # PWM limits
        self.throttle_pwm = max(self.min_pwm, min(self.max_pwm, int(self.throttle_pwm)))
        
        # Log (only for significant changes)
        if not self.y_pid.in_deadband:
            direction = 'UP' if self.throttle_pwm > 1500 else 'DOWN' if self.throttle_pwm < 1500 else 'CENTER'
            logging.debug(f"[THROTTLE] Center Y: {center_y}px, Target: {self.target_center_y}px, "
                         f"Error: {y_error:.1f}px, PWM: {self.throttle_pwm}, Direction: {direction}")
        
        return self.throttle_pwm
    
    def get_status(self):
        """Get control status information"""
        direction = 'UP' if self.throttle_pwm > 1500 else 'DOWN' if self.throttle_pwm < 1500 else 'CENTER'
        return {
            'pwm': self.throttle_pwm,
            'direction': direction,
            'in_deadband': self.y_pid.in_deadband
        }
