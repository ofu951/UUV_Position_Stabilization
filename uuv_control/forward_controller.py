"""
Forward/Backward Control Module
Forward/backward movement based on QR code area (Channel 5, X axis)
"""

import logging
from .pid_controller import PIDController


class ForwardController:
    """Forward/Backward control class"""
    
    def __init__(self, target_area=20000):
        """
        Initialize forward controller
        
        Args:
            target_area: Target pixel area (default: 20000)
        """
        # PID coefficients (from fwd_bwd.py)
        self.area_pid = PIDController(
            kp=0.02,           # Proportional coefficient
            ki=0.0005,         # Integral coefficient
            kd=0.01,           # Derivative coefficient
            max_output=200,
            min_output=-200,
            deadband=200       # 200px deadband
        )
        
        self.target_area = target_area
        
        # PWM values
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.forward_pwm = 1500
        
        logging.info(f"[FORWARD] Controller initialized - Target area: {target_area}px")
        logging.info(f"[FORWARD] PID: Kp={self.area_pid.kp}, Ki={self.area_pid.ki}, "
                    f"Kd={self.area_pid.kd}, Deadband={self.area_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Calculate forward/backward control signal
        
        Args:
            marker_info: Marker information (from image_processor)
            
        Returns:
            int: PWM value (1100-1900)
        """
        if not marker_info:
            self.forward_pwm = self.neutral_pwm
            logging.debug("[FORWARD] No marker - Neutral position")
            return self.forward_pwm
        
        info = marker_info[0]
        current_area = info['area']
        
        # Calculate error: target_area - current_area
        # Positive error: area small, move forward
        # Negative error: area large, move backward
        area_error = self.target_area - current_area
        
        # PID output
        pid_output = self.area_pid.compute(area_error)
        
        # Convert to PWM value
        # Positive PID output -> forward (PWM > 1500)
        # Negative PID output -> backward (PWM < 1500)
        self.forward_pwm = self.neutral_pwm + pid_output
        
        # PWM limits
        self.forward_pwm = max(self.min_pwm, min(self.max_pwm, int(self.forward_pwm)))
        
        # Log (only for significant changes)
        if not self.area_pid.in_deadband:
            direction = 'FORWARD' if self.forward_pwm > 1500 else 'BACKWARD' if self.forward_pwm < 1500 else 'NEUTRAL'
            logging.debug(f"[FORWARD] Area: {current_area:.0f}px, Error: {area_error:.0f}px, "
                         f"PWM: {self.forward_pwm}, Direction: {direction}")
        
        return self.forward_pwm
    
    def get_status(self):
        """Get control status information"""
        direction = 'FORWARD' if self.forward_pwm > 1500 else 'BACKWARD' if self.forward_pwm < 1500 else 'NEUTRAL'
        return {
            'pwm': self.forward_pwm,
            'direction': direction,
            'in_deadband': self.area_pid.in_deadband
        }
