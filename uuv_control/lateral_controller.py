"""
Lateral Control Module
Right/left movement for Y axis centering (Channel 6)
"""

import logging
from .pid_controller import PIDController


class LateralController:
    """Lateral (right/left) control class"""
    
    def __init__(self, frame_width=640, frame_height=480):
        """
        Initialize lateral controller
        
        Args:
            frame_width: Image width
            frame_height: Image height
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Target center point (screen center)
        self.target_center_x = frame_width // 2
        
        # PID coefficients (from center.py - X axis)
        self.x_pid = PIDController(
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
        self.lateral_pwm = 1500
        
        logging.info(f"[LATERAL] Controller initialized - Target X: {self.target_center_x}px")
        logging.info(f"[LATERAL] PID: Kp={self.x_pid.kp}, Ki={self.x_pid.ki}, "
                    f"Kd={self.x_pid.kd}, Deadband={self.x_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Calculate lateral control signal
        
        Args:
            marker_info: Marker information (from image_processor)
            
        Returns:
            int: PWM value (1100-1900)
        """
        if not marker_info:
            self.lateral_pwm = self.neutral_pwm
            logging.debug("[LATERAL] No marker - Neutral position")
            return self.lateral_pwm
        
        info = marker_info[0]
        center_x, center_y = info['center']
        
        # X axis error: center_x - target_center_x
        # Positive: Marker on right -> Move right (PWM > 1500)
        # Negative: Marker on left -> Move left (PWM < 1500)
        x_error = center_x - self.target_center_x
        
        # PID output
        pid_output = self.x_pid.compute(x_error)
        
        # Convert to PWM value
        # Positive error -> Move right -> PWM > 1500
        self.lateral_pwm = self.neutral_pwm + pid_output
        
        # PWM limits
        self.lateral_pwm = max(self.min_pwm, min(self.max_pwm, int(self.lateral_pwm)))
        
        # Log (only for significant changes)
        if not self.x_pid.in_deadband:
            direction = 'LEFT' if self.lateral_pwm < 1500 else 'RIGHT' if self.lateral_pwm > 1500 else 'CENTER'
            logging.debug(f"[LATERAL] Center X: {center_x}px, Target: {self.target_center_x}px, "
                         f"Error: {x_error:.1f}px, PWM: {self.lateral_pwm}, Direction: {direction}")
        
        return self.lateral_pwm
    
    def get_status(self):
        """Get control status information"""
        direction = 'LEFT' if self.lateral_pwm < 1500 else 'RIGHT' if self.lateral_pwm > 1500 else 'CENTER'
        return {
            'pwm': self.lateral_pwm,
            'direction': direction,
            'in_deadband': self.x_pid.in_deadband
        }
