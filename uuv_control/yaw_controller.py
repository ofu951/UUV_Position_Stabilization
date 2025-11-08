"""
Yaw Control Module
Yaw control based on left/right edge difference (Channel 4)
"""

import logging
from .pid_controller import PIDController


class YawController:
    """Yaw control class"""
    
    def __init__(self):
        """Initialize yaw controller"""
        # PID coefficients (from yaw.py and center.py)
        self.yaw_pid = PIDController(
            kp=5.0,            # High proportional coefficient
            ki=0.025,          # Integral coefficient
            kd=1.0,            # Derivative coefficient
            max_output=200,
            min_output=-200,
            deadband=2         # 2px deadband (very sensitive)
        )
        
        # PWM values
        self.neutral_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900
        self.yaw_pwm = 1500
        
        logging.info("[YAW] Controller initialized")
        logging.info(f"[YAW] PID: Kp={self.yaw_pid.kp}, Ki={self.yaw_pid.ki}, "
                    f"Kd={self.yaw_pid.kd}, Deadband={self.yaw_pid.deadband}px")
    
    def calculate_control(self, marker_info):
        """
        Calculate yaw control signal
        
        Args:
            marker_info: Marker information (from image_processor)
            
        Returns:
            int: PWM value (1100-1900)
        """
        if not marker_info:
            self.yaw_pwm = self.neutral_pwm
            logging.debug("[YAW] No marker - Neutral position")
            return self.yaw_pwm
        
        info = marker_info[0]
        edge_lengths = info['edge_lengths']
        
        # Left and right edge lengths
        left_edge = edge_lengths.get('SOL', 0)
        right_edge = edge_lengths.get('SAG', 0)
        
        # Yaw error: Right - Left edge difference
        # Positive: Right edge longer -> Turn right (PWM > 1500)
        # Negative: Left edge longer -> Turn left (PWM < 1500)
        yaw_error = right_edge - left_edge
        
        # PID output
        pid_output = self.yaw_pid.compute(yaw_error)
        
        # Convert to PWM value
        self.yaw_pwm = self.neutral_pwm + pid_output
        
        # PWM limits
        self.yaw_pwm = max(self.min_pwm, min(self.max_pwm, int(self.yaw_pwm)))
        
        # Log (only for significant changes)
        if not self.yaw_pid.in_deadband:
            direction = 'RIGHT' if self.yaw_pwm > 1500 else 'LEFT' if self.yaw_pwm < 1500 else 'STRAIGHT'
            logging.debug(f"[YAW] Left: {left_edge:.1f}px, Right: {right_edge:.1f}px, "
                         f"Diff: {yaw_error:.1f}px, PWM: {self.yaw_pwm}, Direction: {direction}")
        
        return self.yaw_pwm
    
    def get_status(self):
        """Get control status information"""
        direction = 'RIGHT' if self.yaw_pwm > 1500 else 'LEFT' if self.yaw_pwm < 1500 else 'STRAIGHT'
        return {
            'pwm': self.yaw_pwm,
            'direction': direction,
            'in_deadband': self.yaw_pid.in_deadband
        }
