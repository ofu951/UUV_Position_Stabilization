"""
PID Controller Module
Base class for all PID controllers
"""

import time
import logging


class PIDController:
    """General purpose PID controller"""
    
    def __init__(self, kp, ki, kd, max_output=200, min_output=-200, deadband=0):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional coefficient
            ki: Integral coefficient
            kd: Derivative coefficient
            max_output: Maximum output value
            min_output: Minimum output value
            deadband: Deadband value (error is considered 0 within this range)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.deadband = deadband
        
        # PID state
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.in_deadband = False
    
    def compute(self, error):
        """
        Calculate PID output
        
        Args:
            error: Error value
            
        Returns:
            float: PID output value
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.0001  # Minimum delta time
        
        # Deadband control
        was_in_deadband = self.in_deadband
        if abs(error) <= self.deadband:
            error = 0
            self.integral = 0  # Reset integral within deadband
            self.in_deadband = True
        else:
            self.in_deadband = False
        
        # Log if deadband status changed
        if was_in_deadband != self.in_deadband:
            status = "ACTIVE" if self.in_deadband else "INACTIVE"
            logging.debug(f"PID Deadband {status} - Error: {error:.2f}")
        
        # Calculate integral
        self.integral += error * dt
        
        # Calculate derivative
        derivative = (error - self.previous_error) / dt
        
        # PID output
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Anti-windup: Correct integral if output exceeds limits
        if output > self.max_output:
            output = self.max_output
            self.integral -= error * dt  # Prevent integral windup
        elif output < self.min_output:
            output = self.min_output
            self.integral -= error * dt  # Prevent integral windup
        
        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.in_deadband = False
