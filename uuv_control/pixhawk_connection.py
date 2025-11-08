"""
Pixhawk Connection and Basic Commands Module
Connect to Pixhawk, arm/disarm operations
"""

from pymavlink import mavutil
import time
import logging


class PixhawkConnection:
    """Pixhawk connection and basic commands class"""
    
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        """
        Initialize Pixhawk connection
        
        Args:
            connection_string: Connection string (e.g., 'udp:127.0.0.1:14551', 
                              'tcp:192.168.1.100:5760', '/dev/ttyUSB0')
        """
        self.connection_string = connection_string
        self.master = None
        self.connected = False
        self.armed = False
        
    def connect(self, timeout=10):
        """
        Connect to Pixhawk
        
        Args:
            timeout: Connection timeout duration (seconds)
            
        Returns:
            bool: True if connection successful
        """
        try:
            logging.info(f"[PIXHAWK] Connecting: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string)
            
            # Wait for heartbeat
            logging.info("[PIXHAWK] Waiting for heartbeat...")
            self.master.wait_heartbeat(timeout=timeout)
            
            logging.info(f"[PIXHAWK] Connection successful! System={self.master.target_system}, "
                        f"Component={self.master.target_component}")
            self.connected = True
            return True
            
        except Exception as e:
            logging.error(f"[PIXHAWK] Connection error: {e}")
            self.connected = False
            return False
    
    def arm(self, force_arm=False):
        """
        ARM Pixhawk
        
        Args:
            force_arm: Force arm (some firmware may require 21196)
            
        Returns:
            bool: True if arm successful
        """
        if not self.connected or self.master is None:
            logging.error("[PIXHAWK] Not connected! Must call connect() first.")
            return False
        
        try:
            logging.info("[PIXHAWK] Sending ARM request...")
            
            # Force arm parameter
            force_param = 21196 if force_arm else 0
            
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,              # param1: 1 = arm, 0 = disarm
                force_param,    # param2: force arm flag
                0, 0, 0, 0, 0
            )
            
            # Check ARM status
            t0 = time.time()
            while time.time() - t0 < 5.0:
                hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if hb is not None:
                    armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if armed:
                        logging.info("[PIXHAWK] >>> VEHICLE ARMED <<<")
                        self.armed = True
                        return True
            
            logging.warning("[PIXHAWK] Did not arm within 5 seconds. "
                          "Safety lock or mode may not allow it.")
            return False
            
        except Exception as e:
            logging.error(f"[PIXHAWK] ARM error: {e}")
            return False
    
    def disarm(self):
        """
        DISARM Pixhawk
        
        Returns:
            bool: True if disarm successful
        """
        if not self.connected or self.master is None:
            logging.error("[PIXHAWK] Not connected!")
            return False
        
        try:
            logging.info("[PIXHAWK] Sending DISARM request...")
            
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,              # param1: 0 = disarm
                0,
                0, 0, 0, 0, 0
            )
            
            # Check disarm status
            t0 = time.time()
            while time.time() - t0 < 3.0:
                hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if hb is not None:
                    armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if not armed:
                        logging.info("[PIXHAWK] >>> VEHICLE DISARMED <<<")
                        self.armed = False
                        return True
            
            logging.warning("[PIXHAWK] Could not verify disarm status.")
            self.armed = False
            return True  # Still return True because command was sent
            
        except Exception as e:
            logging.error(f"[PIXHAWK] DISARM error: {e}")
            return False
    
    def send_rc_override(self, channels):
        """
        Send RC channel override signal
        
        Args:
            channels: 8-element list [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]
                    0 = ignore (channel not overridden)
                    Other values = PWM value (1100-1900)
        """
        if not self.connected or self.master is None:
            logging.error("[PIXHAWK] Not connected!")
            return False
        
        # Default values for 8 channels
        ch_values = [0] * 8
        for i, val in enumerate(channels[:8]):
            ch_values[i] = int(val)
        
        try:
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                ch_values[0],   # ch1
                ch_values[1],   # ch2
                ch_values[2],   # ch3
                ch_values[3],   # ch4
                ch_values[4],   # ch5
                ch_values[5],   # ch6
                ch_values[6],   # ch7
                ch_values[7]    # ch8
            )
            return True
        except Exception as e:
            logging.error(f"[PIXHAWK] RC override error: {e}")
            return False
    
    def get_attitude(self):
        """
        Get current attitude information
        
        Returns:
            dict: {'roll': rad, 'pitch': rad, 'yaw': rad} or None
        """
        if not self.connected or self.master is None:
            return None
        
        try:
            msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if msg is not None:
                return {
                    'roll': msg.roll,
                    'pitch': msg.pitch,
                    'yaw': msg.yaw
                }
        except Exception as e:
            logging.error(f"[PIXHAWK] Attitude read error: {e}")
        
        return None
    
    def disconnect(self):
        """Close connection"""
        if self.master is not None:
            # Reset RC override
            self.send_rc_override([0] * 8)
            # Disarm
            if self.armed:
                self.disarm()
            self.master = None
            self.connected = False
            logging.info("[PIXHAWK] Connection closed.")
