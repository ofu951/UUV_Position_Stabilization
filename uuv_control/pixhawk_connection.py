"""
Pixhawk Bağlantı ve Temel Komutlar Modülü
Pixhawk'a bağlanma, arm/disarm işlemleri
"""

from pymavlink import mavutil
import time
import logging


class PixhawkConnection:
    """Pixhawk bağlantı ve temel komutlar sınıfı"""
    
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        """
        Pixhawk bağlantısını başlat
        
        Args:
            connection_string: Bağlantı string'i (örn: 'udp:127.0.0.1:14551', 
                              'tcp:192.168.1.100:5760', '/dev/ttyUSB0')
        """
        self.connection_string = connection_string
        self.master = None
        self.connected = False
        self.armed = False
        
    def connect(self, timeout=10):
        """
        Pixhawk'a bağlan
        
        Args:
            timeout: Bağlantı timeout süresi (saniye)
            
        Returns:
            bool: Bağlantı başarılı ise True
        """
        try:
            logging.info(f"[PIXHAWK] Bağlantı kuruluyor: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string)
            
            # Heartbeat bekle
            logging.info("[PIXHAWK] Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=timeout)
            
            logging.info(f"[PIXHAWK] Bağlantı başarılı! System={self.master.target_system}, "
                        f"Component={self.master.target_component}")
            self.connected = True
            return True
            
        except Exception as e:
            logging.error(f"[PIXHAWK] Bağlantı hatası: {e}")
            self.connected = False
            return False
    
    def arm(self, force_arm=False):
        """
        Pixhawk'ı ARM et
        
        Args:
            force_arm: Zorla arm etme (bazı firmware'lerde 21196 gerekebilir)
            
        Returns:
            bool: Arm başarılı ise True
        """
        if not self.connected or self.master is None:
            logging.error("[PIXHAWK] Bağlantı yok! Önce connect() çağrılmalı.")
            return False
        
        try:
            logging.info("[PIXHAWK] ARM isteği gönderiliyor...")
            
            # Force arm parametresi
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
            
            # ARM durumunu kontrol et
            t0 = time.time()
            while time.time() - t0 < 5.0:
                hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if hb is not None:
                    armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if armed:
                        logging.info("[PIXHAWK] >>> ARAÇ ARM OLDU <<<")
                        self.armed = True
                        return True
            
            logging.warning("[PIXHAWK] 5 saniye içinde arm olmadı. "
                          "Güvenlik kilidi veya mode izin vermiyor olabilir.")
            return False
            
        except Exception as e:
            logging.error(f"[PIXHAWK] ARM hatası: {e}")
            return False
    
    def disarm(self):
        """
        Pixhawk'ı DISARM et
        
        Returns:
            bool: Disarm başarılı ise True
        """
        if not self.connected or self.master is None:
            logging.error("[PIXHAWK] Bağlantı yok!")
            return False
        
        try:
            logging.info("[PIXHAWK] DISARM isteği gönderiliyor...")
            
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,              # param1: 0 = disarm
                0,
                0, 0, 0, 0, 0
            )
            
            # Disarm durumunu kontrol et
            t0 = time.time()
            while time.time() - t0 < 3.0:
                hb = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                if hb is not None:
                    armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if not armed:
                        logging.info("[PIXHAWK] >>> ARAÇ DISARM OLDU <<<")
                        self.armed = False
                        return True
            
            logging.warning("[PIXHAWK] Disarm durumu kontrol edilemedi.")
            self.armed = False
            return True  # Yine de True döndür çünkü komut gönderildi
            
        except Exception as e:
            logging.error(f"[PIXHAWK] DISARM hatası: {e}")
            return False
    
    def send_rc_override(self, channels):
        """
        RC kanallarına override sinyali gönder
        
        Args:
            channels: 8 elemanlı liste [ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8]
                    0 = ignore (kanal override edilmez)
                    Diğer değerler = PWM değeri (1100-1900)
        """
        if not self.connected or self.master is None:
            logging.error("[PIXHAWK] Bağlantı yok!")
            return False
        
        # 8 kanal için varsayılan değerler
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
            logging.error(f"[PIXHAWK] RC override hatası: {e}")
            return False
    
    def get_attitude(self):
        """
        Mevcut attitude bilgisini al
        
        Returns:
            dict: {'roll': rad, 'pitch': rad, 'yaw': rad} veya None
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
            logging.error(f"[PIXHAWK] Attitude okuma hatası: {e}")
        
        return None
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.master is not None:
            # RC override'ı sıfırla
            self.send_rc_override([0] * 8)
            # Disarm et
            if self.armed:
                self.disarm()
            self.master = None
            self.connected = False
            logging.info("[PIXHAWK] Bağlantı kapatıldı.")

