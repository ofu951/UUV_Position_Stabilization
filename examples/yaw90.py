from pymavlink import mavutil
import math
import time

#####################
# KULLANICI AYARLARI
#####################

TARGET_YAW_DEG = 90.0  # Robotun tutmasını istediğimiz yön (derece)
Kp = 4.0
Ki = 0.0   # ilk güvenli test için integral kapalı
Kd = 1.0

# RC override mapping:
# 1500 -> nötr
# 1900 -> max sağ
# 1100 -> max sol

#####################
# HELPER FONKSİYONLAR
#####################

def angle_error_deg(target, current):
    """
    açı farkını -180..180 bandına indir
    """
    error = target - current
    while error > 180:
        error -= 360
    while error < -180:
        error += 360
    return error

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error):
        now = time.time()
        if self.prev_time is None:
            dt = 0.02
        else:
            dt = now - self.prev_time
            if dt <= 0:
                dt = 0.0001

        # integral (windup'ı şimdilik çok takmıyoruz çünkü Ki=0)
        self.integrator += error * dt

        derivative = (error - self.prev_error) / dt

        output = (self.Kp * error) + (self.Ki * self.integrator) + (self.Kd * derivative)

        self.prev_error = error
        self.prev_time = now
        return output

def yaw_cmd_to_rc(yaw_cmd_norm):
    """
    yaw_cmd_norm: -1.0 .. +1.0
      -1.0 -> 1100
       0.0 -> 1500
      +1.0 -> 1900
    """
    if yaw_cmd_norm > 1.0:
        yaw_cmd_norm = 1.0
    if yaw_cmd_norm < -1.0:
        yaw_cmd_norm = -1.0

    rc_val = 1500 + yaw_cmd_norm * 400.0
    return int(rc_val)

def send_rc_override(master, ch4_val):
    """
    Sadece ch4 (yaw) override ediyoruz.
    Diğer kanallara 0 gönderiyoruz ki 'ignore' olsun.
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0,          # ch1 ignore
        0,          # ch2 ignore
        0,          # ch3 ignore
        ch4_val,    # ch4 yaw
        0,          # ch5 ignore
        0,          # ch6 ignore
        0,          # ch7 ignore
        0           # ch8 ignore
    )

def arm_vehicle(master):
    """
    Pixhawk'ı ARM et.
    Mavlink cmd_long: MAV_CMD_COMPONENT_ARM_DISARM (400)
    param1 = 1 -> arm, 0 -> disarm
    param2 = force arm (genelde 0 bırakıyoruz, ama bazı firmware'lerde 21196 kullanılıyor)
    """
    print("[ARM] Arming request gönderiliyor...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,     # param1: 1 = arm, 0 = disarm
        0,     # param2: force arm flag (bazı stack'lerde 21196 gerekebilir)
        0, 0, 0, 0, 0
    )

    # ARM state doğrulama loop
    # heartbeat'ten base_mode okuyarak armed mı değil mi bakacağız
    t0 = time.time()
    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if hb is not None:
            # base_mode bit 0x80 (128) genelde ARMED flag
            # MAV_MODE_FLAG_SAFETY_ARMED = 128
            armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            print(f"[ARM] Heartbeat kontrol: armed={armed}, base_mode={hb.base_mode}")
            if armed:
                print("[ARM] >>> ARAÇ ARM OLDU <<<")
                return True

        if time.time() - t0 > 5.0:
            print("[ARM] Uyarı: 5 sn içinde arm olmadı. Güvenlik kilidi / mode izin vermiyor olabilir.")
            return False

#####################
# ANA AKIŞ
#####################

# 1. Bağlantı
print("[CONNECT] Pixhawk'a bağlanılıyor...")
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# 2. Heartbeat bekle
print("[CONNECT] Heartbeat bekleniyor...")
master.wait_heartbeat()
print(f"[CONNECT] Heartbeat alındı. system={master.target_system}, component={master.target_component}")

# 3. ARM ET
armed_ok = arm_vehicle(master)
if not armed_ok:
    print("[CONNECT] Araç arm olmadı. Devam etmek mantıksız ama yine de loop'a gireceğiz. (Thruster dönmeyebilir)")
else:
    print("[CONNECT] Araç arm durumda, PID kontrol başlayacak.")

# PID objesi hazırla
pid_yaw = PID(Kp, Ki, Kd)

print("[CONTROL] Kontrol döngüsüne giriliyor... (Ctrl+C ile çık)")
try:
    while True:
        # 4. ATTITUDE oku
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg is None:
            print("[CONTROL] ATTITUDE gelmedi.")
            continue

        # yaw'ı dereceye çevir
        current_yaw_deg = math.degrees(msg.yaw)

        # 5. Hata hesapla (target 90°)
        err = angle_error_deg(TARGET_YAW_DEG, current_yaw_deg)

        # 6. PID uygula
        yaw_pid_out = pid_yaw.update(err)

        # PID çıktısını normalize et (-1..1 civarına küçült)
        # Bu gain'i gerektiğinde ayarlayacağız. Küçük başla güvenli olsun.
        yaw_norm = yaw_pid_out / 50.0

        # RC kanalına çevir (1100..1900)
        ch4_val = yaw_cmd_to_rc(yaw_norm)

        # RC override gönder
        send_rc_override(master, ch4_val)

        # Debug print
        print(f"[LOOP] yaw_now={current_yaw_deg:6.2f} deg | err={err:6.2f} deg | pid={yaw_pid_out:7.2f} | norm={yaw_norm:5.2f} | ch4={ch4_val}")

        time.sleep(0.02)  # ~50 Hz

except KeyboardInterrupt:
    print("\n[STOP] Ctrl+C görüldü. Override sıfırlanıyor ve disarm deneniyor...")

    # RC override bırak (0 = ignore)
    send_rc_override(master, 0)

    # Disarm isteği
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # disarm
        0,
        0, 0, 0, 0, 0
    )

    print("[STOP] Bitti.")
