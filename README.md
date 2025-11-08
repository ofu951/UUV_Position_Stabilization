# UUV Position Stabilization Control System

Pixhawk entegrasyonlu modüler robot kontrol sistemi. QR kod tespiti ve 4 eksenli stabilizasyon sağlar.

## 📁 Proje Yapısı

```
UUV_Position_Stabilization/
├── uuv_control/              # Ana kontrol modülleri
│   ├── __init__.py
│   ├── pixhawk_connection.py # Pixhawk bağlantı ve komutlar
│   ├── pid_controller.py    # PID kontrolcü sınıfı
│   ├── image_processor.py   # Görüntü işleme ve QR kod tespit
│   ├── forward_controller.py # Forward/Backward kontrol (Ch5)
│   ├── yaw_controller.py    # Yaw kontrol (Ch4)
│   ├── lateral_controller.py # Lateral kontrol (Ch6)
│   ├── throttle_controller.py # Throttle kontrol (Ch3)
│   ├── main.py              # Ana kontrol scripti
│   └── README.md           # Detaylı dokümantasyon
├── run_uuv_control.py      # Kolay çalıştırma scripti
├── yaw90.py                # Orijinal yaw kontrol örneği
├── yaw.py                  # Orijinal yaw görüntü işleme
├── fwd_bwd.py              # Orijinal forward/backward kontrol
├── center.py               # Orijinal merkez kontrol
└── README.md               # Bu dosya
```

## 🚀 Hızlı Başlangıç

### 1. Gereksinimleri Yükleyin

```bash
pip install pymavlink opencv-python numpy
```

### 2. Sistemi Çalıştırın

```bash
python run_uuv_control.py
```

veya

```bash
python -m uuv_control.main
```

## 🎯 Özellikler

### Kontrol Eksenleri

1. **Forward/Backward (Channel 5 - X Ekseni)**
   - QR kod alanı < 20000px → İleri git (yaklaş)
   - QR kod alanı > 20000px → Geri git (uzaklaş)

2. **Yaw (Channel 4)**
   - Sol kenar > Sağ kenar → Sağa dön
   - Sağ kenar > Sol kenar → Sola dön

3. **Lateral (Channel 6 - Y Ekseni)**
   - Marker sağda → Sola git
   - Marker solda → Sağa git

4. **Throttle (Channel 3 - Z Ekseni)**
   - Marker aşağıda → Yukarı çık
   - Marker yukarıda → Aşağı in

### Pixhawk Özellikleri

- ✅ Bağlantı yönetimi (UDP, TCP, Serial)
- ✅ ARM/DISARM komutları
- ✅ RC kanal override
- ✅ Otomatik güvenlik kapatma

## ⚙️ Yapılandırma

### Pixhawk Bağlantısı

`uuv_control/main.py` dosyasında `connection_string` değiştirilebilir:

```python
# SITL Simülasyonu
connection_string = 'udp:127.0.0.1:14551'

# TCP Bağlantı
connection_string = 'tcp:192.168.1.100:5760'

# USB Seri (Linux)
connection_string = '/dev/ttyUSB0'

# USB Seri (Windows)
connection_string = 'COM3'
```

### PID Katsayıları

Her kontrolcü için PID katsayıları ilgili modül dosyasında ayarlanabilir:

- **Forward**: `uuv_control/forward_controller.py`
- **Yaw**: `uuv_control/yaw_controller.py`
- **Lateral**: `uuv_control/lateral_controller.py`
- **Throttle**: `uuv_control/throttle_controller.py`

## 📊 PID Katsayıları (Varsayılan)

| Kontrolcü | Kp | Ki | Kd | Deadband |
|-----------|----|----|----|----------|
| Forward | 0.02 | 0.0005 | 0.01 | 200px |
| Yaw | 5.0 | 0.025 | 1.0 | 2px |
| Lateral | 2.0 | 0.02 | 0.4 | 15px |
| Throttle | 2.0 | 0.02 | 0.4 | 15px |

## 📝 Kullanım Örneği

```python
from uuv_control.main import UUVControlSystem

# Kontrol sistemini başlat
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',
    camera_index=0,
    frame_width=640,
    frame_height=480
)

# Çalıştır
control_system.run()
```

## 🔧 Modüler Yapı

Sistem modüler olarak tasarlanmıştır. Her kontrolcü bağımsız olarak kullanılabilir:

```python
from uuv_control.forward_controller import ForwardController
from uuv_control.yaw_controller import YawController

forward_ctrl = ForwardController(target_area=20000)
yaw_ctrl = YawController()

# Marker bilgisi ile kontrol
marker_info = [...]  # image_processor'dan gelen bilgi
forward_pwm = forward_ctrl.calculate_control(marker_info)
yaw_pwm = yaw_ctrl.calculate_control(marker_info)
```

## 📋 Loglama

Sistem otomatik olarak log dosyası oluşturur:
- **Dosya**: `uuv_control_YYYYMMDD_HHMMSS.log`
- **Konum**: Çalıştırma dizini
- **İçerik**: Tüm kontrol işlemleri ve hatalar

## ⚠️ Güvenlik

- Sistem kapatılırken otomatik olarak:
  - RC override sıfırlanır
  - Pixhawk disarm edilir
  - Bağlantı kapatılır
- Marker tespit edilemediğinde tüm kanallar nötr pozisyona (1500) döner

## 📚 Detaylı Dokümantasyon

Detaylı dokümantasyon için: `uuv_control/README.md`

## 🔄 Orijinal Kodlar

Proje kök dizinindeki dosyalar orijinal test kodlarıdır:
- `yaw90.py`: Pixhawk yaw kontrol örneği
- `yaw.py`: Yaw görüntü işleme
- `fwd_bwd.py`: Forward/backward kontrol
- `center.py`: Merkez kontrol

## 📄 Lisans

Bu proje eğitim ve araştırma amaçlıdır.
