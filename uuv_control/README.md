# UUV Position Stabilization Control System

Modüler robot kontrol sistemi - Pixhawk entegrasyonlu QR kod takip ve stabilizasyon sistemi.

## Klasör Yapısı

```
uuv_control/
├── __init__.py                 # Paket başlatma
├── pixhawk_connection.py       # Pixhawk bağlantı ve temel komutlar
├── pid_controller.py           # PID kontrolcü sınıfı
├── image_processor.py          # Görüntü işleme ve QR kod tespit
├── forward_controller.py       # Forward/Backward kontrol (Channel 5)
├── yaw_controller.py           # Yaw kontrol (Channel 4)
├── lateral_controller.py       # Lateral kontrol (Channel 6)
├── throttle_controller.py      # Throttle kontrol (Channel 3)
├── main.py                     # Ana kontrol scripti
└── README.md                   # Bu dosya
```

## Özellikler

### 1. Pixhawk Bağlantı ve Kontrol
- Pixhawk'a bağlanma (UDP, TCP, Serial)
- ARM/DISARM komutları
- RC kanal override desteği

### 2. Görüntü İşleme
- ArUco marker tespiti
- Marker bilgisi çıkarımı (alan, kenar uzunlukları, merkez)

### 3. Kontrol Modülleri

#### Forward Controller (Channel 5 - X Ekseni)
- **Hedef**: QR kod alanını 20000px'e getirmek
- **Mantık**: 
  - Alan < 20000px → İleri git (yaklaş)
  - Alan > 20000px → Geri git (uzaklaş)
- **PID Katsayıları**: Kp=0.02, Ki=0.0005, Kd=0.01, Deadband=200px

#### Yaw Controller (Channel 4)
- **Hedef**: Sol ve sağ kenar uzunluklarını eşitlemek
- **Mantık**: 
  - Sol kenar > Sağ kenar → Sağa dön
  - Sağ kenar > Sol kenar → Sola dön
- **PID Katsayıları**: Kp=5.0, Ki=0.025, Kd=1.0, Deadband=2px

#### Lateral Controller (Channel 6 - Y Ekseni)
- **Hedef**: Marker'ı ekranın X ekseninde ortalamak
- **Mantık**: 
  - Marker sağda → Sola git
  - Marker solda → Sağa git
- **PID Katsayıları**: Kp=2.0, Ki=0.02, Kd=0.4, Deadband=15px

#### Throttle Controller (Channel 3 - Z Ekseni)
- **Hedef**: Marker'ı ekranın Y ekseninde ortalamak
- **Mantık**: 
  - Marker aşağıda → Yukarı çık
  - Marker yukarıda → Aşağı in
- **PID Katsayıları**: Kp=2.0, Ki=0.02, Kd=0.4, Deadband=15px

## Kullanım

### Temel Kullanım

```python
from uuv_control.main import UUVControlSystem

# Kontrol sistemini başlat
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',  # Pixhawk bağlantısı
    camera_index=0,                            # Kamera index'i
    frame_width=640,                          # Görüntü genişliği
    frame_height=480                           # Görüntü yüksekliği
)

# Çalıştır
control_system.run()
```

### Komut Satırından Çalıştırma

```bash
python -m uuv_control.main
```

### Bağlantı String Örnekleri

```python
# SITL Simülasyonu (UDP)
connection_string = 'udp:127.0.0.1:14551'

# TCP Bağlantı
connection_string = 'tcp:192.168.1.100:5760'

# USB Seri Bağlantı (Linux)
connection_string = '/dev/ttyUSB0'

# USB Seri Bağlantı (Windows)
connection_string = 'COM3'
```

## Kanal Atamaları

| Kanal | İsim | Eksen | Açıklama |
|-------|------|-------|----------|
| Ch3 | Throttle | Z | Yukarı/Aşağı (Batma/Çıkma) |
| Ch4 | Yaw | - | Sol/Sağ Dönüş |
| Ch5 | Forward | X | İleri/Geri |
| Ch6 | Lateral | Y | Sağ/Sol |

## PID Katsayıları

### Forward Controller
- **Kp**: 0.02
- **Ki**: 0.0005
- **Kd**: 0.01
- **Deadband**: 200px

### Yaw Controller
- **Kp**: 5.0
- **Ki**: 0.025
- **Kd**: 1.0
- **Deadband**: 2px

### Lateral Controller
- **Kp**: 2.0
- **Ki**: 0.02
- **Kd**: 0.4
- **Deadband**: 15px

### Throttle Controller
- **Kp**: 2.0
- **Ki**: 0.02
- **Kd**: 0.4
- **Deadband**: 15px

## Gereksinimler

```txt
pymavlink
opencv-python
numpy
```

## Kurulum

```bash
pip install pymavlink opencv-python numpy
```

## Loglama

Sistem otomatik olarak log dosyası oluşturur:
- Dosya adı: `uuv_control_YYYYMMDD_HHMMSS.log`
- Konum: Çalıştırma dizini
- Format: Timestamp, log seviyesi, mesaj

## Güvenlik

- Sistem kapatılırken otomatik olarak:
  - RC override sıfırlanır
  - Pixhawk disarm edilir
  - Bağlantı kapatılır

## Notlar

- PWM değerleri: 1100-1900 aralığında (1500 = nötr)
- Marker tespit edilemediğinde tüm kanallar nötr pozisyona (1500) döner
- Deadband içinde PID çıkışı sıfırlanır ve integral sıfırlanır

