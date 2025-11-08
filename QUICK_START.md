# Hızlı Başlangıç Kılavuzu

## 1. Gereksinimleri Yükleyin

```bash
pip install -r requirements.txt
```

veya manuel olarak:

```bash
pip install pymavlink opencv-python numpy
```

## 2. Kodu Çalıştırın

### Yöntem 1: Kolay Çalıştırma Scripti (Önerilen)

```bash
python run_uuv_control.py
```

### Yöntem 2: Modül Olarak Çalıştırma

```bash
python -m uuv_control.main
```

## 3. Yapılandırma

### Pixhawk Bağlantısı

`uuv_control/main.py` dosyasındaki `connection_string` değerini değiştirin:

```python
# Satır 24 civarında
connection_string = 'udp:127.0.0.1:14551'  # SITL için
# veya
connection_string = 'tcp:192.168.1.100:5760'  # TCP için
# veya
connection_string = 'COM3'  # Windows USB için
# veya
connection_string = '/dev/ttyUSB0'  # Linux USB için
```

### Kamera Ayarları

Kamera index'i ve çözünürlük ayarları `main()` fonksiyonunda değiştirilebilir:

```python
control_system = UUVControlSystem(
    connection_string='udp:127.0.0.1:14551',
    camera_index=0,        # Kamera index'i (0, 1, 2, ...)
    frame_width=640,       # Görüntü genişliği
    frame_height=480      # Görüntü yüksekliği
)
```

## 4. Çalıştırma Öncesi Kontroller

✅ Pixhawk bağlantısı hazır mı?
✅ Kamera çalışıyor mu?
✅ Gerekli Python paketleri yüklü mü?
✅ ArUco marker'lar hazır mı?

## 5. Çalıştırma

1. Pixhawk'ı başlatın (veya SITL simülasyonu)
2. Kamerayı bağlayın
3. Scripti çalıştırın:
   ```bash
   python run_uuv_control.py
   ```

## 6. Çıkış

- 'q' tuşuna basarak çıkabilirsiniz
- Veya Ctrl+C ile durdurabilirsiniz
- Sistem otomatik olarak güvenli şekilde kapanır

## Sorun Giderme

### Import Hatası

Eğer import hatası alırsanız:

```bash
# Modül olarak çalıştırın
python -m uuv_control.main
```

### Kamera Bulunamadı

Kamera index'ini değiştirin veya kameranın bağlı olduğundan emin olun.

### Pixhawk Bağlantı Hatası

- Bağlantı string'ini kontrol edin
- Pixhawk'ın çalıştığından emin olun
- Firewall ayarlarını kontrol edin (UDP/TCP için)

## Notlar

- İlk çalıştırmada log dosyası oluşturulur: `uuv_control_YYYYMMDD_HHMMSS.log`
- Marker tespit edilemediğinde robot nötr pozisyonda kalır
- Tüm kanallar 1500 PWM değerinde nötr pozisyondadır

