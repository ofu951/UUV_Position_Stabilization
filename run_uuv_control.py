"""
UUV Control System - Çalıştırma Scripti
Bu script ana kontrol sistemini çalıştırır
"""

import sys
import os

# Proje kök dizinini path'e ekle
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

# uuv_control modülünü import et
try:
    from uuv_control.main import main
except ImportError as e:
    print(f"Import hatası: {e}")
    print("\nLütfen şu komutu çalıştırın:")
    print("python -m uuv_control.main")
    sys.exit(1)

if __name__ == "__main__":
    main()

