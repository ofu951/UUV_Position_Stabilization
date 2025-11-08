import cv2
import numpy as np
import time

class CameraArucoDetector:
    def __init__(self):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.frame_count = 0
        self.fps = 0
        self.start_time = time.time()
        
    def detect_markers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)
        return corners, ids
    
    def calculate_bounding_box_info(self, corners):
        if corners is None or len(corners) == 0:
            return None, None, None
        
        box_info = []
        
        for i, corner in enumerate(corners):
            points = corner[0].astype(np.float32)
            
            # Alan hesapla
            area = cv2.contourArea(points)
            
            # Kenar uzunluklarını hesapla ve isimlendir
            edge_lengths = {}
            edge_names = ['UST', 'SAG', 'ALT', 'SOL']
            
            for j in range(4):
                p1 = points[j]
                p2 = points[(j + 1) % 4]
                distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                edge_lengths[edge_names[j]] = distance
            
            # Ortalama, min, max hesapla
            all_lengths = list(edge_lengths.values())
            avg_edge_length = np.mean(all_lengths)
            min_edge_length = np.min(all_lengths)
            max_edge_length = np.max(all_lengths)
            
            box_info.append({
                'area': area,
                'edge_lengths': edge_lengths,  # Dictionary olarak sakla
                'edge_names': edge_names,
                'avg_edge_length': avg_edge_length,
                'min_edge_length': min_edge_length,
                'max_edge_length': max_edge_length,
                'center': points.mean(axis=0).astype(int),
                'points': points  # Köşe noktalarını da sakla
            })
        
        return box_info
    
    def draw_detection_results(self, frame, corners, ids):
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            box_info = self.calculate_bounding_box_info(corners)
            
            for i, corner in enumerate(corners):
                center = corner[0].mean(axis=0).astype(int)
                
                # ID bilgisi
                cv2.putText(frame, f"ID: {ids[i][0]}", 
                          (center[0] - 30, center[1] - 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if box_info and i < len(box_info):
                    info = box_info[i]
                    
                    # Köşe noktalarını ve isimlerini göster
                    for j, point in enumerate(info['points']):
                        # Köşe numarası
                        cv2.circle(frame, tuple(point.astype(int)), 8, (0, 0, 255), -1)
                        cv2.putText(frame, f"K{j+1}", 
                                  tuple(point.astype(int) + np.array([10, -10])),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                        # Köşe koordinatları
                        cv2.putText(frame, f"({point[0]:.0f},{point[1]:.0f})", 
                                  tuple(point.astype(int) + np.array([10, 15])),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 0), 1)
                    
                    # Kenar uzunluklarını çizgilerin ortasına yaz
                    edge_positions = []
                    for j in range(4):
                        p1 = info['points'][j]
                        p2 = info['points'][(j + 1) % 4]
                        mid_point = ((p1[0] + p2[0])/2, (p1[1] + p2[1])/2)
                        edge_positions.append(mid_point)
                    
                    # Alan bilgisi
                    area_text = f"Alan: {info['area']:.0f} px²"
                    cv2.putText(frame, area_text, 
                              (center[0] - 40, center[1] + 20),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return frame
    
    def draw_detailed_info(self, frame, corners, ids):
        if ids is not None and corners is not None:
            box_info = self.calculate_bounding_box_info(corners)
            
            y_offset = 120
            
            for i, info in enumerate(box_info):
                marker_id = ids[i][0]
                
                # Marker başlığı
                cv2.putText(frame, f"=== Marker ID {marker_id} ===", 
                          (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                y_offset += 25
                
                # Alan bilgisi
                cv2.putText(frame, f"Toplam Alan: {info['area']:.0f} px²", 
                          (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 1)
                y_offset += 20
                
                # Ortalama kenar uzunluğu
                cv2.putText(frame, f"Ortalama Kenar: {info['avg_edge_length']:.1f} px", 
                          (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 1)
                y_offset += 20
                
                # Kenar uzunlukları (açık şekilde)
                cv2.putText(frame, "KENAR UZUNLUKLARI:", 
                          (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)
                y_offset += 20
                
                for edge_name in info['edge_names']:
                    length = info['edge_lengths'][edge_name]
                    cv2.putText(frame, f"  {edge_name}: {length:.1f} px", 
                              (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    y_offset += 18
                
                # Köşe bilgileri
                cv2.putText(frame, "KOSE NOKTALARI:", 
                          (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 100), 1)
                y_offset += 20
                
                for j, point in enumerate(info['points']):
                    cv2.putText(frame, f"  K{j+1}: ({point[0]:.0f}, {point[1]:.0f})", 
                              (15, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)
                    y_offset += 15
                
                y_offset += 10
        
        return frame
    
    def calculate_fps(self):
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time > 1.0:
            self.fps = self.frame_count / elapsed_time
            self.frame_count = 0
            self.start_time = current_time
        
        return self.fps
    
    def draw_status_info(self, frame, corners, ids):
        fps = self.calculate_fps()
        
        cv2.putText(frame, f"FPS: {fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        if ids is not None:
            status_text = f"Markerlar tespit edildi: {len(ids)}"
            color = (0, 255, 0)
            
            id_list = ", ".join([str(id[0]) for id in ids])
            cv2.putText(frame, f"ID'ler: {id_list}", 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            status_text = "Marker tespit edilmedi"
            color = (0, 0, 255)
        
        cv2.putText(frame, status_text, 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        return frame

def initialize_camera():
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Hata: Kamera acilamadi")
        return None
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    print("Kamera basariyla acildi")
    return cap

def main():
    detector = CameraArucoDetector()
    cap = initialize_camera()
    
    if cap is None:
        return
    
    print("Aruco marker tespiti baslatildi")
    print("KOSE NUMARALANDIRMASI:")
    print("K1 -> Sol Ust")
    print("K2 -> Sag Ust") 
    print("K3 -> Sag Alt")
    print("K4 -> Sol Alt")
    print("Cikmak icin 'q' tusuna basin")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            corners, ids = detector.detect_markers(frame)
            result_frame = detector.draw_detection_results(frame.copy(), corners, ids)
            result_frame = detector.draw_detailed_info(result_frame, corners, ids)
            result_frame = detector.draw_status_info(result_frame, corners, ids)
            
            cv2.imshow("Aruco Marker Tespiti - Detayli", result_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
    except Exception as e:
        print(f"Hata: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
