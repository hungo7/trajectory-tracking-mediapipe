import cv2
import time
import socket  # 1. Thêm thư viện kết nối UDP
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# --- CẤU HÌNH UDP SOCKET ĐỂ GỬI SANG QT ---
UDP_IP = "127.0.0.1"  # Chạy trên cùng máy tính
UDP_PORT = 1235       # Khớp với cổng bind(1235) trong Qt của bạn
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Biến toàn cục lưu kết quả nhận diện từ luồng phụ (Callback)
latest_result = None

def print_result(result: vision.HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result

# Khởi tạo cài đặt bộ nhận diện tay theo chuẩn Tasks API mới
base_options = python.BaseOptions(model_asset_path='hand_landmarker.task')
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.LIVE_STREAM,
    num_hands=1,
    min_hand_detection_confidence=0.7,
    min_hand_presence_confidence=0.7,
    min_tracking_confidence=0.7,
    result_callback=print_result
)

detector = vision.HandLandmarker.create_from_options(options)
cap = cv2.VideoCapture(0)

print(f"Đang gửi dữ liệu UDP đến {UDP_IP}:{UDP_PORT}... Nhấn 'q' để thoát.")

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
    
    frame_timestamp_ms = int(time.time() * 1000)
    detector.detect_async(mp_image, frame_timestamp_ms)

    command = "NONE"  # Mặc định ban đầu là STOP

    if latest_result is not None and latest_result.hand_landmarks:
        hand_landmarks = latest_result.hand_landmarks[0]
        
        # Vẽ các điểm khớp tay lên màn hình
        h, w, _ = frame.shape
        for lm in hand_landmarks:
            cx, cy = int(lm.x * w), int(lm.y * h)
            cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

        # Logic thuật toán xử lý cử chỉ
        index_up = hand_landmarks[8].y < hand_landmarks[6].y
        middle_up = hand_landmarks[12].y < hand_landmarks[10].y
        ring_down = hand_landmarks[16].y > hand_landmarks[14].y
        pinky_down = hand_landmarks[20].y > hand_landmarks[18].y

        if index_up and not middle_up and ring_down and pinky_down:
            command = "TURN_LEFT"
        elif index_up and middle_up and ring_down and pinky_down:
            command = "TURN_RIGHT"
        else:
            command = "NONE"

    # --- CHỖ THAY ĐỔI MỚI: LỌC GÓI TIN TRƯỚC KHI GỬI ---
    # Chỉ gửi lệnh khi bạn giơ tay ra lệnh Tiến hoặc Lùi thực sự.
    # Nếu là STOP (không giơ tay / rụt tay), Python sẽ im lặng hoàn toàn để nhường quyền cho Pure Pursuit.
    if command in ["TURN_LEFT", "TURN_RIGHT"]:
        packet = f"{command}\n"
        sock.sendto(bytes(packet, "utf-8"), (UDP_IP, UDP_PORT))

    # Vẫn in chữ lên camera local của máy tính để bạn theo dõi bình thường
    cv2.putText(frame, command, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (0, 255, 0), 3, cv2.LINE_AA)

    cv2.imshow("Hand Gesture Control (Tasks API)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

detector.close()
cap.release()
cv2.destroyAllWindows()