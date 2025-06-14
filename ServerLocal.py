import cv2
import time
from ultralytics import YOLO
import paho.mqtt.client as mqtt

# Cấu hình ngưỡng
CONFIDENCE_THRESHOLD = 0.5
ALERT_THRESHOLD = 5
ALERT_TIME_LIMIT = 4

# Biến theo dõi
fire_count = 0
start_time = None
last_fire_time = None
previous_alert_state = False

# MQTT cấu hình
mqtt_broker = "localhost"
mqtt_port = 1883
mqtt_topic = "esp32/fire_detection"

def on_connect(client, userdata, flags, rc):
    print(f"Đã kết nối MQTT Broker với mã: {rc}")

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect

try:
    mqtt_client.connect(mqtt_broker, mqtt_port, 60)
    mqtt_client.loop_start()
    print(f"Đã kết nối đến MQTT broker tại {mqtt_broker}:{mqtt_port}")
except:
    print("Không thể kết nối MQTT broker!")

def main():
    global fire_count, start_time, last_fire_time, previous_alert_state

    url = "http://192.168.117.114/stream"
    cap = cv2.VideoCapture(url)

    if not cap.isOpened():
        print("Không thể kết nối stream từ ESP32-CAM.")
        return

    model = YOLO("best_320.pt")
    counter = 0
    frame_counter = 0
    fps = 0
    fps_timer = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Không nhận được frame.")
            continue

        results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]
        annotated_frame = frame.copy()
        fire_detected = False

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                conf = box.conf[0].item()
                label = "fire" if class_id == 1 else "smoke"

                if conf >= CONFIDENCE_THRESHOLD and (label == "fire" or label == "smoke"):
                    fire_detected = True
                    last_fire_time = time.time()
                    if fire_count == 0:
                        start_time = time.time()
                    fire_count += 1


                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(annotated_frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Reset nếu không thấy cháy trong ALERT_TIME_LIMIT
        if not fire_detected and start_time is not None:
            elapsed = time.time() - start_time
            if elapsed > ALERT_TIME_LIMIT:
                fire_count = 0
                start_time = None

        current_alert_state = fire_count >= ALERT_THRESHOLD
        now = time.time()

        # Gửi MQTT "1" nếu phát hiện cháy
        if current_alert_state:
            last_fire_time = now
            cv2.putText(annotated_frame, "FIRE ALERT!", (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

            if current_alert_state != previous_alert_state:
                try:
                    mqtt_client.publish(mqtt_topic, "1")
                    print("🔥 CẢNH BÁO CHÁY! Đã gửi MQTT '1'")
                except Exception as e:
                    print(f"Lỗi gửi MQTT: {e}")
            previous_alert_state = True

        # Gửi MQTT "0" nếu không cháy trong 10 giây kể từ lần cuối
        elif previous_alert_state and last_fire_time is not None and (now - last_fire_time > 10):
            try:
                mqtt_client.publish(mqtt_topic, "0")
                print("✅ HẾT CHÁY 10s! Đã gửi MQTT '0'")
                previous_alert_state = False
            except Exception as e:
                print(f"Lỗi gửi MQTT: {e}")

        # Hiển thị Fire Count và FPS
        cv2.putText(annotated_frame, f"Fire Count: {fire_count}", (150, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)


        frame_counter += 1
        elapsed = time.time() - fps_timer
        if elapsed >= 1.0:
            fps = frame_counter / elapsed
            frame_counter = 0
            fps_timer = time.time()

        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("ESP32-CAM + Fire Detection", annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            filename = f"capture_{counter}.jpg"
            cv2.imwrite(filename, annotated_frame)
            print(f"Đã lưu ảnh: {filename}")
            counter += 1

    mqtt_client.loop_stop()
    mqtt_client.disconnect()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
