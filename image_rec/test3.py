import cv2
import torch
from ultralytics import YOLO
from pathlib import Path
import time
import os

# Load model and set device
device = 'cuda' if torch.cuda.is_available() else 'cpu'
MODEL_PATH = Path('best_JH.pt')
model = YOLO(MODEL_PATH)
model.to(device)
print(f'✅ model {MODEL_PATH} loaded {device}')

# Create results directory
output_dir = Path("results")
output_dir.mkdir(exist_ok=True)

# Open camera (0 means default camera)
cap = cv2.VideoCapture(0)

# Detection history and counters
detection_history = []
last_saved_label = None
image_counter = 1

# Record first seen time for each label
first_seen_time = {}
current_conf_threshold = 0.3  # default threshold

# ID map for labels
id_map = {
    "10": "Bullseye", "11": "1", "12": "2", "13": "3", "14": "4",
    "15": "5", "16": "6", "17": "7", "18": "8", "19": "9",
    "20": "a", "21": "b", "22": "c", "23": "d", "24": "e",
    "25": "f", "26": "g", "27": "h", "28": "s", "29": "t",
    "30": "u", "31": "v", "32": "w", "33": "x", "34": "y",
    "35": "z", "36": "Up", "37": "Down", "38": "Right", "39": "Left",
    "40": "Target"
}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run YOLO inference with current threshold
    results = model(frame, conf=current_conf_threshold, verbose=False)

    detected_flag = False
    label = None
    conf = 0.0

    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        label = str(cls_id + 10)  # use YOLO id+10 as string
        conf = float(box.conf[0])

        # First seen: record time regardless of confidence
        if label not in first_seen_time:
            first_seen_time[label] = time.time()

        # Confirmed detection: only if confidence > threshold
        if conf > current_conf_threshold:
            detected_flag = True
            break

    # --- dynamic confidence relaxation ---
    if label in first_seen_time:
        elapsed = time.time() - first_seen_time[label]
        if elapsed > 3 and current_conf_threshold > 0.1:
            print(f"⚠️ Relaxing threshold for {id_map.get(label, label)} to 0.1 after 3s no save")
            current_conf_threshold = 0.1

    if detected_flag and label:
        detection_history.append(label)
        if len(detection_history) > 3:
            detection_history.pop(0)

        # Check if last 3 detections are the same
        if len(detection_history) == 3 and all(l == label for l in detection_history):
            if label != last_saved_label:
                res_plotted = results[0].plot()

                # Compute latency from first seen
                detect_time = time.time()
                latency = detect_time - first_seen_time.get(label, detect_time)

                print(f"✅ Detected: {id_map.get(label, label)} "
                      f"(delay {latency:.2f} s since first seen)")

                save_path = output_dir / f"image{image_counter}_{id_map.get(label, label)}.jpg"
                cv2.imwrite(str(save_path), res_plotted)
                print(f"💾 Saved -> {save_path}")

                image_counter += 1
                last_saved_label = label

                # Reset threshold and first_seen for this label
                current_conf_threshold = 0.3
                if label in first_seen_time:
                    del first_seen_time[label]
    else:
        detection_history.clear()

    # Quit key (optional)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
