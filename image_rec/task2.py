import cv2
import torch
from ultralytics import YOLO
from pathlib import Path
import time
import os

device = 'cuda' if torch.cuda.is_available() else 'cpu'
MODEL_PATH = Path('best_JH.pt')
model = YOLO(MODEL_PATH)
model.to(device)

output_dir = Path("results")
output_dir.mkdir(exist_ok=True)

cap = cv2.VideoCapture(0)

id_map = {
    "10": "Bullseye",
    "11": "1",
    "12": "2",
    "13": "3",
    "14": "4",
    "15": "5",
    "16": "6",
    "17": "7",
    "18": "8",
    "19": "9",
    "20": "A",
    "21": "B",
    "22": "C",
    "23": "D",
    "24": "E",
    "25": "F",
    "26": "G",
    "27": "H",
    "28": "S",
    "29": "T",
    "30": "U",
    "31": "V",
    "32": "W",
    "33": "X",
    "34": "Y",
    "35": "Z",
    "36": "Up Arrow",
    "37": "Down Arrow",
    "38": "Right Arrow",
    "39": "Left Arrow",
    "40": "Target"
}

image_counter = 1
last_saved_label = None
last_saved_time = 0  # track last save timestamp
detection_history = []

COOLDOWN_TIME = 20  # cooldown in seconds

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, conf=0.3, verbose=False)
    detected_flag = False
    label = None

    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        label = str(cls_id + 10)
        conf = float(box.conf[0])
        if conf > 0.3:
            detected_flag = True
            break

    if detected_flag and label:
        detection_history.append(label)
        if len(detection_history) > 3:
            detection_history.pop(0)

        if len(detection_history) == 3 and all(l == label for l in detection_history):
            current_time = time.time()
            if (label != last_saved_label) or (current_time - last_saved_time > COOLDOWN_TIME):
                res_plotted = results[0].plot()
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_path = output_dir / f"{label}_{timestamp}.jpg"
                cv2.imwrite(str(save_path), res_plotted)
                print(f"image_number: {image_counter}")
                print(f"image_name: {id_map[label]}")
                print(f"image_id= {label}")
                image_counter += 1
                last_saved_label = label
                last_saved_time = current_time
    else:
        detection_history.clear()

    time.sleep(0.5)

cap.release()
cv2.destroyAllWindows()
