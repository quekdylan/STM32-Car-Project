import cv2
import torch
from ultralytics import YOLO
from pathlib import Path
import time
from PIL import Image
import re

device = 'cuda' if torch.cuda.is_available() else 'cpu'
MODEL_PATH = Path('best_JH.pt')
model = YOLO(MODEL_PATH)
model.to(device)

output_dir = Path("results")
output_dir.mkdir(exist_ok=True)

cap = cv2.VideoCapture(0)

id_map = {
    "10": "Bullseye", "11": "1", "12": "2", "13": "3", "14": "4",
    "15": "5", "16": "6", "17": "7", "18": "8", "19": "9",
    "20": "A", "21": "B", "22": "C", "23": "D", "24": "E",
    "25": "F", "26": "G", "27": "H", "28": "S", "29": "T",
    "30": "U", "31": "V", "32": "W", "33": "X", "34": "Y",
    "35": "Z", "36": "Up", "37": "Down", "38": "Right", "39": "Left",
    "40": "Target"
}

image_counter = 1
detection_history = []
saved_labels = set()
bullseye_last_time = 0
last_detect_time = time.time()

frames_required = 3
cooldown_bullseye = 10
combine_timeout = 20
zoom_factor = 2

def zoom_center(image, factor=2):
    h, w = image.shape[:2]
    new_w, new_h = int(w / factor), int(h / factor)
    x1 = w//2 - new_w//2
    y1 = h//2 - new_h//2
    x2 = x1 + new_w
    y2 = y1 + new_h
    cropped = image[y1:y2, x1:x2]
    return cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

def extract_number(fname):
    match = re.search(r'image(\d+)', fname)
    return int(match.group(1)) if match else 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_zoomed = zoom_center(frame, zoom_factor)
    results = model(frame_zoomed, conf=0.2, verbose=False)
    detected_flag = False
    label = None

    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        label = str(cls_id + 10)
        conf = float(box.conf[0])
        if conf > 0.2:
            detected_flag = True
            break

    if detected_flag and label:
        detection_history.append(label)
        if len(detection_history) > frames_required:
            detection_history.pop(0)

        if len(detection_history) == frames_required and all(l == label for l in detection_history):
            save_ok = False
            if label == "10":
                if time.time() - bullseye_last_time >= cooldown_bullseye:
                    save_ok = True
                    bullseye_last_time = time.time()
            else:
                if label not in saved_labels:
                    save_ok = True
                    saved_labels.add(label)

            if save_ok:
                res_plotted = results[0].plot()
                save_path = output_dir / f"image{image_counter}_{id_map.get(label, label)}.jpg"
                cv2.imwrite(str(save_path), res_plotted)
                print(f"image_number: {image_counter}")
                print(f"image_name: {id_map[label]}")
                print(f"image_id= {label}")
                image_counter += 1
                last_detect_time = time.time()
    else:
        detection_history.clear()

    if time.time() - last_detect_time >= combine_timeout and saved_labels:
        imgs = []
        files = [
            f for f in output_dir.iterdir()
            if f.is_file() and f.suffix.lower() in [".jpg", ".png"] and "combine_result" not in f.name
        ]
        files_sorted = sorted(files, key=lambda x: extract_number(x.stem))
        for file in files_sorted:
            try:
                imgs.append(Image.open(file))
            except:
                continue
        if imgs:
            widths, heights = zip(*(i.size for i in imgs))
            total_width = sum(widths)
            max_height = max(heights)
            new_img = Image.new('RGB', (total_width, max_height))
            x_offset = 0
            for im in imgs:
                new_img.paste(im, (x_offset, 0))
                x_offset += im.width
            combine_path = output_dir / "combine_result.jpg"
            new_img.save(combine_path)
            print(f"💾 Combined saved -> {combine_path}")
        last_detect_time = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
