import cv2
import torch
import os
import string
import numpy as np
import glob
from PIL import Image
from ultralytics import YOLO
from pathlib import Path
from datetime import datetime

# confidence threshold for the YOLO model during inference and file path to weights
MODEL_CONFIG = {"conf": 0.3, "path": Path(__file__).parent / "best_JH.pt"}  # model trained on 50k images

device = 'cuda' if torch.cuda.is_available() else 'cpu'

id_map = {
    "10": 10,  # Bullseye
    "11": 11,  # 1
    "12": 12,  # 2
    "13": 13,  # 3
    "14": 14,  # 4
    "15": 15,  # 5
    "16": 16,  # 6
    "17": 17,  # 7
    "18": 18,  # 8
    "19": 19,  # 9
    "20": 20,  # a
    "21": 21,  # b
    "22": 22,  # c
    "23": 23,  # d
    "24": 24,  # e
    "25": 25,  # f
    "26": 26,  # g
    "27": 27,  # h
    "28": 28,  # s
    "29": 29,  # t
    "30": 30,  # u
    "31": 31,  # v
    "32": 32,  # w
    "33": 33,  # x
    "34": 34,  # y
    "35": 35,  # z
    "36": 36,  # Up Arrow
    "37": 37,  # Down Arrow
    "38": 38,  # Right Arrow
    "39": 39,  # Left Arrow
    "40": 40   # target
}


def load_model():
    model = YOLO(MODEL_CONFIG["path"])
    model.to(device)
    return model


def find_largest_or_central_bbox(bboxes, signal):
    """
    Filter and select the best bounding box based on selection mode
    """
    if not bboxes:
        return "NA", 0.0

    # Exclude 'end' class
    valid_bboxes = [bbox for bbox in bboxes if bbox["label"] != "10" and bbox["confidence"] > 0.3]
    if not valid_bboxes:
        return "NA", 0.0

    # If there is only one bounding box, return it
    if len(valid_bboxes) == 1:
        return valid_bboxes[0]["label"], valid_bboxes[0]["bbox_area"]

    # Find the largest bounding box area
    max_area = max(bbox["bbox_area"] for bbox in valid_bboxes)
    # bbox is of similar size if it is within 10% range of largest bbox area
    threshold = 0.1
    largest_bboxes = [bbox for bbox in valid_bboxes if bbox["bbox_area"] >= (1 - threshold) * max_area]

    # Tie-breaking logic using signal
    if signal == 'L':      # Pick the leftmost object
        chosen_bbox = min(largest_bboxes, key=lambda x: x["xywh"][0])
    elif signal == 'R':    # Pick the rightmost object
        chosen_bbox = max(largest_bboxes, key=lambda x: x["xywh"][0])
    else:                  # Default: largest bbox
        chosen_bbox = max(largest_bboxes, key=lambda x: x["bbox_area"])

    return chosen_bbox["label"], chosen_bbox["bbox_area"]


def predict_image(logger, model, image_path, output_dir, signal):
    """
    FOR TASK 1 (保留原样)
    """
    formatted_time = datetime.now().strftime('%d-%m_%H-%M-%S.%f')[:-3]
    img_name = f"processed_{formatted_time}.jpg"

    results = model.predict(
        source=image_path,
        conf=MODEL_CONFIG["conf"],
        imgsz=640,
        device=device,
        verbose=False
    )
    logger.debug(f"predict speed (ms): {results[0].speed}")

    bboxes = []
    if results[0].boxes:  # If there are any detected objects
        for result in results:
            for box in result.boxes:
                cls_index = int(box.cls.tolist()[0])
                label = result.names[cls_index]
                xywh = box.xywh.tolist()[0]
                bbox_area = xywh[2] * xywh[3]
                confidence = box.conf.tolist()[0]
                bboxes.append({"label": label, "xywh": xywh, "bbox_area": bbox_area, "confidence": confidence})
    logger.debug(f"Bounding boxes: '{bboxes}")

    os.makedirs(output_dir, exist_ok=True)
    output_file_path = output_dir / img_name
    results[0].save(output_file_path)
    logger.debug(f"Saved processed image: '{output_file_path}'")

    selected_label, selected_area = find_largest_or_central_bbox(bboxes, signal)
    image_id = id_map.get(selected_label, "NA")

    if selected_label != "NA":
        logger.debug(f"Image '{image_path.name}': Detected '{selected_label}' area={selected_area:.2f} (ID: {image_id})")
    else:
        logger.debug(f"Image '{image_path.name}': No objects detected.")

    return image_id


def _parse_stage_from_name(name: str) -> str:
    """
    从文件名解析 stage（按 RPi 命名：..._1.jpg 或 ..._2.jpg）。
    解析失败时返回 'unknown'。
    """
    base = os.path.basename(name)
    stem = os.path.splitext(base)[0]
    parts = stem.split("_")
    if len(parts) >= 2 and parts[-1].isdigit():
        return parts[-1]  # '1' or '2'
    return "unknown"


def predict_image_t2(logger, model, image_path, output_dir, signal):
    """
    FOR TASK 2:
    - 只识别左右箭头（38/39），bullseye 忽略；
    - 识别成功才保存到 fullsize，并且强制命名为：
        keep_stage1.jpg 或 keep_stage2.jpg（覆盖写）
    - 未识别返回 None（不保存任何 fullsize 图）
    """
    # 先跑一次推理（不立即保存到磁盘）
    results = model.predict(
        source=image_path,
        conf=MODEL_CONFIG["conf"],
        imgsz=640,
        device=device,
        verbose=False
    )
    logger.debug(f"predict speed (ms): {results[0].speed}")

    # 收集左右箭头（置信度>0.3）
    bboxes = []
    if results[0].boxes:
        for result in results:
            for box in result.boxes:
                cls_index = int(box.cls.tolist()[0])
                label = result.names[cls_index]
                xywh = box.xywh.tolist()[0]
                bbox_area = xywh[2] * xywh[3]
                confidence = box.conf.tolist()[0]
                if label in ["38", "39"] and confidence > 0.3:
                    bboxes.append({"label": label, "xywh": xywh, "bbox_area": bbox_area, "confidence": confidence})

    logger.debug(f"Bounding boxes: '{bboxes}")

    # 选框
    selected_label, selected_area = find_largest_or_central_bbox(bboxes, signal)

    # 识别失败：不保存，直接返回 None
    if selected_label not in ["38", "39"]:
        logger.debug(f"Image '{os.path.basename(image_path)}': No valid left/right arrow detected.")
        return

    # 识别成功：根据原始文件名判断 stage，并覆盖保存到固定文件名
    stage = _parse_stage_from_name(os.path.basename(image_path))
    keep_name = "keep_stage1.jpg" if stage == "1" else ("keep_stage2.jpg" if stage == "2" else None)

    if keep_name is None:
        # 无法判断 stage，就不保存（避免影响 stitch）
        logger.debug(f"Image '{os.path.basename(image_path)}': stage not found, skip saving.")
        return selected_label

    # 生成带框图到内存，然后覆盖写入 keep_stageX.jpg
    os.makedirs(output_dir, exist_ok=True)
    annotated = results[0].plot()  # numpy array with YOLO annotations
    keep_path = str(Path(output_dir) / keep_name)
    # OpenCV 用 BGR，annotated 已经是 BGR
    cv2.imwrite(keep_path, annotated)
    logger.debug(f"Saved annotated image to: '{keep_path}' (stage={stage})")

    return selected_label


def stitch_image(logger, output_dir, fullsize_dir):
    """
    只拼接 fullsize 目录下的 keep_stage1.jpg + keep_stage2.jpg 两张图。
    若缺失任意一张，则返回 None。
    最终输出到 output_dir/concatenated.jpg
    """
    try:
        img1_path = Path(fullsize_dir) / "keep_stage1.jpg"
        img2_path = Path(fullsize_dir) / "keep_stage2.jpg"

        if not img1_path.exists() or not img2_path.exists():
            logger.debug("Stitch skipped: keep_stage1.jpg or keep_stage2.jpg missing.")
            return

        # 读取两张图
        img1 = cv2.imread(str(img1_path))
        img2 = cv2.imread(str(img2_path))
        if img1 is None or img2 is None:
            logger.debug("Stitch skipped: failed to read keep_stage images.")
            return

        # 统一高度后左右拼接
        h1, w1 = img1.shape[:2]
        h2, w2 = img2.shape[:2]
        target_h = min(h1, h2)

        def resize_to_h(img, target_h):
            h, w = img.shape[:2]
            new_w = int(w * (target_h / h))
            return cv2.resize(img, (new_w, target_h), interpolation=cv2.INTER_AREA)

        img1r = resize_to_h(img1, target_h)
        img2r = resize_to_h(img2, target_h)
        stitched = np.hstack([img1r, img2r])

        # 输出目录
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        out_path = Path(output_dir) / "concatenated.jpg"
        cv2.imwrite(str(out_path), stitched)
        logger.debug(f"Concatenated image saved to: '{out_path}'")

        return Image.fromarray(cv2.cvtColor(stitched, cv2.COLOR_BGR2RGB))

    except Exception as e:
        logger.debug(f"An error occurred in stitch_image: {e}")
        return


# -------- 下面这两个旧的 resize 辅助函数保留，但在新逻辑中不再使用 --------
def resize_image(logger, image_path, output_dir):
    try:
        img = Image.open(image_path)
        ratio = 0.1952  # scale 3280x2464 to ~640x480
        resized_img = img.resize([int(ratio * s) for s in img.size], Image.LANCZOS)
        output_path = Path(output_dir) / image_path.name.replace("processed", "resized")
        resized_img.save(output_path)
    except Exception as e:
        logger.debug(f"Error resizing image '{image_path}: {e}'")


def resize_all_images(logger, output_dir, fullsize_dir):
    for img_path in Path(fullsize_dir).iterdir():
        if img_path.is_file() and img_path.suffix.lower() in ('.png', '.jpg', '.jpeg'):
            resize_image(logger, img_path, output_dir)
