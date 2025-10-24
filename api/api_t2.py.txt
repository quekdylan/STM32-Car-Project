from werkzeug.datastructures import FileStorage
import time
from flask_restx import Resource, Api, marshal
from flask_cors import CORS
from flask import Flask, request
from pathlib import Path
import os
import sys
import shutil  # for copying pre-saved image
import cv2     # kept in case downstream plotting needs it

# Allow Python to find sibling directories
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from models.models import get_models
from tools.logger import setup_logger
from tools.network import network_monitor
from algo.algorithms.algo import MazeSolver
from algo.tools.commands import CommandGenerator
from image_rec.model import load_model, predict_image_t2, stitch_image

# ------------------------------------------------------
# Flask + API setup
# ------------------------------------------------------
app = Flask(__name__)
api = Api(app, validate=True)
restx_models = get_models(api)
logger = setup_logger()
CORS(app)

# ------------------------------------------------------
# Load YOLO model once at startup
# ------------------------------------------------------
logger.info("Loading YOLO model...")
model = load_model()
logger.info("Model loaded successfully.")

# ------------------------------------------------------
# Helpers
# ------------------------------------------------------
def _parse_stage(filename: str) -> str:
    """
    从上传文件名解析 stage。期望形如:  <timestamp>_<stage>.jpg
    解析失败返回 'unknown'
    """
    base = os.path.basename(filename)
    stem = os.path.splitext(base)[0]
    parts = stem.split("_")
    if len(parts) >= 2 and parts[-1].isdigit():
        return parts[-1]  # '1' or '2'
    return "unknown"


# ------------------------------------------------------
# STATUS CHECK
# ------------------------------------------------------
@api.route('/status')
class Status(Resource):
    @api.response(model=restx_models["Ok"], code=200, description="Success")
    def get(self):
        return marshal({"result": "ok"}, restx_models["Ok"]), 200


# ------------------------------------------------------
# PATHFINDING
# ------------------------------------------------------
@api.route('/path')
class PathFinding(Resource):
    @api.expect(restx_models["PathFindingRequest"])
    @api.response(model=restx_models["PathFindingResponse"], code=200, description="Success")
    @api.response(model=restx_models["Error"], code=500, description="Internal Server Error")
    def post(self):
        try:
            content = request.json
            logger.debug(f"Request from RPi: {content}")
            obstacles = content['obstacles']
            retrying = content.get('retrying', False)
            robot_x, robot_y = content.get('robot_x', 1), content.get('robot_y', 1)
            robot_direction = content.get('robot_dir', 0)

            maze_solver = MazeSolver(
                size_x=20, size_y=20,
                robot_x=robot_x, robot_y=robot_y,
                robot_direction=robot_direction
            )

            for ob in obstacles:
                maze_solver.add_obstacle(ob['x'], ob['y'], ob['d'], ob['id'])

            start = time.time()
            optimal_path, cost = maze_solver.get_optimal_path()
            runtime = time.time() - start
            logger.debug(f"A* runtime: {runtime:.2f}s | cost={cost}")

            motions, obstacle_id_with_signals, scanned_obstacles = maze_solver.optimal_path_to_motion_path(optimal_path)
            command_generator = CommandGenerator()
            commands = command_generator.generate_commands(
                motions, obstacle_id_with_signals, scanned_obstacles, optimal_path)

            path_results = [pos.get_dict() for pos in optimal_path]

            return marshal({
                "data": {"path": path_results, "commands": commands}
            }, restx_models["PathFindingResponse"]), 200

        except Exception as error:
            logger.debug("", exc_info=True)
            return marshal({"error": repr(error)}, restx_models["Error"]), 500


# ------------------------------------------------------
# FILE UPLOAD PARSER
# ------------------------------------------------------
file_upload_parser = api.parser()
file_upload_parser.add_argument('file', location='files', type=FileStorage, required=True)


# ------------------------------------------------------
# IMAGE PREDICTION (Task 2)
# ------------------------------------------------------
@api.route('/image')
class ImagePredict(Resource):
    @api.expect(file_upload_parser)
    @api.response(model=restx_models["ImagePredictResponse"], code=200, description="Success")
    @api.response(model=restx_models["Error"], code=500, description="Internal Server Error")
    def post(self):
        try:
            file = request.files['file']
            filename = file.filename
            logger.info(f"Received file: {filename}")

            # 解析 obstacle_id 与 signal（兼容多种命名）
            parts = filename.replace(".jpg", "").split("_")
            if len(parts) == 3:
                _, obstacle_id, signal = parts
            elif len(parts) == 2:
                obstacle_id, signal = parts[1], "unknown"
            elif len(parts) == 1:
                obstacle_id, signal = "0", "unknown"
            else:
                obstacle_id, signal = parts[-2], parts[-1]
            stage = _parse_stage(filename)

            logger.debug(f"Parsed filename -> obstacle_id={obstacle_id}, signal={signal}, stage={stage}")

            # 保存原图
            upload_dir = Path("image_rec_files/uploads")
            upload_dir.mkdir(parents=True, exist_ok=True)
            file_path = upload_dir / filename
            file.save(file_path)
            logger.debug(f"Saved image to {file_path}")

            # YOLO 推理（内部在识别成功时会写 keep_stage1/keep_stage2）
            output_dir = Path("image_rec_files/output/fullsize")
            output_dir.mkdir(parents=True, exist_ok=True)

            image_id = predict_image_t2(logger, model, file_path, output_dir, signal)

            # ⚠️ Early 失败且是第二个障碍(stage=2)：
            #    不返回方向，让 RPi 进入第三次 snap；
            #    同时用预置图补上 keep_stage2.jpg 以保证最终能拼接。
            if image_id is None and stage == "2":
                logger.info("[Fallback] Recognition failed for stage 2 -> use pre-saved keep_stage2.jpg")
                try:
                    src = Path(__file__).resolve().parent / "keep_stage2.jpg"
                    dst = output_dir / "keep_stage2.jpg"
                    if src.exists():
                        shutil.copy2(src, dst)
                        logger.info(f"[Fallback] Copied pre-saved keep_stage2.jpg -> {dst}")
                    else:
                        logger.warning(f"[Fallback] Missing pre-saved file: {src}")
                except Exception as e:
                    logger.error(f"[Fallback] Error replacing keep_stage2: {e}")

            # 返回到 RPi（保持 image_id=None 时，RPi 会继续第三次 snap / 或 default）
            return marshal({
                "obstacle_id": obstacle_id,
                "image_id": image_id
            }, restx_models["ImagePredictResponse"]), 200

        except Exception as error:
            logger.error(f"[API /image] Error: {repr(error)}", exc_info=True)
            return {"error": repr(error)}, 500


# ------------------------------------------------------
# IMAGE STITCHING
# ------------------------------------------------------
@api.route('/stitch')
class Stitch(Resource):
    @api.response(model=restx_models["Ok"], code=200, description="Success")
    @api.response(model=restx_models["Error"], code=500, description="Internal Server Error")
    def get(self):
        try:
            output_dir = Path("image_rec_files/output")
            fullsize_dir = output_dir / "fullsize"
            output_dir.mkdir(parents=True, exist_ok=True)
            fullsize_dir.mkdir(parents=True, exist_ok=True)

            img = stitch_image(logger, output_dir, fullsize_dir)
            logger.info("Stitch completed.")
            return marshal({"result": "ok"}, restx_models["Ok"]), 200
        except Exception as error:
            logger.error(f"[API /stitch] Error: {repr(error)}", exc_info=True)
            return marshal({"error": repr(error)}, restx_models["Error"]), 500


# ------------------------------------------------------
# RUN SERVER
# ------------------------------------------------------
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
