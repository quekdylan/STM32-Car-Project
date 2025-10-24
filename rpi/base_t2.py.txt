import json
import logging
import time
from multiprocessing import Process
from typing import Optional
import requests

from .base_rpi import RaspberryPi
from .communication.camera import snap_using_picamera2
from .constant.settings import API_TIMEOUT, URL

logger = logging.getLogger(__name__)


class TaskTwo(RaspberryPi):
    """
    TaskTwo (3-F retry mode)
    - 1st F: snap stage=1 -> send L/R immediately
    - 2nd F: snap stage=2 (early try)
        -> if recognized, stitch & finish
        -> else wait for next F (3rd)
    - 3rd F: snap stage=2 again (late try)
        -> if recognized, send result
        -> else send default 'L' (fake box will be handled on API)
    """

    def __init__(self) -> None:
        super().__init__()
        self.first_done = False
        self.completed = False
        self.awaiting_retry = False  # only for the second obstacle

    def start(self) -> None:
        """Start RPi orchestration."""
        logger.info("TaskTwo (3-F mode) starting...")

        try:
            self.android_link.connect()
            self.stm_link.connect()
            self.check_api()

            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_recv_stm32.start()

            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_android.start()

            logger.info("Listening for STM signal: 'F' (snap trigger)")
            self.proc_recv_stm32.join()

        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            logger.error(f"Error in TaskTwo: {e}")
            self.stop()

    def recv_stm(self) -> None:
        """Waits for STM messages and triggers snap when 'F' is received."""
        while not self.completed:
            message: Optional[str] = self.stm_link.wait_receive()
            logger.debug(f"wait recv stm: {message}")

            if not message:
                continue

            msg = message.strip().upper()
            logger.info(f"STM message received: {msg}")

            try:
                if msg == "F":
                    # --- First obstacle ---
                    if not self.first_done:
                        logger.info("[STM] 1st 'F' -> snapping first image")
                        self.snap_and_recognize(stage=1)
                        self.first_done = True

                    # --- Second obstacle, early try ---
                    elif not self.awaiting_retry:
                        logger.info("[STM] 2nd 'F' -> snapping second image (early try)")
                        success = self.snap_and_recognize(stage=2, return_success=True)
                        if success:
                            logger.info("[STM] Second obstacle recognized (early).")
                            self.request_stitch()
                            logger.info("TaskTwo completed successfully.")
                            self.completed = True
                            self.stop()
                            break
                        else:
                            logger.info("[STM] Not recognized (early) -> wait for 3rd 'F'")
                            self.awaiting_retry = True

                    # --- Second obstacle, late try (3rd F) ---
                    else:
                        logger.info("[STM] 3rd 'F' -> retry snapping second image (late try)")
                        success = self.snap_and_recognize(stage=2, return_success=True)
                        if not success:
                            logger.info("[STM] Still not recognized -> default 'L'")
                            self.stm_link.send_cmd("L")
                            logger.info("STM32 <- L (default)")

                        self.request_stitch()
                        logger.info("TaskTwo completed successfully (after retry).")
                        self.completed = True
                        self.stop()
                        break

            except Exception as e:
                logger.error(f"Error in recv_stm: {e}")

    def recv_android(self) -> None:
        """Receive start signal from Android and forward 'S' to STM."""
        # 任务完成后退出该循环，防止刷 b'' 日志
        while not self.completed:
            try:
                android_str = self.android_link.recv()
            except OSError:
                # 连接掉线，不要刷日志；稍作等待再看 completed 标志
                self.android_dropped.set()
                time.sleep(0.2)
                continue
            except Exception as e:
                # 其他接收异常，安静等待以避免刷屏
                logger.debug(f"Android recv error: {e}")
                time.sleep(0.2)
                continue

            # 防止空消息刷屏（包括 b''/None/空字符串）
            if not android_str:
                time.sleep(0.2)
                continue

            try:
                message: dict = json.loads(android_str)
            except Exception:
                logger.debug(f"android raw: {android_str}")
                continue

            if message.get("cat") == "control":
                self.stm_link.send_cmd("S")
                logger.info("STM32 <- S")

        logger.info("recv_android stopped (task completed).")

    def snap_and_recognize(self, stage: int, return_success: bool = False) -> bool:
        """Take picture, send to API, interpret result, send L/R to STM."""
        ts = int(time.time())
        filename = f"/home/mdpgroup19/MDP/SC2079-MDP-Group-19/image_rec/results/{ts}_{stage}.jpg"
        filename_send = f"{ts}_{stage}.jpg"

        logger.info(f"Snapping image (stage {stage}) -> {filename_send}")
        results = None

        try:
            url = f"{URL}/image"
            results = snap_using_picamera2(filename=filename, filename_send=filename_send, url=url)
            logger.info(f"[Camera] Server response: {results}")
        except Exception as e:
            logger.error(f"[Camera] Error contacting API: {e}")

        try:
            image_id = None
            if isinstance(results, dict):
                image_id = results.get("image_id")
                if image_id is not None:
                    image_id = str(image_id).strip()

            logger.info(f"[SNAP {stage}] Recognition result = {image_id}")

            if image_id in ("38", "R"):
                direction = "R"
            elif image_id in ("39", "L"):
                direction = "L"
            else:
                direction = None

            if return_success:
                if direction is None:
                    return False
                self.stm_link.send_cmd(direction)
                logger.info(f"STM32 <- {direction}")
                return True

            if direction is None:
                direction = "L"
                logger.info("[SNAP 1] Default L due to no detection")

            self.stm_link.send_cmd(direction)
            logger.info(f"STM32 <- {direction}")
            return True

        except Exception as e:
            logger.error(f"Error while interpreting recognition result: {e}")
            if not return_success:
                self.stm_link.send_cmd("L")
                logger.warning("Fallback: STM32 <- L (safe mode)")
            return False

    def request_stitch(self) -> None:
        """Requests the API to stitch images."""
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = requests.get(url=f"{URL}/stitch", timeout=API_TIMEOUT)
                if response.status_code == 200:
                    logger.info("Images stitched successfully.")
                    return
                else:
                    logger.error(f"Stitch request failed (code {response.status_code})")
            except Exception as e:
                logger.error(f"Error requesting stitch (attempt {attempt+1}): {e}")
            time.sleep(1)
        logger.error("Failed to stitch after 3 retries.")
