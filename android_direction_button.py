# Android app → Raspberry Pi → STM32import json
# STM32 → Raspberry Pi → Android app
import os
import sys
import json


sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from rpi.communication.android import AndroidLink, AndroidMessage
from rpi.constant.consts import Category


android_link = AndroidLink()
android_link.connect()
android_link.send(AndroidMessage("info", "You are reconnected!"))

while True:
    android_str = android_link.recv()
    if android_str is None:
        continue
    print(android_str)
    message: dict = json.loads(android_str)
    print(f"Message received: {message}")

android_link.disconnect()
