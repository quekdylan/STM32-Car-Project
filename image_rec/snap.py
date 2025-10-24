import os
import requests

results_dir = "/home/mdpgroup19/MDP/SC2079-MDP-Group-19/image_rec/results"
os.makedirs(results_dir, exist_ok=True)

pc_ip = "192.168.2.6"   # 你的电脑IP
upload_url = f"http://{pc_ip}:5000/upload"

# 模拟拍3张照片，这里用touch代替，你可以换成snap函数
for i in range(1, 4):
    filename = os.path.join(results_dir, f"test{i}.jpg")
    os.system(f"libcamera-still -n -t 1000 -o {filename}")  # 每隔1秒拍1张
    print(f"📷 Captured {filename}")

    # 上传到PC
    with open(filename, "rb") as f:
        r = requests.post(upload_url, files={"file": (f"test{i}.jpg", f)})
        print(r.json())
