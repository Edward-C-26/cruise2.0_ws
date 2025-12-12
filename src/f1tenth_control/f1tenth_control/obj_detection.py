#!/usr/bin/env python3
import cv2

video_devices = [f"/dev/video{i}" for i in range(6)]  # video0 到 video5

for dev in video_devices:
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        print(f"{dev}: cannot open")
        continue

    ret, frame = cap.read()
    if not ret:
        print(f"{dev}: opened but cannot read frame")
        cap.release()
        continue

    # 检查是否是彩色图像
    if len(frame.shape) == 3 and frame.shape[2] == 3:
        print(f"{dev}: RGB camera detected, resolution {frame.shape[1]}x{frame.shape[0]}")
    else:
        print(f"{dev}: not RGB (grayscale or unknown)")

    cap.release()

