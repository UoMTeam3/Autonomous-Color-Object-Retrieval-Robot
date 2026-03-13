#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# =========================================================
# ✅ 你的参数（已替换进“别人代码”）
# =========================================================

# 1) YOLO 模型路径（换成你的 best.pt）
MODEL_PATH = "/home/laptop5/mycobot_ros2/mycobot_280/mycobot_280pi/mycobot_280pi/best.pt"

# 2) RealSense 分辨率 & 帧率（建议和采集数据一致）
WIDTH, HEIGHT, FPS = 640, 480, 30

# 3) YOLO 推理参数
CONF_THRES = 0.5   # 置信度阈值：越大越严格（漏检↑，误检↓）
IMG_SIZE = 640     # YOLO 输入尺寸：一般=训练时的 imgsz（常见 640）

# （可选）深度采样窗口大小：中心点附近 win x win
DEPTH_WIN = 5


def main():
    # ---------- 1) 加载 YOLOv8 模型 ----------
    print("[INFO] Loading YOLOv8 model...")
    model = YOLO(MODEL_PATH)
    print("[INFO] Model loaded. Classes:", model.names)

    # ---------- 2) 配置 RealSense 流 ----------
    pipeline = rs.pipeline()
    config = rs.config()

    # ✅ 用你的 WIDTH / HEIGHT / FPS
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    print("[INFO] Starting RealSense pipeline...")
    profile = pipeline.start(config)

    # 深度对齐到彩色图像
    align = rs.align(rs.stream.color)

    # 深度尺度：把 depth unit 转成米
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"[INFO] Depth scale: {depth_scale} meters per unit")

    print("[INFO] Press 'q' to quit.")

    intr = None  # 彩色相机内参（只需取一次）

    try:
        while True:
            # ---------- 3) 读取并对齐帧 ----------
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            # 只取一次内参（intrinsics）
            if intr is None:
                intr = color_frame.profile.as_video_stream_profile().intrinsics
                print(f"[INFO] Color intrinsics: fx={intr.fx:.2f}, fy={intr.fy:.2f}, "
                      f"ppx={intr.ppx:.2f}, ppy={intr.ppy:.2f}")

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # ---------- 4) YOLOv8 推理 ----------
            # ✅ 用你的 CONF_THRES 和 IMG_SIZE
            results = model.predict(
                source=color_image,
                conf=CONF_THRES,   # 你的置信度阈值
                imgsz=IMG_SIZE,    # 你的输入尺寸
                verbose=False
            )

            annotated = color_image.copy()

            # ---------- 5) 处理检测结果 ----------
            for r in results:
                boxes = r.boxes
                if boxes is None:
                    continue

                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())

                    # bbox 中心点（深度取样用）
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # ---------- 6) 计算距离（中心附近窗口取样） ----------
                    # ✅ 把窗口大小参数化为 DEPTH_WIN
                    win = DEPTH_WIN
                    half = win // 2

                    x_start = max(cx - half, 0)
                    x_end = min(cx + half, depth_image.shape[1] - 1)
                    y_start = max(cy - half, 0)
                    y_end = min(cy + half, depth_image.shape[0] - 1)

                    depth_patch = depth_image[y_start:y_end + 1, x_start:x_end + 1].astype(float)
                    valid = depth_patch[depth_patch > 0]  # 去掉 0（无效深度）

                    if valid.size > 0:
                        # 原版用 mean；想更抗噪可以改成 np.median(valid)
                        depth_m = valid.mean() * depth_scale
                        dist_text = f"{depth_m:.2f} m"
                    else:
                        depth_m = None
                        dist_text = "no depth"

                    # ---------- 7) 像素 -> 3D 相机坐标 ----------
                    coord_text = "X:-- Y:-- Z:--"
                    if depth_m is not None and intr is not None:
                        X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], depth_m)
                        coord_text = f"X:{X:.4f} Y:{Y:.4f} Z:{Z:.4f}"

                    # ---------- 8) 画框 + 文本 ----------
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated, (cx, cy), 3, (0, 255, 0), -1)

                    class_name = model.names.get(cls_id, str(cls_id))
                    label = f"{class_name} {conf:.2f} {dist_text}"

                    text_x, text_y = x1, max(y1 - 10, 0)
                    cv2.putText(annotated, label, (text_x, text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    cv2.putText(annotated, coord_text, (text_x, min(text_y + 18, annotated.shape[0] - 5)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

                    print(f"Detected {class_name} @ px=({cx},{cy}), dist={dist_text}, 3D(camera)={coord_text}")

            # ---------- 9) 显示 ----------
            cv2.imshow("RealSense + YOLOv8 (color+shape+distance+3D)", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC 退出
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] Stopped RealSense pipeline and closed window.")


# ✅ 修正 main 入口写法（你原来是 _name_ / _main_，那会导致 main 根本不运行）
if __name__ == "__main__":
    main()

