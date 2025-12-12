#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO

class YOLOLaneSwitch(Node):
    def __init__(self):
        super().__init__('yolo_lane_switch')

        # 加载 YOLO 模型
        model_path = os.path.join(os.path.dirname(__file__), "best.pt")
        self.get_logger().info(f"Loading YOLOv8 model: {model_path}")
        self.model = YOLO(model_path, task='detect')
        self.bridge = CvBridge()

        # 打开摄像头 /dev/video4
        self.cap = cv2.VideoCapture("/dev/video4")
        if not self.cap.isOpened():
            self.get_logger().warning("Cannot open /dev/video4, continuing without camera")
            self.cap = None
        else:
            # 设置分辨率为 640x480
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 发布换赛道消息
        self.pub_lane = self.create_publisher(Bool, '/lane_switch', 10)

        # 连续有效帧计数
        self.valid_counter = 0
        self.threshold = 1  # 连续7帧触发换赛道 

        # 定时器循环
        self.timer = self.create_timer(0.1, self.loop_callback)  # 10Hz

        self.get_logger().info("YOLOv8 lane switch detector ready.")

    def loop_callback(self):
        valid_detection = False

        # 如果摄像头可用，读取帧
        if self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                results = self.model(frame)

                # 判断有效检测
                if results and results[0].boxes:
                    dets = results[0].boxes
                    # 检测到只有一个目标且是 car 或 apple
                    if len(dets) == 1:
                        cls_id = int(dets.cls[0].cpu().numpy())
                        label = self.model.names[cls_id]
                        if label in ['car', 'apple']:
                            valid_detection = True
            else:
                self.get_logger().debug("No image input from camera")
        else:
            self.get_logger().debug("Camera not available, skipping detection")

        # 更新连续有效帧计数
        if valid_detection:
            self.valid_counter += 1
        else:
            self.valid_counter = 0  # 重置

        # 发布换赛道消息
        lane_switch_msg = Bool()
        if self.valid_counter >= self.threshold:
            lane_switch_msg.data = True
            self.pub_lane.publish(lane_switch_msg)
            self.get_logger().info("Publishing lane switch!")
            self.valid_counter = 0
        else:
            lane_switch_msg.data = False
            self.pub_lane.publish(lane_switch_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOLaneSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.cap is not None:
        node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

