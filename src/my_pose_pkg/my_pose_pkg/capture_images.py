#!/usr/bin/env python3
"""
capture_rgb_images.py - RGB拍照工具
功能：订阅RGB图像，按键拍照并保存
按键说明：
  SPACE / 回车 - 拍照（保存RGB）
  P            - 打印当前拍照数量
  Q            - 退出程序
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import threading
import cv2

class RGBImageCapture(Node):
    """RGB Image capture node"""
    def __init__(self):
        super().__init__('rgb_image_capture')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.count = 1

        # 创建 image 文件夹
        self.save_dir = os.path.join(os.getcwd(), 'image')
        os.makedirs(self.save_dir, exist_ok=True)

        # 订阅 RGB 图像
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            10
        )

        self.get_logger().info("Ready. SPACE/ENTER=拍照, P=打印数量, Q=退出")

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def take_picture(self):
        if self.rgb_image is not None:
            rgb_path = os.path.join(self.save_dir, f"{self.count}.png")
            cv2.imwrite(rgb_path, self.rgb_image)
            self.get_logger().info(f"Saved: {rgb_path}")
            self.count += 1
        else:
            self.get_logger().warn("RGB image not received yet.")

    def print_count(self):
        print(f"Total images captured: {self.count - 1}")

    def exit_program(self):
        print("Exiting program...")
        rclpy.shutdown()


def keyboard_listener(capture_node):
    """Keyboard listener thread"""
    print("Keyboard listener started. SPACE/ENTER=拍照, P=打印数量, Q=退出")
    while rclpy.ok():
        try:
            key = input().strip().upper()
            if key == '' or key == ' ':
                capture_node.take_picture()
            elif key == 'P':
                capture_node.print_count()
            elif key == 'Q':
                capture_node.exit_program()
                break
            else:
                print(f"Unknown command: '{key}'")
        except EOFError:
            break
        except KeyboardInterrupt:
            capture_node.exit_program()
            break


def main(args=None):
    rclpy.init(args=args)
    capture_node = RGBImageCapture()

    # 启动 ROS spin 线程
    spin_thread = threading.Thread(target=rclpy.spin, args=(capture_node,), daemon=True)
    spin_thread.start()

    try:
        keyboard_listener(capture_node)
    finally:
        capture_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

