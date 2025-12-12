
#!/usr/bin/env python3
"""
VILA Lane Switch Detection Node
替代 YOLO，使用 VILA 进行瓶子和人的检测
发布 /object_detected 话题触发 pure_pursuit 变道
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
import subprocess
import tempfile
import os
import time

class VILALaneSwitch(Node):
    def __init__(self):
        super().__init__('vila_lane_switch')

        # 摄像头设置 - 使用 video2 (RGB 相机)
        self.get_logger().info("Opening camera /dev/video2...")
        self.cap = cv2.VideoCapture(2)
        
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open /dev/video2!")
            self.cap = None
        else:
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info("✅ Camera opened successfully")

        # 发布到 /object_detected 话题 (pure_pursuit 会监听)
        self.pub_object = self.create_publisher(Bool, '/object_detected', 10)

        # VILA 提示词 - 简单直接
        self.prompt = """Look at this road view. Reply ONE word only:
BOTTLE - if you see a bottle, can, or small object on the road
PERSON - if you see any person or people
CLEAR - if the road is clear
Answer:"""

        # 连续检测计数器
        self.bottle_counter = 0
        self.person_counter = 0
        self.detection_threshold = 2  # 连续2次检测才触发
        
        # 冷却时间（避免重复触发）
        self.last_trigger_time = 0
        self.cooldown_seconds = 3.0

        # 定时器 - 每2秒检测一次
        self.timer = self.create_timer(2.0, self.detection_loop)
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("VILA Lane Switch Detector Ready")
        self.get_logger().info("  Detecting: BOTTLE (switch lane) | PERSON (stop)")
        self.get_logger().info("  Threshold: 2 consecutive detections")
        self.get_logger().info("  Cooldown: 3 seconds")
        self.get_logger().info("=" * 50)

    def ask_vila(self, image_path):
        """询问 VILA 看到了什么"""
        cmd = f"""
        docker run --rm --runtime nvidia \
            -v {image_path}:/tmp/image.jpg \
            dustynv/nano_llm:r36.4.0 \
            python3 -m nano_llm.chat --api=mlc \
            --model Efficient-Large-Model/VILA-2.7b \
            --max-context-len 64 \
            --max-new-tokens 10 \
            --prompt '/tmp/image.jpg' \
            --prompt '{self.prompt}'
        """
        
        try:
            result = subprocess.check_output(
                cmd, 
                shell=True, 
                text=True, 
                timeout=10,
                stderr=subprocess.DEVNULL
            )
            
            response = result.strip().upper()
            
            # 解析响应
            if 'BOTTLE' in response or 'CAN' in response or 'OBJECT' in response:
                return 'BOTTLE'
            elif 'PERSON' in response or 'PEOPLE' in response:
                return 'PERSON'
            else:
                return 'CLEAR'
                
        except subprocess.TimeoutExpired:
            self.get_logger().warn('VILA timeout')
            return 'CLEAR'
        except Exception as e:
            self.get_logger().error(f'VILA error: {e}')
            return 'CLEAR'

    def detection_loop(self):
        """主检测循环"""
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().debug('Camera not available')
            return

        # 1. 捕获图像
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # 保存临时图像
        temp_path = tempfile.mktemp(suffix='.jpg')
        cv2.imwrite(temp_path, frame)

        # 2. 检测
        self.get_logger().info('🔍 Detecting...', throttle_duration_sec=1.0)
        detection = self.ask_vila(temp_path)
        
        # 清理临时文件
        os.remove(temp_path)

        # 3. 更新计数器
        if detection == 'BOTTLE':
            self.bottle_counter += 1
            self.person_counter = 0  # 重置人计数
            self.get_logger().info(f'🍾 Bottle detected ({self.bottle_counter}/{self.detection_threshold})')
            
        elif detection == 'PERSON':
            self.person_counter += 1
            self.bottle_counter = 0  # 重置瓶子计数
            self.get_logger().warn(f'🧍 Person detected ({self.person_counter}/{self.detection_threshold})')
            
        else:  # CLEAR
            self.bottle_counter = 0
            self.person_counter = 0
            self.get_logger().info('✅ Path clear')

        # 4. 检查是否触发
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        time_since_last = current_time - self.last_trigger_time

        # 冷却检查
        if time_since_last < self.cooldown_seconds:
            return

        # 5. 发布检测结果
        msg = Bool()
        
        # 瓶子：触发变道
        if self.bottle_counter >= self.detection_threshold:
            msg.data = True
            self.pub_object.publish(msg)
            self.get_logger().warn('🔄 LANE SWITCH TRIGGERED (Bottle)')
            self.last_trigger_time = current_time
            self.bottle_counter = 0
            
        # 人：也触发（pure_pursuit 会处理停车逻辑）
        elif self.person_counter >= self.detection_threshold:
            msg.data = True
            self.pub_object.publish(msg)
            self.get_logger().error('🛑 PERSON DETECTED - SWITCH LANE TO AVOID')
            self.last_trigger_time = current_time
            self.person_counter = 0
            
        else:
            # 正常发布 False
            msg.data = False
            self.pub_object.publish(msg)

    def destroy_node(self):
        """清理资源"""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VILALaneSwitch()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down VILA detector')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
