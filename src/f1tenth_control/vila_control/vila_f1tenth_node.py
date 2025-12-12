#!/usr/bin/env python3
"""
VILA Vision-Based Control Node for F1TENTH
整合到 cruise2.0_ws/src/f1tenth_control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import subprocess
import tempfile
import os
import time

class VilaF1TenthController(Node):
    def __init__(self):
        super().__init__('vila_f1tenth_controller')
        
        # 参数
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('check_interval', 1.0)  # 秒
        self.declare_parameter('safe_speed', 1.0)  # m/s
        self.declare_parameter('model', 'Efficient-Large-Model/VILA-2.7b')
        
        # ROS 2 接口
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10
        )
        
        # Ackermann 驱动发布者
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter('drive_topic').value,
            10
        )
        
        # 状态发布者
        self.status_pub = self.create_publisher(String, '/vila/status', 10)
        self.safe_pub = self.create_publisher(Bool, '/vila/safe', 10)
        
        # 状态
        self.latest_image = None
        self.processing = False
        self.is_safe = True
        self.last_decision = "STOP"
        
        # 简单而强大的提示词
        self.prompt = """You are controlling a self-driving race car.
Look at the camera view and reply with ONE word:
- STOP: if you see obstacles, people, walls, or danger
- GO: if the path ahead is clear and safe
Answer:"""
        
        # 定时检查
        interval = self.get_parameter('check_interval').value
        self.timer = self.create_timer(interval, self.check_safety)
        
        self.get_logger().info('VILA F1TENTH 控制器已启动')
        self.get_logger().info(f'检查间隔: {interval} 秒')
        self.get_logger().info(f'安全速度: {self.get_parameter("safe_speed").value} m/s')

    def image_callback(self, msg):
        """接收相机图像"""
        if not self.processing:
            try:
                self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().error(f'图像转换失败: {e}')

    def check_safety(self):
        """定期安全检查"""
        if self.latest_image is None or self.processing:
            return
        
        self.processing = True
        
        try:
            # 保存图像
            temp_path = tempfile.mktemp(suffix='.jpg')
            cv2.imwrite(temp_path, self.latest_image)
            
            # 询问 VILA
            decision = self.ask_vila(temp_path)
            
            # 处理决策
            self.process_decision(decision)
            
            # 清理
            os.remove(temp_path)
            
        except Exception as e:
            self.get_logger().error(f'安全检查失败: {e}')
            self.emergency_stop()
        finally:
            self.processing = False

    def ask_vila(self, image_path):
        """询问 VILA 的决策"""
        model = self.get_parameter('model').value
        
        cmd = f"""
        docker run --rm --runtime nvidia \
            -v {image_path}:/tmp/image.jpg \
            dustynv/nano_llm:r36.4.0 \
            python3 -m nano_llm.chat --api=mlc \
            --model {model} \
            --max-context-len 64 \
            --max-new-tokens 8 \
            --prompt '/tmp/image.jpg' \
            --prompt '{self.prompt}'
        """
        
        try:
            result = subprocess.check_output(
                cmd, 
                shell=True, 
                text=True, 
                timeout=5,
                stderr=subprocess.DEVNULL
            )
            
            response = result.strip().upper()
            
            # 解析响应
            if 'STOP' in response:
                return 'STOP'
            elif 'GO' in response:
                return 'GO'
            else:
                self.get_logger().warn(f'不明确的响应: {response}')
                return 'STOP'  # 默认保守
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('VILA 响应超时')
            return 'STOP'
        except Exception as e:
            self.get_logger().error(f'VILA 调用失败: {e}')
            return 'STOP'

    def process_decision(self, decision):
        """处理 VILA 的决策"""
        self.last_decision = decision
        
        if decision == 'STOP':
            self.get_logger().warn('🛑 STOP - 检测到障碍物或危险')
            self.is_safe = False
            self.emergency_stop()
        else:
            self.get_logger().info('✅ GO - 路径清晰')
            self.is_safe = True
            self.safe_forward()
        
        # 发布状态
        status_msg = String()
        status_msg.data = decision
        self.status_pub.publish(status_msg)
        
        safe_msg = Bool()
        safe_msg.data = self.is_safe
        self.safe_pub.publish(safe_msg)

    def emergency_stop(self):
        """紧急停止"""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

    def safe_forward(self):
        """安全前进"""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = self.get_parameter('safe_speed').value
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VilaF1TenthController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('停止 VILA 控制器')
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()