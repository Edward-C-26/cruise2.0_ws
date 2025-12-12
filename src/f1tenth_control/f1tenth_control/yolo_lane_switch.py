#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ultralytics import YOLO
import cv2
import os

class YOLOLaneSwitch(Node):
    def __init__(self):
        super().__init__('yolo_lane_switch')

        # 1. Load Model
        model_path = os.path.join(os.path.dirname(__file__), "best.pt")
        self.model = YOLO(model_path, task='detect')

        # 2. OPEN CAMERA (Fast Mode)
        self.video_port = 4 
        self.cap = cv2.VideoCapture(self.video_port)
        
        # --- LATENCY FIX 1: Tiny Resolution ---
        # 320x240 is plenty for an apple/car and runs much faster
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # --- LATENCY FIX 2: Buffer Size ---
        # This tells the OS "Don't save old frames, give me the newest one ONLY"
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            self.get_logger().error(f"COULD NOT OPEN /dev/video{self.video_port}!")

        # 3. Publisher
        self.pub_lane = self.create_publisher(Bool, '/object_detected', 10)

        # 4. Timer (Run faster - 30Hz)
        self.timer = self.create_timer(0.033, self.loop_callback)
        
        self.valid_counter = 0
        self.threshold = 1  # Instant trigger

    def loop_callback(self):
        if self.cap is None or not self.cap.isOpened():
            return

        # --- LATENCY FIX 3: Flush the Buffer ---
        # Sometimes setting BUFFERSIZE isn't enough. 
        # We grab a frame, but if we detect lag, we can grab another to ensure freshness.
        ret, frame = self.cap.read()
        
        if not ret:
            return

        # Run YOLO (Verbose=False makes it slightly faster)
        results = self.model(frame, verbose=False, imgsz=320)
        valid_detection = False

        # Fast Check
        if results and results[0].boxes:
            # Check the top detection only (fastest)
            box = results[0].boxes[0]
            cls_id = int(box.cls.cpu().numpy())
            conf = float(box.conf.cpu().numpy())
            label = self.model.names[cls_id]
            
            # Lower confidence slightly to trigger faster on distant apples
            if label in ['car', 'apple'] and conf > 0.4:
                valid_detection = True

        # Publish Instantly
        msg_out = Bool()
        if valid_detection:
            msg_out.data = True
            self.pub_lane.publish(msg_out)
            # Only log occasionally to save CPU
            # self.get_logger().info("OBJECT SEEN!") 
        else:
            msg_out.data = False
            self.pub_lane.publish(msg_out)

    def __del__(self):
        if self.cap: self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOLaneSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()