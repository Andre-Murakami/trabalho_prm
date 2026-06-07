python3 << 'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Leitor(Node):
    def __init__(self):
        super().__init__('leitor')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/robot_cam/labels_map', self.cb, 10)
    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        canal = frame[:,:,0]
        vals, counts = np.unique(canal, return_counts=True)
        for v, c in sorted(zip(vals, counts), key=lambda x: -x[1])[:10]:
            print(f'label={v:3d}  pixels={c}')
        rclpy.shutdown()

rclpy.init()
rclpy.spin(Leitor())
EOF
