import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import os, sys, time
import cv2
from cv_bridge import CvBridge
import numpy as np 


# Dynamically inject local Unitree SDK into sys.path by climbing up to the workspace root
current_dir = os.path.dirname(os.path.abspath(__file__))
SDK_PATH = None

while current_dir != os.path.dirname(current_dir):  # Stop if we hit the filesystem root '/'
    possible_sdk = os.path.join(current_dir, "sdk", "unitree_sdk2_python")
    if os.path.exists(possible_sdk):
        SDK_PATH = possible_sdk
        break
    current_dir = os.path.dirname(current_dir)

if SDK_PATH and SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)
else:
    print(f"Warning: Could not find Unitree SDK path dynamically!", file=sys.stderr)

# Unitree SDK Imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

NETWORK_INTERFACE = "enp4s0"

class cameraimg(Node):
    def __init__(self):
        super().__init__('cameraimg')

        if NETWORK_INTERFACE:
            ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        else:
            ChannelFactoryInitialize(0)

        # 2. Setup Unitree Sport Client
        self.client = VideoClient()
        self.client.SetTimeout(30.0)
        self.client.Init()

        self.publisher = self.create_publisher(Image,'go2/camera',10)
        self.bridge = CvBridge()
        self.timer_period = 0.04
        self.timer = self.create_timer(self.timer_period,self.timer_callback)
        self.get_logger().info("Go2 Camera Node has started.")

    def timer_callback(self):
        code, data = self.client.GetImageSample()
        if code == 0:
            try:
                image_data = np.frombuffer(bytes(data), dtype=np.uint8)
                cv_image = cv2.imdecode(image_data,cv2.IMREAD_COLOR)

                if cv_image is not None:
                    ros_image_msg = self.bridge.cv2_to_imgmsg(cv_image,encoding="bgr8")
                    ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                    ros_image_msg.header.frame_id = "base_link"

                    self.publisher.publish(ros_image_msg)
                else:
                    self.get_logger().warn("Decoded image is empty.")

            except Exception as e:
                self.get_logger().error(f"Failed to convert or publish image: {str(e)}")
        else: 
            self.get_logger().error(f"GetImageSample failed with error code: {code}")

def main(args=None):
    rclpy.init(args=args)
    node = cameraimg()
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Go2 Camera Node...")    
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()




        


        
