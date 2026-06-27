import os
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Dynamically inject local Unitree SDK into sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
SDK_PATH = os.path.join(ROOT_DIR, "sdk", "unitree_sdk2_python")

if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)

# Unitree SDK Imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

NETWORK_INTERFACE = "enp4s0"

#Limiters 
vxlimit = 0.4
vylimit = 0.4 
vyawlimit = 0.2

class Go2SportapiBridge(Node):
    def __init__(self):
        super().__init__('go2_Sportapi_bridge')
        
        # 1. Initialize Unitree SDK Channel
        if NETWORK_INTERFACE:
            ChannelFactoryInitialize(0, NETWORK_INTERFACE)
        else:
            ChannelFactoryInitialize(0)

        # 2. Setup Unitree Sport Client
        self.client = SportClient()
        self.client.SetTimeout(10.0)
        self.client.Init()

        # 3. Boot sequence to get the robot ready
        self.get_logger().info("Standing up Unitree Go2...")
        self.client.StandUp()
        time.sleep(3.0)
        
        self.client.BalanceStand()
        time.sleep(1.0)
        self.get_logger().info("Robot is standing and balancing. Ready for commands.")

        self.sub = self.create_subscription(Twist,'/cmd_vel_manual',self.cmd_vel_callback,10)

    def cmd_vel_callback(self, msg:Twist):
        vx = msg.linear.x 
        vy = msg.linear.y 
        vyaw = msg.angular.z 
        #hardcoded clamping
        if (vxlimit < vx or vylimit < vy or vyawlimit < vyaw):
            self.get_logger().warn(f"Received velocity is above the limiting value. vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
            return

        self.get_logger().debug(f"Sending Move: vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")

        ret = self.client.Move(vx,vy,vyaw)
        if ret != 0:
            self.get_logger().warn(f"SportClient Move failed return code: {ret}")

    def stop_robot(self):
        """Safe shutdown sequence when the node is killed."""
        self.get_logger().info("Stopping robot motion...")
        for _ in range(5):
            self.client.StopMove()
            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = Go2SportapiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


