import time
import sys
import os

# Point to the local SDK path
sys.path.append("/workspace/sdk/unitree_sdk2_python")

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.core.channel import ChannelSubscriber

# Initialize network interface inside the container
ChannelFactoryInitialize(0, "eth0")

def callback(msg):
    # Only print when a key is actively pressed
    if msg.keys != 0:
        print(f"-> Actively Pressed Keys Value: {msg.keys}", flush=True)

if __name__ == "__main__":
    print("--- Telemetry Listener Booted ---")
    print("Press buttons on your Unitree remote controller to log their values...")
    
    # Notice the () appended here to instantiate the IDL class instance layout
    sub = ChannelSubscriber("rt/wireless_controller", unitree_go_msg_dds__WirelessController_())
    sub.Init(callback, 10)
    
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting telemetry check.")
