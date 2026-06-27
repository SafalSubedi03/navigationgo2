import os
import sys
import time

# Dynamically inject local Unitree SDK into sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)

SDK_PATH = os.path.join(ROOT_DIR, "sdk", "unitree_sdk2_python")

if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)

# Imports
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient


# CONFIG

NETWORK_INTERFACE = "lo" #enp4s0

# Very small motion
VX = 0.4     # forward (m/s)
VY = 0.0      # sideways
VYAW = 0.0    # rotation

DURATION_SEC = 2.0
COMMAND_HZ = 25


def main():

    if NETWORK_INTERFACE:
        ChannelFactoryInitialize(0, NETWORK_INTERFACE)
    else:
        ChannelFactoryInitialize(0)

    client = SportClient()

    client.SetTimeout(10.0)

    client.Init()

    print("Standing up...")

    ret = client.StandUp()
    print("StandUp ret =", ret)

    time.sleep(3)

    ret = client.BalanceStand()
    print("BalanceStand ret =", ret)

    time.sleep(2)

    print(
        f"Moving slowly: vx={VX}, vy={VY}, yaw={VYAW}"
    )

    start = time.time()

    while time.time() - start < DURATION_SEC:

        ret = client.Move(VX, VY, VYAW)

        if ret != 0:
            print("Move ret =", ret)

        time.sleep(1.0 / COMMAND_HZ)

    print("Stopping...")

    for _ in range(10):
        client.StopMove()
        time.sleep(0.05)

    print("Done")


if __name__ == "__main__":
    main()
