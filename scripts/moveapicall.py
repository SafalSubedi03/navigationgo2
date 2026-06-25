import sys
import time

sys.path.append('/home/safal/Desktop/unitreego2nav/sdk/unitree_sdk2_python')

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

# ─────────────────────────────────────────
#  CONFIGURABLE PARAMETERS
# ─────────────────────────────────────────
NETWORK_INTERFACE = "eth0"   # <-- Change to your interface (check with: ip addr)
VX            =  0.5
VY            =  0.0
VYAW          =  0.0
DURATION_SEC  =  5.0
COMMAND_HZ    = 10
# ─────────────────────────────────────────

def main():
    # Initialize with network interface (pass nothing if no interface needed)
    if NETWORK_INTERFACE:
        ChannelFactoryInitialize(0, NETWORK_INTERFACE)
    else:
        ChannelFactoryInitialize(0)

    client = SportClient()
    client.SetTimeout(5.0)
    client.Init()

    print("Standing up...")
    client.BalanceStand()
    time.sleep(1.0)

    print(f"Moving: vx={VX}, vy={VY}, vyaw={VYAW} for {DURATION_SEC} seconds...")
    start = time.time()

    while time.time() - start < DURATION_SEC:
        code = client.Move(VX, VY, VYAW)
        if code != 0:
            print(f"Warning: Move returned code {code}")
        time.sleep(1.0 / COMMAND_HZ)

    print("Duration reached. Stopping...")
    client.StopMove()
    print("Done.")

if __name__ == "__main__":
    main()