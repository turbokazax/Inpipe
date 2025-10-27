from dynamixel_sdk import *

PORT = "/dev/tty.usbserial-FT763IB9"
BAUD = 1_000_000
DXL_ID = 1
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE    = 11
ADDR_TORQUE_ENABLE     = 512
ADDR_GOAL_POSITION     = 564
ADDR_PRESENT_POSITION  = 580

TORQUE_ENABLE  = 1

def int32(u):  # interpret as signed
    return u - (1 << 32) if u & (1 << 31) else u

port = PortHandler(PORT)
packet = PacketHandler(PROTOCOL_VERSION)
assert port.openPort() and port.setBaudRate(BAUD)

# ping + check we’re in position mode
model, c, e = packet.ping(port, DXL_ID); print("ping:", model, c, e)
op, c, e = packet.read1ByteTxRx(port, DXL_ID, ADDR_OPERATING_MODE); print("opmode:", op)

# read current position
pos_u, c, e = packet.read4ByteTxRx(port, DXL_ID, ADDR_PRESENT_POSITION)
pos = int32(pos_u)
print("present:", pos)

# enable torque
c, e = packet.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
print("torque on:", c, e)

# move +10° relative (PH42 ≈ 1688 ticks/deg from your Wizard numbers 185000 ticks ≈ 109.63°)
TICKS_PER_DEG = 185000 / 109.63  # ≈ 1688.1
delta = int(10 * TICKS_PER_DEG)  # ~ +10 degrees
# goal = pos + delta
goal = 0

c, e = packet.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_POSITION, goal)
print("write goal:", c, e, "goal:", goal)

# watch it
import time
for i in range(15):
    time.sleep(0.1)
    p_u, c, e = packet.read4ByteTxRx(port, DXL_ID, ADDR_PRESENT_POSITION)
    print(i, int32(p_u))