# manager for all things SNAC (shit nobody cares about)
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from dynamixel_sdk.group_sync_read import GroupSyncRead
import platform

def get_tty_port():
    system = platform.system()
    if system == "Windows":
        return "COM3"  # Adjust as necessary
    elif system == "Linux":
        return "/dev/ttyUSB0"  # Adjust as necessary
    elif system == "Darwin":  # macOS
        # return "/dev/tty.usbserial-FT763IB9"  # Adjust as necessary
        return "/dev/tty.usbserial-FT66WNPY"
    else:
        raise Exception("Unsupported operating system")

class PortPacketManager():
    def __init__(self, proto_ver = 2.0):
        self.PORT = get_tty_port()
        self.port = PortHandler(self.PORT)
        self.packet = PacketHandler(proto_ver)

    def getPortHandler(self):
        return self.port
    
    def getPacketHandler(self):
        return self.packet


PortPacketManager = PortPacketManager()