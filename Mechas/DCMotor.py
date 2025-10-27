from dynamixel_sdk import *
from Misc.ControlTable import ControlTable as CT
from Misc.int32 import int32
from Misc.OpModes import OpModes
from Misc.Colors import Colors

class DCMotor:
    def __init__(self, DXL_ID, BAUD=1e6, PORT="/dev/tty.usbserial-FT763IB9"):
        self.DXL_ID = DXL_ID;
        self.PORT = PORT;
        self.BAUD = BAUD;
        self.PROTOCOL_VER = 2;
        self.port = PortHandler(self.PORT)
        self.packet = PacketHandler(self.PROTOCOL_VER)
        assert self.port.openPort() and self.port.setBaudRate(self.BAUD), f"Failed to open port {self.PORT} at {self.BAUD} baud."
        print(f"Motor {self.DXL_ID} initialized:")
        self.getInfo(verbose=True)
        match self.DXL_ID:
            case 1:
                self.setLEDColor(Colors.RED.value, verbose=False)
            case 2:
                self.setLEDColor(Colors.GREEN.value, verbose=False)
            case 3:
                self.setLEDColor(Colors.BLUE.value, verbose=False)
            case 4:
                self.setLEDColor(Colors.YELLOW.value, verbose=False)

    def getInfo(self, verbose = False):
        if verbose: print("ID:", self.DXL_ID)
        model, c, e = self.packet.ping(self.port, self.DXL_ID)
        if verbose: print("ping:", model, c, e)
        opmode, c,e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.OPERATING_MODE.value)
        if verbose: print("opmode", opmode)
        return {"model": model,
                "opmode": opmode
                }
    def setOpMode(self, opmode: OpModes, verbose=True):
        torque = self.getTorque(verbose=False)
        if torque == 1: self.disableTorque(verbose=False)
        c, e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.OPERATING_MODE.value, opmode.value)
        if torque == 1: self.enableTorque(verbose=False)
        info = self.getInfo(verbose=False)
        print(f"ID {self.DXL_ID}: set OpMode - {info['opmode']} - {OpModes(info['opmode']).name}")
    # POSITION:
    def getCurrentPosition(self, verbose = True):
        pos, c, e = self.packet.read4ByteTxRx(self.port, self.DXL_ID, CT.CURRENT_POSITION.value)
        if verbose: print("Position:", int32(pos))
        return int32(pos)
    
    def getGoalPosition(self, verbose = True):
        goal, c, e = self.packet.read4ByteTxRx(self.port, self.DXL_ID, CT.GOAL_POSITION.value)
        if verbose: print("Goal Position:", int32(goal))
        return int32(goal)
    
    def getMaxPositionLimit(self, verbose = False):
        max_pos, c,e, = self.packet.read4ByteTxRx(self.port, self.DXL_ID, CT.MAX_POSITION_LIMIT.value)
        if verbose: print("Max Position Limit:", int32(max_pos))
        return int32(max_pos)

    def getMinPositionLimit(self, verbose = False):
        min_pos, c,e, = self.packet.read4ByteTxRx(self.port, self.DXL_ID, CT.MIN_POSITION_LIMIT.value)
        if verbose: print("Min Position Limit:", int32(min_pos))
        return int32(min_pos)

    def setGoalPosition(self, goal, verbose = True):
        opmode = self.getInfo()["opmode"]
        assert opmode == 3 or opmode == 4, "Motor must be in POSITION (3) or EXTENDED POSITION (4) mode"
        if goal > self.getMaxPositionLimit(verbose=False):
            goal = self.getMaxPositionLimit(verbose=False)
        if goal < self.getMinPositionLimit(verbose=False):
            goal = self.getMinPositionLimit(verbose=False)
        c, e = self.packet.write4ByteTxRx(self.port, self.DXL_ID, CT.GOAL_POSITION.value, goal)
        if verbose: print("write goal:", c,e, "goal:", goal)

    def reachedGoalPosition(self, tolerance = 0, verbose = True):
        delta = abs(self.getCurrentPosition(verbose=False) - self.getGoalPosition(verbose=False))
        print("Delta:", delta, "Reached?", delta <= tolerance)
        return delta <= tolerance
    
    def zero(self, verbose = True):
        opmode = self.getInfo()["opmode"]
        assert opmode == 3 or opmode == 4, "Motor must be in POSITION (3) or EXTENDED POSITION (4) mode"
        self.setGoalPosition(0, verbose)
    
    # TORQUE:
    def getTorque(self, verbose = True):
        tm, c, e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.TORQUE_ENABLE.value)
        state = "ON" if tm == 1 else "OFF"
        if verbose: print(f"Torque: {state}")
        return tm
    def enableTorque(self, verbose = True):
        c, e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.TORQUE_ENABLE.value, 1)
        if e == 0 and verbose: print("Torque enabled")
    def disableTorque(self, verbose = True):
        c, e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.TORQUE_ENABLE.value, 0)
        if e == 0 and verbose: print("Torque disabled")

    # VELOCITY:
    def getCurrentVelocity(self, verbose = True):
        velo, c, e = self.packet.read4ByteTxRx(self.port, self.DXL_ID, CT.CURRENT_VELOCITY.value)
        if verbose: print("Current Velocity:", int32(velo))
        return int32(velo)
    
    def setGoalVelocity(self, velocity, verbose = True):
        opmode = self.getInfo()["opmode"]
        assert opmode == 1, "Motor must be in VELOCITY (1) mode"
        c, e = self.packet.write4ByteTxRx(self.port, self.DXL_ID, CT.GOAL_VELOCITY.value, velocity)
        if verbose: print("Set Goal Velocity:", velocity, "Response:", c, e)


    # LED: 
    def getLEDColor(self, verbose = True):
        red, c, e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.LED_RED)
        green, c, e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.LED_GREEN)
        blue, c, e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.LED_BLUE)
        return (red, green, blue)

    def setLEDColor(self, color: Colors, verbose = True):
        r = color[0]
        g = color[1]
        b = color[2]
        c,e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.LED_RED.value, r)
        c,e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.LED_GREEN.value, g)
        c,e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.LED_BLUE.value, b)
        if verbose: print(f"LED Color set to: R:{r} G:{g} B:{b}")
    
