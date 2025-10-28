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
        self.AnglePerPosTick = 0.00059259
        self.rotated = False
        self.rot_counter = 0
        self.rot_allowed = 0
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
        match opmode:
            case 0: opmode = OpModes.TORQUE
            case 1: opmode = OpModes.VELOCITY
            case 3: opmode = OpModes.POSITION
            case 4: opmode = OpModes.EXTENDED_POSITION
            case 16: opmode = OpModes.PWM
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

    def isMoving(self, verbose=True):
        status, c, e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.MOVING.value)
        if verbose:
            print(f"Motor {self.DXL_ID} is", "MOVING" if status else "NOT moving")
        return status

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
        if opmode == 3:
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
    
    def rotateByAngle(self, angle, times = 1, verbose = True):
        # if self.reachedGoalPosition() and not self.rotated:
        #     pos = self.getCurrentPosition()
        #     # 1 pos = 0.00059259 deg
        #     angle = round(angle / self.AnglePerPosTick)
        #     pos += angle
        #     self.setGoalPosition(pos)
        #     self.rotated = True
        # assert times >= 1, "times should be at least 1"
        # if self.reachedGoalPosition() and self.rot_counter < self.rot_allowed:
        #     self.rot_allowed += times
        #     pos = self.getCurrentPosition()
        #     # 1 pos = 0.00059259 deg
        #     angle = round(angle / self.AnglePerPosTick)
        #     pos += angle
        #     self.setGoalPosition(pos)
        #     self.rot_counter+=1
        if not self.isMoving():
            pos = self.getCurrentPosition()
            # 1 pos = 0.00059259 deg
            angle = round(angle / self.AnglePerPosTick)
            pos += angle
            self.setGoalPosition(pos)

    def zero(self, verbose = True):
        opmode = self.getInfo()["opmode"]
        assert opmode == 3 or opmode == 4, "Motor must be in POSITION (3) or EXTENDED POSITION (4) mode"
        self.setGoalPosition(0, verbose)

    def forceZero(self, verbose = True):
        opmode = self.getInfo()["opmode"]
        if opmode != OpModes.EXTENDED_POSITION or opmode != OpModes.POSITION:
            self.setOpMode(OpModes.POSITION, verbose=False)
            self.setGoalPosition(0, verbose)
            if verbose: print(f"Motor {self.DXL_ID} force zeroed")
            while not self.reachedGoalPosition(1):
                continue
            self.setOpMode(opmode)
        else:
            self.zero(verbose = verbose)
    
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

    # DRIVE MODE (Address: CT.DRIVE_MODE)
    def getDriveMode(self, verbose=True):
        dm, c, e = self.packet.read1ByteTxRx(self.port, self.DXL_ID, CT.DRIVE_MODE.value)
        if verbose:
            print(f"Drive Mode: 0x{dm:02X}")
            print(f"  - Reverse: {'ON' if (dm & 0x01) else 'OFF'};  Profile: {'Time' if (dm & 0x04) else 'Velocity'};  TorqueOnByGoalUpdate: {'ON' if (dm & 0x08) else 'OFF'}")
        return dm

    def setDriveMode(self, value, verbose=True):
        # Drive Mode is in EEPROM; torque must be disabled before writing.
        torque = self.getTorque(verbose=False)
        if torque == 1:
            self.disableTorque(verbose=False)
        c, e = self.packet.write1ByteTxRx(self.port, self.DXL_ID, CT.DRIVE_MODE.value, value & 0xFF)
        if torque == 1:
            self.enableTorque(verbose=False)
        if verbose:
            print(f"Set Drive Mode to 0x{value & 0xFF:02X} -> resp:", c, e)
        return value & 0xFF

    def setReverseMode(self, enable=True, verbose=True):
        """Toggle Reverse Mode (bit0). When enabled: positive command becomes clockwise, negative is counterclockwise."""
        dm = self.getDriveMode(verbose=False)
        if enable:
            dm |= 0x01  # set bit0
        else:
            dm &= ~0x01  # clear bit0
        return self.setDriveMode(dm, verbose=verbose)

    def invert(self):
        dm = self.getDriveMode()
        dm = dm & 0x01
        if dm:
            self.setReverseMode(False) # unreverse if reverse = ON
        else:
            self.setReverseMode(True) # reverse if reverse = OFF

    def setTorqueOnByGoalUpdate(self, enable=True, verbose=True):
        """Toggle Torque On by Goal Update (bit3) without disturbing other bits."""
        dm = self.getDriveMode(verbose=False)
        if enable:
            dm |= 0x08
        else:
            dm &= ~0x08
        return self.setDriveMode(dm, verbose=verbose)

    def setProfileTimeBased(self, enable=True, verbose=True):
        """Configure profile to Time-based (bit2=1) or Velocity-based (bit2=0)."""
        dm = self.getDriveMode(verbose=False)
        if enable:
            dm |= 0x04
        else:
            dm &= ~0x04
        return self.setDriveMode(dm, verbose=verbose)

    # general force stop:
    def stop(self, verbose = True):
        self.port = PortHandler(self.PORT)
        self.packet = PacketHandler(self.PROTOCOL_VER)
        assert self.port.openPort() and self.port.setBaudRate(self.BAUD), f"Failed to open port {self.PORT} at {self.BAUD} baud."
        self.setGoalPWM(0, force = True)
        self.setGoalVelocity(0, force = True)
        self.port.closePort()

    # VELOCITY:
    def getCurrentVelocity(self, verbose = True):
        velo, c, e = self.packet.read4ByteTxRx(self.port, self.DXL_ID, CT.CURRENT_VELOCITY.value)
        if verbose: print("Current Velocity:", int32(velo))
        return int32(velo)
    
    def setGoalVelocity(self, velocity, verbose = True, force = False):
        opmode = self.getInfo()["opmode"]
        assert opmode == 1 or force, "Motor must be in VELOCITY (1) mode"
        c, e = self.packet.write4ByteTxRx(self.port, self.DXL_ID, CT.GOAL_VELOCITY.value, velocity)
        if verbose: print("Set Goal Velocity:", velocity, "Response:", c, e)

    # PWM:
    def getPWMLimit(self, verbose = False):
        limit, c, e = self.packet.read2ByteTxRx(self.port, self.DXL_ID, CT.PWM_LIMIT.value)
        if verbose: print("PWM Limit:", int32(limit))
        return int32(limit)
    
    def getGoalPWM(self, verbose = True):
        pwm, c, e = self.packet.read2ByteTxRx(self.port, self.DXL_ID, CT.GOAL_PWM.value)
        if verbose: print("Goal PWM:", int32(pwm))
        return int32(pwm)
    
    def setGoalPWM(self, pwm, verbose = True, force = False):
        opmode = self.getInfo()["opmode"]
        assert opmode == 16 or force, "Motor must be in PWM (16) mode"
        if abs(pwm) > self.getPWMLimit(verbose=False):
            pwm = self.getPWMLimit(verbose=False) * (1 if pwm >0 else -1)
        c, e = self.packet.write2ByteTxRx(self.port, self.DXL_ID, CT.GOAL_PWM.value, pwm)
        if verbose: print("Set Goal PWM:", pwm, "Response:", c, e)
    
    def getGoalCurrent(self, verbose = True):
        current, c, e = self.packet.read2ByteTxRx(self.port, self.DXL_ID, CT.GOAL_CURRENT.value)
        if verbose: print("Goal Current:", int32(current))
        return int32(current)
    
    def setGoalCurrent(self, current, verbose = True):
        opmode = self.getInfo()["opmode"]
        assert opmode == 0, "Motor must be in TORQUE (0) mode"
        c, e = self.packet.write2ByteTxRx(self.port, self.DXL_ID, CT.GOAL_CURRENT.value, current)
        if verbose: print("Set Goal Current:", current, "Response:", c, e)

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
    
