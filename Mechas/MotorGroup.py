from Mechas.DCMotor import DCMotor
from Misc.OpModes import OpModes
from Misc.Colors import Colors

class MotorGroup:
    def __init__(self, *motors: DCMotor):
        self.motors = motors

    def enableTorque(self):
        for motor in self.motors:
            print(f"Motor {motor.DXL_ID}:")
            motor.enableTorque()

    def disableTorque(self):
        for motor in self.motors:
            motor.disableTorque()

    def getCurrentPosition(self):
        for motor in self.motors:
            # print(motor.getCurrentPosition(verbose=False))
            print(f"Motor {motor.DXL_ID}: {motor.getCurrentPosition(verbose=False)}")

    def setGoalPosition(self, goal):
        for motor in self.motors:
            motor.setGoalPosition(goal)
    
    def setOpmode(self, opmode: OpModes):
        for motor in self.motors:
            motor.setOpMode(opmode)
            
    def zero(self):
        for motor in self.motors:
            motor.zero(verbose = True)
    
    def forceZero(self):
        for motor in self.motors:
            motor.forceZero()

    def stop(self):
        for motor in self.motors:
            motor.stop()

    # INFO / STATE:
    def getInfo(self, verbose: bool = False):
        """
        Call getInfo() on all motors.
        Returns a list of info dicts in the same order as self.motors.
        """
        infos = []
        for motor in self.motors:
            info = motor.getInfo(verbose=verbose)
            if verbose:
                print(f"Motor {motor.DXL_ID}: {info}")
            infos.append(info)
        return infos

    def getOpMode(self, verbose: bool = True):
        """
        Get operation mode for all motors.
        Returns a dict {DXL_ID: OpMode}.
        """
        modes = {}
        for motor in self.motors:
            modes[motor.DXL_ID] = motor.getOpMode(verbose=verbose)
        return modes

    def isMoving(self, verbose: bool = True):
        """
        Check isMoving() for all motors.
        Returns a dict {DXL_ID: bool}.
        """
        states = {}
        for motor in self.motors:
            state = motor.isMoving(verbose=verbose)
            states[motor.DXL_ID] = state
        return states

    # POSITION HELPERS:
    def getGoalPosition(self):
        """
        Get goal position for all motors.
        Returns a dict {DXL_ID: goal_pos}.
        """
        goals = {}
        for motor in self.motors:
            goals[motor.DXL_ID] = motor.getGoalPosition(verbose=False)
        return goals

    def reachedGoalPosition(self, tolerance: int = 0, verbose: bool = True):
        """
        Check if all motors have reached their goal within a given tolerance.
        Returns a dict {DXL_ID: bool}.
        """
        reached = {}
        for motor in self.motors:
            r = motor.reachedGoalPosition(tolerance=tolerance, verbose=verbose)
            reached[motor.DXL_ID] = r
        return reached
    
    def reachedPosition(self, reference: int, tolerance: int = 0, verbose: bool = True):
        """
        Check if all motors have reached a specific position within a given tolerance.
        Returns a dict {DXL_ID: bool}.
        """
        reached = {}
        for motor in self.motors:
            r = motor.reachedPosition(reference, tolerance=tolerance, verbose=verbose)
            reached[motor.DXL_ID] = r
        return reached

    def rotateByAngle(self, angle: float, verbose: bool = True):
        """
        Rotate all motors by the same angle (in degrees).
        """
        for motor in self.motors:
            if verbose:
                print(f"Rotating motor {motor.DXL_ID} by {angle} degrees")
            motor.rotateByAngle(angle, verbose=verbose)

    # TORQUE:
    def getTorque(self, verbose: bool = True):
        """
        Get torque state (ON/OFF) for all motors.
        Returns a dict {DXL_ID: torque_state}.
        """
        states = {}
        for motor in self.motors:
            states[motor.DXL_ID] = motor.getTorque(verbose=verbose)
        return states

    # DRIVE MODE:
    def getDriveMode(self, verbose: bool = True):
        """
        Get drive mode for all motors.
        Returns a dict {DXL_ID: drive_mode_byte}.
        """
        modes = {}
        for motor in self.motors:
            modes[motor.DXL_ID] = motor.getDriveMode(verbose=verbose)
        return modes

    def setDriveMode(self, value: int, verbose: bool = True):
        """
        Set the same drive mode byte for all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"Setting drive mode of motor {motor.DXL_ID} to 0x{value & 0xFF:02X}")
            motor.setDriveMode(value, verbose=verbose)

    def setReverseMode(self, enable: bool = True, verbose: bool = True):
        """
        Enable / disable reverse mode on all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"{'Enabling' if enable else 'Disabling'} reverse mode for motor {motor.DXL_ID}")
            motor.setReverseMode(enable=enable, verbose=verbose)

    def invert(self, verbose: bool = True):
        """
        Toggle reverse mode for all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"Inverting direction for motor {motor.DXL_ID}")
            motor.invert()

    def setTorqueOnByGoalUpdate(self, enable: bool = True, verbose: bool = True):
        """
        Configure TorqueOnByGoalUpdate (bit3) on all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"{'Enabling' if enable else 'Disabling'} TorqueOnByGoalUpdate for motor {motor.DXL_ID}")
            motor.setTorqueOnByGoalUpdate(enable=enable, verbose=verbose)

    def setProfileTimeBased(self, enable: bool = True, verbose: bool = True):
        """
        Configure Time-based / Velocity-based profile for all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"Setting profile {'TIME' if enable else 'VELOCITY'} based for motor {motor.DXL_ID}")
            motor.setProfileTimeBased(enable=enable, verbose=verbose)

    # VELOCITY:
    def getCurrentVelocity(self, verbose: bool = True):
        """
        Get current velocity for all motors.
        Returns a dict {DXL_ID: velocity}.
        """
        velos = {}
        for motor in self.motors:
            velos[motor.DXL_ID] = motor.getCurrentVelocity(verbose=verbose)
        return velos

    def setGoalVelocity(self, velocity: int, verbose: bool = True, force: bool = False):
        """
        Set the same goal velocity for all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"Setting goal velocity of motor {motor.DXL_ID} to {velocity}")
            motor.setGoalVelocity(velocity, verbose=verbose, force=force)

    # PWM:
    def getPWMLimit(self, verbose: bool = False):
        """
        Get PWM limit for all motors.
        Returns a dict {DXL_ID: limit}.
        """
        limits = {}
        for motor in self.motors:
            limits[motor.DXL_ID] = motor.getPWMLimit(verbose=verbose)
        return limits

    def getGoalPWM(self, verbose: bool = True):
        """
        Get goal PWM for all motors.
        Returns a dict {DXL_ID: pwm}.
        """
        pwms = {}
        for motor in self.motors:
            pwms[motor.DXL_ID] = motor.getGoalPWM(verbose=verbose)
        return pwms

    def setGoalPWM(self, pwm: int, verbose: bool = True, force: bool = False):
        """
        Set the same goal PWM for all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"Setting goal PWM of motor {motor.DXL_ID} to {pwm}")
            motor.setGoalPWM(pwm, verbose=verbose, force=force)

    # CURRENT / TORQUE MODE:
    def getGoalCurrent(self, verbose: bool = True):
        """
        Get goal current for all motors.
        Returns a dict {DXL_ID: current}.
        """
        currents = {}
        for motor in self.motors:
            currents[motor.DXL_ID] = motor.getGoalCurrent(verbose=verbose)
        return currents

    def setGoalCurrent(self, current: int, verbose: bool = True):
        """
        Set the same goal current for all motors.
        """
        for motor in self.motors:
            if verbose:
                print(f"Setting goal current of motor {motor.DXL_ID} to {current}")
            motor.setGoalCurrent(current, verbose=verbose)

    # LED:
    def getLEDColor(self, verbose: bool = True):
        """
        Get LED color (R,G,B) for all motors.
        Returns a dict {DXL_ID: (r, g, b)}.
        """
        colors = {}
        for motor in self.motors:
            col = motor.getLEDColor(verbose=verbose)
            colors[motor.DXL_ID] = col
        return colors

    def setLEDColor(self, color, verbose: bool = True):
        """
        Set the same LED color for all motors.
        'color' should be either a Colors enum value (.value) or an (R,G,B) tuple,
        matching what DCMotor.setLEDColor expects.
        """
        for motor in self.motors:
            if verbose:
                print(f"Setting LED color for motor {motor.DXL_ID} to {color}")
            motor.setLEDColor(color, verbose=verbose)