from Mechas.DCMotor import DCMotor
from Misc.OpModes import OpModes

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
            print(motor.getCurrentPosition(verbose=False))

    def setGoalPosition(self, goal):
        for motor in self.motors:
            motor.setGoalPosition(goal)
    
    def setOpmode(self, opmode: OpModes):
        for motor in self.motors:
            motor.setOpMode(opmode)

    def zero(self):
        for motor in self.motors:
            motor.zero(verbose = True)