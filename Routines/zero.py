from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Mechas.MotorGroup import MotorGroup
from Misc.OpModes import OpModes


motor1 = DCMotor(0)
motor2 = DCMotor(1)
mg = MotorGroup(motor1, motor2)
goal = 1000000
class zero(Routine):
    def __init__(self):
        super().__init__()
        mg.enableTorque()
        mg.setOpmode(OpModes.EXTENDED_POSITION)
        # mg.forceZero()
        # motor1.forceZero()
        # motor2.forceZero()
        mg.setGoalPosition(goal)
    def loop(self):
        mg.getCurrentPosition()
        if all(mg.reachedPosition(goal, 100).values()):
            mg.forceZero()
    def run(self):
        # try:
        #     while True:
        #         self.loop()
        # except KeyboardInterrupt:
        #     print("Routine stopped by user.")
        #     self.onStop()
        super().run()
    def onStop(self):
        mg.stop()

if __name__ == "__main__":
    routine = zero()
    routine.run()