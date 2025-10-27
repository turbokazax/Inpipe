from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Misc.OpModes import OpModes
import time

class test3(Routine):
    def __init__(self):
        super().__init__()
        # Initialize hardware here so it's tied to the instance
        self.motor1 = DCMotor(1)
        self.motor1.enableTorque()
        self.motor1.setOpMode(OpModes.POSITION)
        self.motor2 = DCMotor(2)
        self.motor2.enableTorque()
        self.motor2.setOpMode(OpModes.VELOCITY)
        self.motor1.zero()
        self.x = self.motor1.getMinPositionLimit(verbose=False)
    def loop(self):
        """Main loop for the test3 routine"""
        # Put your repeated actions here
        self.motor1.setGoalPosition(self.x)
        self.motor1.getCurrentPosition()
        if self.motor1.reachedGoalPosition():
            if self.x >= self.motor1.getMaxPositionLimit(verbose=False) - 10:
                self.x = self.motor1.getMinPositionLimit(verbose=False)
            elif self.x <= self.motor1.getMinPositionLimit(verbose=False) + 10:
                self.x = self.motor1.getMaxPositionLimit(verbose=False)
        self.motor2.setGoalVelocity(1000)
        pass

    def run(self):
        """Continuously run the loop until stopped manually (Ctrl/Cmd + C)"""
        try:
            while True:
                self.loop()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Routine stopped by user.")
            self.onStop()
    
    def onStop(self):
        print(self.motor1.getInfo()["opmode"])
        self.motor2.setGoalVelocity(0)
        self.motor1.zero()

if __name__ == "__main__":
    routine = test3()
    routine.run()