#Jan 18, 2026

from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Mechas.MotorGroupOLD import MotorGroup
from Misc.OpModes import OpModes
from Logic.Trajectory import Trajectory, TrajectoryManager
from Misc.deg import deg
import time, math
from enum import Enum

motor1 = DCMotor(1)
motor2 = DCMotor(2)
motor3 = DCMotor(3)
motor4 = DCMotor(4)
motors = MotorGroup(motor1, motor2,motor3, motor4)

traj1 = Trajectory(deg(90), deg(180), deg(270), deg(360))
trm1 = TrajectoryManager(traj1)
trm1.setLooped(True)

class fsm(Enum):
    POSXPOSY = 1
    NEGXPOSY = 2
    NEGXNEGY = 3
    POSXNEGY = 4

class test5(Routine):
    def __init__(self):
        super().__init__()
        motors.enableTorque()
        motors.setOpmode(OpModes.EXTENDED_POSITION)
        motors.forceZero()
        motors.setReverseMode(False)
        self.x = 0
        self.y = 0
        self.r = deg(90) # Radius = 151875
        self.x = self.r
        self.state = fsm.POSXPOSY
        print(f"Radius = {self.r}")
        while True:
            if all(x for x in motors.reachedGoalPosition(verbose=False).values()):
                print("All motors zeroed.")
                break
        motor1.setGoalPosition(self.r)
        while True:
            if motor1.reachedGoalPosition(verbose=False):
                print("Motor 1 reached starting position.")
                print("Type C to continue...")
                cmd = input()
                if cmd.lower() == 'c':
                    print("Continuing...")
                    break
    def loop(self):
        # dx = self.r - self.x
        # dy = math.sqrt(dx*(2*self.r-dx))
        # y = math.sqrt(self.r**2 - (self.r - self.x)**2)
        # motor1.setGoalPosition(self.x)
        # motor2.setGoalPosition(self.y)
        # if not motor1.isMoving(verbose=False) and not motor2.isMoving(verbose=False):
        #     if self.x - deg(5) >= 0:
        #         self.x -= deg(5)
        #     else:
        #         self.x = 0
        #     self.y = math.sqrt(self.r**2 - self.x**2)
        # motor1.setGoalPosition(self.x, verbose=False)
        # motor2.setGoalPosition(self.y, verbose=False)
        free = not(motor1.isMoving(verbose=False) or motor2.isMoving(verbose=False) or motor3.isMoving(verbose=False) or motor4.isMoving(verbose=False))
        if self.state == fsm.POSXPOSY:
            if free:
                self.x -= deg(5)
                self.y = math.sqrt(self.r**2 - self.x**2)
                motor1.setGoalPosition(self.x, verbose=False)
                motor2.setGoalPosition(self.y, verbose=False)
                if self.x <= 0:
                    self.state = fsm.NEGXPOSY
        if self.state == fsm.NEGXPOSY:
            if free:
                self.x -= deg(5)
                self.y = math.sqrt(self.r**2 - self.x**2)
                motor3.setGoalPosition(self.x, verbose=False)
                motor2.setGoalPosition(self.y, verbose=False)
                if self.y <= 0:
                    self.state = fsm.NEGXNEGY
        if self.state == fsm.NEGXNEGY:
            if free:
                self.x += deg(5)
                self.y = -math.sqrt(self.r**2 - self.x**2)
                motor3.setGoalPosition(self.x, verbose=False)
                motor4.setGoalPosition(self.y, verbose=False)
                if self.x >= 0:
                    self.state = fsm.POSXNEGY
        if self.state == fsm.POSXNEGY:
            if free:
                self.x += deg(5)
                self.y = -math.sqrt(self.r**2 - self.x**2)
                motor1.setGoalPosition(self.x, verbose=False)
                motor4.setGoalPosition(self.y, verbose=False)
                if self.y >= 0:
                    self.state = fsm.POSXPOSY
        print(f"X = {int(self.x)}, Y = {int(self.y)}")
        # motors.getCurrentPosition(verbose=True)
        # motor1.setGoalPosition((90 / motor1.AnglePerPosTick))
        # motor2.setGoalPosition(0)
        # trm1.follow(motor1)
        # trm1.followGroup(motors)
        #  if not self.completed:
        #     motor1.setGoalPosition(deg(90))
        #     if motor1.reachedGoalPosition(tolerance=50):
        #         motor2.setGoalPosition(deg(90))
        #         motor1.setGoalPosition(deg(0))
        #     if motor2.reachedGoalPosition(tolerance=50) and motor1.reachedGoalPosition(tolerance=50):
        #         self.completed = True
        #         print("Routine completed.")
        pass
    def run(self):
        try:
            while True:
                self.loop()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Routine stopped by user.")
            self.onStop()
    def onStop(self):
        motors.stop()
        

if __name__ == "__main__":
    routine = test5()
    routine.run()