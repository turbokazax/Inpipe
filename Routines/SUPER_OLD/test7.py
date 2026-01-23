# Jan 19

from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Mechas.MotorGroupOLD import MotorGroup
from Misc.OpModes import OpModes
from Logic.Trajectory import Trajectory, TrajectoryManager
from Misc.deg import deg
import time, math
from enum import Enum

import socket

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def safe_sqrt_circle(r, x):
    # sqrt(max(0, r^2 - x^2)) to avoid negatives from rounding
    inside = (r * r) - (x * x)
    if inside < 0:
        inside = 0
    return math.sqrt(inside)


motor1 = DCMotor(1)
motor2 = DCMotor(2)
motor3 = DCMotor(3)
motor4 = DCMotor(4)
motors = MotorGroup(motor1, motor2, motor3, motor4)

# traj1 = Trajectory(deg(90), deg(180), deg(270), deg(360))
# trm1 = TrajectoryManager(traj1)
# trm1.setLooped(True)

class fsm(Enum):
    POSXPOSY = 1
    NEGXPOSY = 2
    NEGXNEGY = 3
    POSXNEGY = 4


class test7(Routine):
    def __init__(self):
        super().__init__()
        motors.enableTorque()
        motors.setOpmode(OpModes.EXTENDED_POSITION)
        motors.setReverseMode(False)

        self.r = deg(180)      # radius in ticks
        
        # tolerance for snapping y to 0 near the axis (ticks)
        # choose something small-ish vs your step; tweak if needed
        self.y_zero_tol = max(1, deg(0.5))

        # Start at (x=+r, y=0)
        self.x = self.r
        self.y = 0
        self.state = fsm.POSXPOSY
        print(f"Radius = {self.r}")

        self.theta = 0 # starting, running angle
        # UDP setup for live X,Y plotting in viewer_xy.py:
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._viewer_addr = ("127.0.0.1", 9999)
        self._last_send = 0.0
        self._send_period = 1.0 / 30.0   # 30 Hz to the viewer (plenty smooth)
        self.last_time_s = time.time() # placeholder for time
        motor2.forceZero()
        motor3.forceZero()
        motor4.forceZero()
        motor1.setGoalPosition(int(self.x))

        motor1.setProfileVelocity(0, verbose=True)
        motor1.setProfileAcceleration(0, verbose=True)
        motor2.setProfileVelocity(0, verbose=True)
        motor2.setProfileAcceleration(0, verbose=True)
        # Wait for motors to reach zero (avoid hot spin)
        while True:
            # print(v for v in motors.reachedGoalPosition(verbose=True).values())
            # if all(v for v in motors.reachedGoalPosition(verbose=False).values()):
            if motor2.reachedGoalPosition(verbose=False) and motor3.reachedGoalPosition(verbose=False) and motor4.reachedGoalPosition(verbose=False):
                print("Motors 2,3,4 zeroed.")
            if motor1.reachedGoalPosition(verbose=False):
                print("Motor 1 reached starting position.")
                print("Type C to continue...")
                cmd = input()
                if cmd.lower() == 'c':
                    self.last_time_s = time.time()  # seconds; for measuring loop timing
                    print("Continuing...")
                    break
            time.sleep(0.01)
    def loop(self):
        now_s = time.time()
        dt_s = now_s - self.last_time_s
        self.last_time_s = now_s

        # Guard against the very first iteration or any large scheduling hiccups
        if dt_s <= 0 or dt_s > 0.25:
            dt_s = 0.01

        t = 3.2  # seconds per quarter rev
        omega = (math.pi / 2) / t  # rad/s

        self.theta += omega * dt_s
        self.theta = min(self.theta, math.pi/2) # for 1st quadrant

        self.x = round(self.r * math.cos(self.theta))
        self.y =round(self.r * math.sin(self.theta))

        motor1.setGoalPosition(int(self.x))
        motor2.setGoalPosition(int(self.y))

        # UDP send X,Y for live pos tracking:
        now = time.time()
        if now - self._last_send >= self._send_period:
            self._last_send = now
            # x,y,r,state
            # msg = f"{int(self.x)},{int(self.y)},{int(self.r)},{self.state.name}\n"
            posm1 = motor1.getCurrentPosition(verbose=False)
            posm2 = motor2.getCurrentPosition(verbose=False)
            posm3 = motor3.getCurrentPosition(verbose=False)
            posm4 = motor4.getCurrentPosition(verbose=False)
            if self.state == fsm.POSXPOSY:
                msg = f"{int(posm1)},{int(posm2)},{int(self.r)},{self.state.name}\n"
            elif self.state == fsm.NEGXPOSY:
                msg = f"{int(posm3)},{int(posm2)},{int(self.r)},{self.state.name}\n"
            elif self.state == fsm.NEGXNEGY:
                msg = f"{int(posm3)},{int(posm4)},{int(self.r)},{self.state.name}\n"
            elif self.state == fsm.POSXNEGY:
                msg = f"{int(posm1)},{int(posm4)},{int(self.r)},{self.state.name}\n"
            try:    
                self._udp.sendto(msg.encode(), self._viewer_addr)
            except OSError:
                pass
        print(f"X = {int(self.x)}, Y = {int(self.y)}, state = {self.state.name}")

    def run(self):
        try:
            while True:
                self.loop()
                # time.sleep(dt)
        except KeyboardInterrupt:
            print("Routine stopped by user.")
            self.onStop()

    def onStop(self):
        motors.stop()


if __name__ == "__main__":
    routine = test7()
    routine.run()