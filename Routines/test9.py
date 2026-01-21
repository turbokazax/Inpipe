# Jan 19

from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Mechas.MotorGroup import MotorGroup
from Misc.OpModes import OpModes
from Logic.Trajectory import Trajectory, TrajectoryManager
from Misc.deg import deg
import time, math
from enum import Enum
from Logic.PIDController import PIDController

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


class test9(Routine):
    def __init__(self):
        super().__init__()
        motors.enableTorque()
        motors.setOpmode(OpModes.EXTENDED_POSITION)
        motors.setReverseMode(False)

        # self.PID = PIDController()
        self.PIDx = PIDController() # for motor1 (X) correction on X-axis error
        self.PIDy = PIDController() # for motor2 (Y) correction on Y-axis error
        
        self.r = deg(180)      # radius in ticks
        
        # tolerance for snapping y to 0 near the axis (ticks)
        # choose something small-ish vs your step; tweak if needed
        self.y_zero_tol = max(1, deg(0.5))

        # Start at (x=+r, y=0)
        self.x = self.r
        self.y = 0
        self.state = fsm.POSXPOSY
        self.theta = 0

        # self.dx = -250 # initial velocity of motor1 
        # self.dy = 0
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

        self.switching_motors = False

        motor1.setProfileVelocity(1500, verbose=True)
        motor1.setProfileAcceleration(10765, verbose=True)
        motor2.setProfileVelocity(1500, verbose=True)
        motor2.setProfileAcceleration(10765, verbose=True)
        motor3.setProfileVelocity(1500, verbose=True)
        motor3.setProfileAcceleration(10765, verbose=True)
        motor4.setProfileVelocity(1500, verbose=True)
        motor4.setProfileAcceleration(10765, verbose=True)
        # Wait for motors to reach zero (avoid hot spin)
        while True:
            # print(v for v in motors.reachedGoalPosition(verbose=True).values())
            # if all(v for v in motors.reachedGoalPosition(verbose=False).values()):
            if motor2.reachedGoalPosition(verbose=False) and motor3.reachedGoalPosition(verbose=False) and motor4.reachedGoalPosition(verbose=False) and motor1.reachedGoalPosition(verbose=False):
                print("Motors 2,3,4 zeroed.")
                print("Motor 1 reached starting position.")
                print("Type C to continue...")
                cmd = input()
                if cmd.lower() == 'c':
                    self.last_time_s = time.time()  # seconds; for measuring loop timing
                    # motors.setOpmode(OpModes.VELOCITY)
                    motors.setOpmode(OpModes.VELOCITY)
                    print("Continuing...")
                    break
            time.sleep(0.01)
        
    def loop(self):
        now_s = time.time()
        dt_s = now_s - self.last_time_s
        self.last_time_s = now_s
        active_motor_x = None
        active_motor_y = None

        # Guard first iteration / hiccups
        if dt_s <= 0 or dt_s > 0.25:
            dt_s = 0.01

        # motion timing
        t = 6  # seconds per quarter circle
        omega = (math.pi / 2.0) / t  # rad/s

        # advance param
        self.theta = self.theta % (2.0 * math.pi)  # wraparound
        # if self.theta >= 0.0 and self.theta < (math.pi / 2.0):
        #     self.state = fsm.POSXPOSY
        # elif self.theta >= math.pi / 2.0 and self.theta < math.pi:
        #     self.state = fsm.NEGXPOSY
        # elif self.theta >= math.pi and self.theta < (3.0 * math.pi / 2.0):
        #     self.state = fsm.NEGXNEGY
        # elif self.theta >= (3.0 * math.pi / 2.0) and self.theta < (2.0 * math.pi):
        #     self.state = fsm.POSXNEGY
        
        if not self.switching_motors:
            self.theta += omega * dt_s
        # self.theta = min(self.theta, math.pi / 2.0)

        # desired position (ticks)
        self.x = self.r * math.cos(self.theta)
        self.y = self.r * math.sin(self.theta)

        # desired velocity in ticks/s (feedforward)
        vx_ticks_s = -self.r * math.sin(self.theta) * omega
        vy_ticks_s =  self.r * math.cos(self.theta) * omega

        # read actual position (ticks)
        # pos_m1 = motor1.getCurrentPosition(verbose=False)
        # pos_m2 = motor2.getCurrentPosition(verbose=False)
        if self.state == fsm.POSXPOSY:
            pos_m1 = motor1.getCurrentPosition(verbose=False)
            pos_m2 = motor2.getCurrentPosition(verbose=False)
        elif self.state == fsm.NEGXPOSY:
            pos_m1 = motor3.getCurrentPosition(verbose=False)
            pos_m2 = motor2.getCurrentPosition(verbose=False)
        elif self.state == fsm.NEGXNEGY:
            pos_m1 = motor3.getCurrentPosition(verbose=False)
            pos_m2 = motor4.getCurrentPosition(verbose=False)
        elif self.state == fsm.POSXNEGY:
            pos_m1 = motor1.getCurrentPosition(verbose=False)
            pos_m2 = motor4.getCurrentPosition(verbose=False)
        # position error (ticks)
        err_x = self.x - pos_m1
        err_y = self.y - pos_m2

        # --- convert ticks/s -> GoalVelocity units (0.01 rev/min) ---
        TICKS_PER_REV = 607_500.0

        def ticks_s_to_gv_units(v_ticks_s: float) -> int:
            rpm = (v_ticks_s / TICKS_PER_REV) * 60.0
            return int(round(rpm * 100.0))  # 0.01 rpm units

        gvx_ff = ticks_s_to_gv_units(vx_ticks_s)
        gvy_ff = ticks_s_to_gv_units(vy_ticks_s)

        # --- small feedback term (P on position error), in GoalVelocity units ---
        Kp_v = 0.01  # (0.01 rpm units) per tick

        gvx = gvx_ff + int(round(Kp_v * err_x))
        gvy = gvy_ff + int(round(Kp_v * err_y))

        # clamp to motor's allowed goal velocity range
        gvx = clamp(gvx, -1500, 1500)
        gvy = clamp(gvy, -1500, 1500)

        if self.state == fsm.POSXPOSY:
            # motor1.setGoalVelocity(gvx, verbose=False)
            # motor2.setGoalVelocity(gvy, verbose=False)
            # motor3.setGoalVelocity(0, verbose=False)
            # motor4.setGoalVelocity(0, verbose=False)
            if abs(motor1.getCurrentPosition(verbose=False)) <= 0 or abs(motor2.getCurrentPosition(verbose=False)) >= self.r:
                self.switching_motors = True
                motor1.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor2.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor1.setGoalPosition(0, verbose=True)
                motor2.setGoalPosition(int(self.r), verbose=True)
                # motor3.setCurrentPosition(motor1.getCurrentPosition(verbose=False), verbose=True)
                if motor1.reachedGoalPosition(verbose=False) and motor2.reachedGoalPosition(verbose=False):
                    motor1.setOpMode(OpModes.VELOCITY, verbose=True)
                    motor2.setOpMode(OpModes.VELOCITY, verbose=True)
                    self.switching_motors = False
                    self.state = fsm.NEGXPOSY
            else:
                motor1.setGoalVelocity(gvx, verbose=False)
                motor2.setGoalVelocity(gvy, verbose=False)
                motor3.setGoalVelocity(0, verbose=False)
                motor4.setGoalVelocity(0, verbose=False)
        elif self.state == fsm.NEGXPOSY:
            # motor1.setGoalVelocity(0, verbose=False)
            # motor4.setGoalVelocity(0, verbose=False)
            # motor3.setGoalVelocity(gvx, verbose=False)
            # motor2.setGoalVelocity(gvy, verbose=False)
            # if self.theta >= math.pi:
            if abs(motor3.getCurrentPosition(verbose=False)) >= self.r or abs(motor2.getCurrentPosition(verbose=False)) <= 0:
                self.switching_motors = True
                motor3.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor2.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor3.setGoalPosition(int(self.r), verbose=True)
                motor2.setGoalPosition(0, verbose=True)
                if motor3.reachedGoalPosition(verbose=False) and motor2.reachedGoalPosition(verbose=False):
                    motor3.setOpMode(OpModes.VELOCITY, verbose=True)
                    motor2.setOpMode(OpModes.VELOCITY, verbose=True)
                    self.switching_motors = False
                    self.state = fsm.NEGXNEGY
            else:
                motor1.setGoalVelocity(0, verbose=False)
                motor4.setGoalVelocity(0, verbose=False)
                motor3.setGoalVelocity(gvx, verbose=False)
                motor2.setGoalVelocity(gvy, verbose=False)
        elif self.state == fsm.NEGXNEGY:
            # motor2.setGoalVelocity(0, verbose=False)
            # motor1.setGoalVelocity(0, verbose=False)
            # motor3.setGoalVelocity(gvx, verbose=False)
            # motor4.setGoalVelocity(gvy, verbose=False)
            # # if self.theta >= (3.0 * math.pi / 2.0):
            if abs(motor3.getCurrentPosition(verbose=False)) <= 0 or abs(motor4.getCurrentPosition(verbose=False)) >= 0:
                switching_motors = True
                motor3.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor4.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor3.setGoalPosition(0, verbose=True)
                motor4.setGoalPosition(0, verbose=True)
                if motor3.reachedGoalPosition(verbose=False) and motor4.reachedGoalPosition(verbose=False):
                    motor3.setOpMode(OpModes.VELOCITY, verbose=True)
                    motor4.setOpMode(OpModes.VELOCITY, verbose=True)
                    self.switching_motors = False
                    self.state = fsm.POSXNEGY
            else:
                motor2.setGoalVelocity(0, verbose=False)
                motor1.setGoalVelocity(0, verbose=False)
                motor3.setGoalVelocity(gvx, verbose=False)
                motor4.setGoalVelocity(gvy, verbose=False)
        elif self.state == fsm.POSXNEGY:
            # if self.theta >= (2.0 * math.pi) - 1e-6:
            if abs(motor1.getCurrentPosition(verbose=False)) >= self.r or abs(motor4.getCurrentPosition(verbose=False)) >= 0:
                self.switching_motors = True
                motor1.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor4.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
                motor1.setGoalPosition(int(self.r), verbose=True)
                motor4.setGoalPosition(0, verbose=True)
                if motor1.reachedGoalPosition(verbose=False) and motor4.reachedGoalPosition(verbose=False):
                    motor1.setOpMode(OpModes.VELOCITY, verbose=True)
                    motor4.setOpMode(OpModes.VELOCITY, verbose=True)
                    self.switching_motors = False
                    self.state = fsm.POSXPOSY
            else:
                motor3.setGoalVelocity(0, verbose=False)
                motor2.setGoalVelocity(0, verbose=False)
                motor1.setGoalVelocity(gvx, verbose=False)
                motor4.setGoalVelocity(gvy, verbose=False)
        # elif self.state == fsm.NEGXNEGY:
        #     motor2.setGoalVelocity(0, verbose=False)
        #     motor1.setGoalVelocity(0, verbose=False)
        #     motor3.setGoalVelocity(gvx, verbose=False)
        #     motor4.setGoalVelocity(gvy, verbose=False)
        #     if self.theta >= (3.0 * math.pi / 2.0):
        #         # switching_motors = True
        #         motor1.setCurrentPosition(motor3.getCurrentPosition(verbose=False), verbose=True)
        #         self.state = fsm.POSXNEGY
        # elif self.state == fsm.POSXNEGY:
        #     motor3.setGoalVelocity(0, verbose=False)
        #     motor2.setGoalVelocity(0, verbose=False)
        #     motor1.setGoalVelocity(gvx, verbose=False)
        #     motor4.setGoalVelocity(gvy, verbose=False)
        #     if self.theta >= (2.0 * math.pi) - 1e-6:
        #         # switching_motors = True
        #         motor2.setCurrentPosition(motor4.getCurrentPosition(verbose=False), verbose=True)
        #         self.state = fsm.POSXPOSY
        # if self.state == fsm.POSXPOSY:
        #     motor1.setGoalVelocity(gvx, verbose=False)
        #     motor2.setGoalVelocity(gvy, verbose=False)
        #     motor3.setGoalVelocity(0, verbose=False)
        #     motor4.setGoalVelocity(0, verbose=False)
        # elif self.state == fsm.NEGXPOSY:
        #     motor1.setGoalVelocity(0, verbose=False)
        #     motor4.setGoalVelocity(0, verbose=False)
        #     motor3.setGoalVelocity(-gvx, verbose=False)
        #     motor2.setGoalVelocity(gvy, verbose=False)
        # elif self.state == fsm.NEGXNEGY:
        #     motor2.setGoalVelocity(0, verbose=False)
        #     motor1.setGoalVelocity(0, verbose=False)
        #     motor3.setGoalVelocity(-gvx, verbose=False)
        #     motor4.setGoalVelocity(-gvy, verbose=False)
        # elif self.state == fsm.POSXNEGY:
        #     motor3.setGoalVelocity(0, verbose=False)
        #     motor2.setGoalVelocity(0, verbose=False)
        #     motor1.setGoalVelocity(gvx, verbose=False)
        #     motor4.setGoalVelocity(-gvy, verbose=False)
        
        # if self.state == fsm.POSXPOSY:
        #     motor1.setGoalVelocity(gvx, verbose=False)
        #     motor2.setGoalVelocity(gvy, verbose=False)
        #     motor3.setGoalVelocity(0, verbose=False)
        #     motor4.setGoalVelocity(0, verbose=False)
        # elif self.state == fsm.NEGXPOSY:
        #     motor1.setGoalVelocity(0, verbose=False)
        #     motor4.setGoalVelocity(0, verbose=False)
        #     motor3.setGoalVelocity(-gvx, verbose=False)
        #     motor2.setGoalVelocity(gvy, verbose=False)
        # elif self.state == fsm.NEGXNEGY:
        #     motor2.setGoalVelocity(0, verbose=False)
        #     motor1.setGoalVelocity(0, verbose=False)
        #     motor3.setGoalVelocity(-gvx, verbose=False)
        #     motor4.setGoalVelocity(-gvy, verbose=False)
        # elif self.state == fsm.POSXNEGY:
        #     motor3.setGoalVelocity(0, verbose=False)
        #     motor2.setGoalVelocity(0, verbose=False)
        #     motor1.setGoalVelocity(gvx, verbose=False)
        #     motor4.setGoalVelocity(-gvy, verbose=False)

        # print(f"Motor 1 (X): {pos_m1}, dx (v_m1): {gvx}; Motor 2 (Y): {pos_m2}, dy (v_m2): {gvy}")
        # UDP send X,Y for live pos tracking:
        now = time.time()
        if now - self._last_send >= self._send_period:
            self._last_send = now
            # x,y,r,state
            # msg = f"{int(self.x)},{int(self.y)},{int(self.r)},{self.state.name}\n"
            # posm1 = motor1.getCurrentPosition(verbose=False)
            # posm2 = motor2.getCurrentPosition(verbose=False)
            # posm3 = motor3.getCurrentPosition(verbose=False)
            # posm4 = motor4.getCurrentPosition(verbose=False)
            # if self.state == fsm.POSXPOSY:
            #     msg = f"{int(posm1)},{int(posm2)},{int(self.r)},{self.state.name}\n"
            # elif self.state == fsm.NEGXPOSY:
            #     msg = f"{int(posm3)},{int(posm2)},{int(self.r)},{self.state.name}\n"
            # elif self.state == fsm.NEGXNEGY:
            #     msg = f"{int(posm3)},{int(posm4)},{int(self.r)},{self.state.name}\n"
            # elif self.state == fsm.POSXNEGY:
            #     msg = f"{int(posm1)},{int(posm4)},{int(self.r)},{self.state.name}\n"
            msg = f"{int(pos_m1)},{int(pos_m2)},{int(self.r)},{self.state.name}\n"
            try:    
                self._udp.sendto(msg.encode(), self._viewer_addr)
            except OSError:
                pass
        print(f"X = {int(self.x)}, Y = {int(self.y)}, theta = {self.theta * 180 / math.pi:.2f} deg, state = {self.state.name}")

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
    routine = test9()
    routine.run()