# Jan 21

from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Mechas.MotorGroupOLD import MotorGroup
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


class test91(Routine):
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
        self._send_period = 1.0 / 100.0   # 30 Hz to the viewer (plenty smooth)
        self.last_time_s = time.time() # placeholder for time
        motor2.forceZero()
        motor3.forceZero()
        motor4.forceZero()
        motor1.setGoalPosition(int(self.x))

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
                    motors.setOpmode(OpModes.VELOCITY)
                    print("Continuing...")
                    break
            time.sleep(0.01)
        
    def loop(self):
                # constants
        TICKS_PER_REV = 607_500.0

        def ticks_s_to_gv_units(v_ticks_s: float) -> int:
            rpm = (v_ticks_s / TICKS_PER_REV) * 60.0
            return int(round(rpm * 100.0))  # 0.01 rpm units

        def vel_cmd_units(v_ff_ticks_s: float, pos_err_ticks: float, Kp_v_units_per_tick: float) -> int:
            gv = ticks_s_to_gv_units(v_ff_ticks_s) + int(round(Kp_v_units_per_tick * pos_err_ticks))
            return clamp(gv, -1500, 1500)

        now_s = time.time()
        dt_s = now_s - self.last_time_s
        self.last_time_s = now_s
        if dt_s <= 0 or dt_s > 0.4:
            dt_s = 0.01

        # trajectory timing
        t = 4
        omega = (math.pi / 2.0) / t

        # advance theta (full circle)
        # self.theta = (self.theta + omega * dt_s) % (2.0 * math.pi)
        self.theta = self.theta % (2.0 * math.pi)
        self.theta += omega * dt_s

        # desired pos (ticks)
        x_des = self.r * math.cos(self.theta)
        y_des = self.r * math.sin(self.theta)

        # desired vel (ticks/s)
        vx_des = -self.r * math.sin(self.theta) * omega
        vy_des =  self.r * math.cos(self.theta) * omega

        # split into + / - motors (piecewise)
        x1_des = max(x_des, 0.0)   # motor1 (+X)
        x3_des = min(x_des, 0.0)   # motor3 (-X, negative)
        y2_des = max(y_des, 0.0)   # motor2 (+Y)
        y4_des = min(y_des, 0.0)   # motor4 (-Y, negative)

        kp_v = 0.01  # (0.01 rpm units) per tick
        # derivatives of the split setpoints
        vx1_ff = vx_des if x_des > 0.0 else 0.0
        # Kp_v_m1 = kp_v if x_des <= 0.0 else 0.0
        vx3_ff = vx_des if x_des < 0.0 else 0.0
        # Kp_v_m3 = kp_v if x_des >= 0.0 else 0.0
        vy2_ff = vy_des if y_des > 0.0 else 0.0
        # Kp_v_m2 = kp_v if y_des <= 0.0 else 0.0
        vy4_ff = vy_des if y_des < 0.0 else 0.0
        # Kp_v_m4 = kp_v if y_des >= 0.0 else 0.0
        Kp_v_m1 = Kp_v_m2 = Kp_v_m3 = Kp_v_m4 = kp_v
        # read actual positions (ticks)
        p1 = motor1.getCurrentPosition(verbose=False)
        p2 = motor2.getCurrentPosition(verbose=False)
        p3 = motor3.getCurrentPosition(verbose=False)
        p4 = motor4.getCurrentPosition(verbose=False)

        # position errors (ticks)
        e1 = x1_des - p1
        e3 = x3_des - p3
        e2 = y2_des - p2
        e4 = y4_des - p4

        # tune this (start tiny; increase until inactive motors hold 0 tightly without oscillation)
        # Kp_v_m1 = 0.005 # (0.01 rpm units) per tick for motor1
        # Kp_v_m2 = 0.005 # (0.01 rpm units) per tick for motor2
        # Kp_v_m3 = 0.005 # (0.01 rpm units) per tick for motor3
        # Kp_v_m4 = 0.005 # (0.01 rpm units) per tick for motor4

        gv1 = vel_cmd_units(vx1_ff, e1, Kp_v_m1)
        gv3 = vel_cmd_units(vx3_ff, e3, Kp_v_m3)
        gv2 = vel_cmd_units(vy2_ff, e2, Kp_v_m2)
        gv4 = vel_cmd_units(vy4_ff, e4, Kp_v_m4)

        # send velocities (ALL motors every loop)
        motor1.setGoalVelocity(gv1, verbose=False)
        motor2.setGoalVelocity(gv2, verbose=False)
        motor3.setGoalVelocity(gv3, verbose=False)
        motor4.setGoalVelocity(gv4, verbose=False)

        # for viewer: continuous physical XY (no motor-pair switching)
        x_meas = p1 + p3   # p3 is negative when active
        y_meas = p2 + p4   # p4 is negative when active

        print(f"Motor 1: {p1}, Motor 2: {p2}, Motor 3: {p3}, Motor 4: {p4}")
    
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
            msg = f"{int(x_meas)},{int(y_meas)},{int(self.r)},{self.state.name}\n"
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
    routine = test91()
    routine.run()