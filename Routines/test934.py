# Jan 23, testing w/ only 2 motors for now, instead of 4 (full X and Y, no separate -X, -Y)

from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
# from Mechas.MotorGroupOLD import MotorGroup
from Mechas.MotorGroup import MotorGroup
from Misc.OpModes import OpModes
from Misc.deg import deg
import time, math
from enum import Enum
from Logic.PIDController import PIDController
import socket
from Misc.Helpers import clamp, getTime

# -----------------------------
# Helpers / constants
# -----------------------------
TICKS_PER_REV = 607_500.0  # H42P: 607,500 ticks per revolution (matches your deg() scale)
GV_MIN, GV_MAX = -2920, 2920  # Goal Velocity units: 0.01 rev/min per unit


# def clamp(v, lo, hi):
#     return lo if v < lo else hi if v > hi else v


def ticks_s_to_gv_units(v_ticks_s: float) -> float:
    """
    Convert ticks/s -> GoalVelocity units (0.01 rev/min).
    """
    rpm = (v_ticks_s / TICKS_PER_REV) * 60.0
    return rpm * 100.0  # 0.01 rpm units


def ff_gv_for_axis(v_ticks_s: float, active: bool) -> float:
    return ticks_s_to_gv_units(v_ticks_s) if active else 0.0


class fsm(Enum):
    POSXPOSY = 1
    NEGXPOSY = 2
    NEGXNEGY = 3
    POSXNEGY = 4


# motor1 = DCMotor(1)  # +X
# motor2 = DCMotor(2)  # +Y
from Misc.PortPacketManager import PortPacketManager as ppm

port = ppm.getPortHandler()
packet = ppm.getPacketHandler()

motor1 = DCMotor(0, port = port, packet = packet)
motor2 = DCMotor(1, port = port, packet = packet)
motors = MotorGroup(motor1, motor2)
# motor3 = DCMotor(3)  # -X
# motor4 = DCMotor(4)  # -Y
# motors = MotorGroup(motor1, motor2, motor3, motor4)
# motors = MotorGroup(motor1, motor2)

from Logic.Telemetry import Sender, Sniffer

# SimpleTelemetry = Sender(port = 9999)
import os

avg_iter_time = 0.0
iter_count = 0
# CSV_Sender = Sender(name = os.path.basename(__file__).strip(".py"), port = 64848)
# CSV_Logger = Sniffer(port = CSV_Sender.getPort())
# Viz_Sender = Sender(port = 9999)
# Viz_Logger = Sniffer(port = Viz_Sender.getPort())

class test934(Routine):
    def __init__(self, csv_sender: Sender =None, csv_logger: Sniffer =None, viz_sender: Sender=None, r0: int = None, k: int = None, period: int = None):
        super().__init__()
        self.csv_sender = csv_sender
        self.csv_logger = csv_logger
        self.viz_sender = viz_sender

        motors.enableTorque()
        motors.setReverseMode(False)

        # Start in EXTENDED_POSITION for "setup / home-ish" placement,
        # then switch to VELOCITY and stay there forever.
        motors.setOpmode(OpModes.EXTENDED_POSITION)

        self.r = 1e-6  # radius (ticks)
        self.r0 = deg(270) if r0 is None else r0 # max radius (ticks)
        self.k = deg(10) if k is None else k # spiral radius scaling factor (arbitrary choice) because ticks are very small by themselves
        # Trajectory param
        self.theta = 0.0
        self.period = 20 if period is None else period # time for full rev (4 x t_quarter)
        self.last_time_s = time.time()

        # State just for logging/visualization
        self.state = fsm.POSXPOSY

        # -----------------------------
        # PIDs (use YOUR PID class)
        # One PID per motor so we don't mix error histories across sign switches.
        # We’ll keep Ki=0 unless you decide otherwise.
        # Output is in GoalVelocity units (0.01 rpm).
        # Error input is in ticks.
        # -----------------------------
        self.PID_m1 = PIDController()
        self.PID_m2 = PIDController()
        self.PID_m3 = PIDController()
        self.PID_m4 = PIDController()

        # Start with P only, add a *small* D later.
        # kp units: (0.01 rpm units) / tick
        # kd units: (0.01 rpm units) * s / tick
        # kp = 0.015 * self.r0 / self.r  # 0.015 for t = 5.0, r0 = deg(180); otherwise re-tune...
        kp = 0.015
        kd = 0.000000 # not necessary for t = 5.0

        for pid in (self.PID_m1, self.PID_m2, self.PID_m3, self.PID_m4):
            pid.setPID(kp, 0.0, kd)
            pid.setOutputLimit(1500/2920 * motor1.getProfileVelocity())       # correction limit; keep < MAX (2920) so FF still matters
            pid.setIntegralLimit(0.0)     # effectively disables integral accumulation

        # UDP setup for viewer_xy.py
        # self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self._viewer_addr = ("127.0.0.1", 9999)
        # self._last_send = 0.0
        # self._send_period = 1.0 / 100.0 # 100 Hz to the viewer (plenty smooth)
        # telemetry = Telemetry("127.0.0.1", 9999, 100)
        print(f"Radius = {self.r} ticks")

        # motor1.setHomingOffset(0)
        # motor2.setHomingOffset(0)
        motors.setHomingOffset(0)
        # Move to start pose: (x=+r, y=0)
        # motor2.forceZero()
        # motor3.forceZero()
        # motor4.forceZero()
        # motor1.setGoalPosition(int(self.r))
        motors.forceZero()

        # Optional profile settings (won’t hurt; in Velocity mode accel profile is commonly used)
        motors.setProfileVelocity(1500)
        motors.setProfileAcceleration(10765)

        # Wait for start pose and zeroing
        while True:
            # if (
            #     motors.
            #     # motor1.reachedGoalPosition(verbose=False, tolerance=1)
            #     # and motor2.reachedGoalPosition(verbose=False, tolerance=1)
            #     # and motor3.reachedGoalPosition(verbose=False, tolerance=1)
            #     # and motor4.reachedGoalPosition(verbose=False, tolerance=1)
            # ):
            if all(motors.reachedGoalPosition(verbose=False, tolerance=1).values()):
                print("All motors ready at start pose.")
                print("Type C to continue...")
                if input().lower() == "c":
                    self.last_time_s = time.time()
                    motors.setOpmode(OpModes.VELOCITY)  # IMPORTANT: stay in VELOCITY forever after this
                    print("Continuing in VELOCITY mode...")
                    break
            # time.sleep(0.01)

    def loop(self):
        global avg_iter_time, iter_count
        # -----------------------------
        # Timing
        # -----------------------------
        now_s = time.time()
        dt_s = now_s - self.last_time_s
        self.last_time_s = now_s
        if dt_s <= 0.0 or dt_s > 0.45:
            dt_s = 0.01
        avg_iter_time += dt_s
        iter_count += 1
        # -----------------------------
        # Trajectory: full circle (smooth)
        # -----------------------------
        # t_quarter = 5.0 * self.r / self.r0  # seconds per quarter circle scaled by radius
        t_quarter = 5.0 if self.period is None else self.period / 4.0 # seconds per quarter circle
        omega = (math.pi / 2.0) / t_quarter  # rad/s

        self.theta += omega * dt_s
        self.r = self.k * self.theta
        
        self.r = min(self.r, self.r0)  # max radius
        x_des = self.r * math.cos(self.theta)
        y_des = self.r * math.sin(self.theta)

        vx_des = -self.r * math.sin(self.theta) * omega 
        + omega * math.cos(self.theta) * self.k 
        # <- radius changing w.r.t omega (CURRENT)
        vy_des = self.r * math.cos(self.theta) * omega 
        + omega * math.sin(self.theta) * self.k 
        # <- radius changing w.r.t omega (CURRENT)
        
        # Determine quadrant state (for debugging only)
        if x_des >= 0 and y_des >= 0:
            self.state = fsm.POSXPOSY
        elif x_des < 0 and y_des >= 0:
            self.state = fsm.NEGXPOSY
        elif x_des < 0 and y_des < 0:
            self.state = fsm.NEGXNEGY
        else:
            self.state = fsm.POSXNEGY

        # -----------------------------
        # Split desired position into motor targets
        # -----------------------------
        # x1_des = max(x_des, 0.0)  # motor1 (+X)
        x1_des = x_des  # motor1 (+X)
        y2_des = y_des  # motor2 (+Y)
        # x3_des = min(x_des, 0.0)  # motor3 (-X)
        # y2_des = max(y_des, 0.0)  # motor2 (+Y)
        # y4_des = min(y_des, 0.0)  # motor4 (-Y)

        # Which motors are "active" for FF this cycle?
        # x_pos_active = (x_des > 0.0)
        # x_neg_active = (x_des < 0.0)
        # y_pos_active = (y_des > 0.0)
        # y_neg_active = (y_des < 0.0)
        x_active = True
        y_active = True

        # gv1_ff = ff_gv_for_axis(vx_des, x_pos_active)
        # gv3_ff = ff_gv_for_axis(vx_des, x_neg_active)
        # gv2_ff = ff_gv_for_axis(vy_des, y_pos_active)
        # gv4_ff = ff_gv_for_axis(vy_des, y_neg_active)
        gv1_ff = ff_gv_for_axis(vx_des, x_active)
        gv2_ff = ff_gv_for_axis(vy_des, y_active)
        # -----------------------------
        # Read positions (ticks)
        # -----------------------------
        # p1 = motor1.getCurrentPosition(verbose=False)
        # p2 = motor2.getCurrentPosition(verbose=False)
        poss = motors.sync_read_current_position()
        p1 = poss[motor1.DXL_ID] 
        p2 = poss[motor2.DXL_ID]
        # p3 = motor3.getCurrentPosition(verbose=False)
        # p4 = motor4.getCurrentPosition(verbose=False)
        p3 = p4 = 0  # dummy values for motor3, motor4

        # -----------------------------
        # PID corrections (in GoalVelocity units)
        # PID.calculate(reference, target, dt)
        # reference = current position (ticks), target = desired position (ticks)
        # output = correction velocity in GV units (0.01 rpm)
        # -----------------------------
        c1 = self.PID_m1.calculate(p1, x1_des, dt_s)
        # c3 = self.PID_m3.calculate(p3, x3_des, dt_s)
        c2 = self.PID_m2.calculate(p2, y2_des, dt_s)
        # c4 = self.PID_m4.calculate(p4, y4_des, dt_s)

        gv1 = clamp(int(round(gv1_ff + c1)), GV_MIN, GV_MAX)
        # gv3 = clamp(int(round(gv3_ff + c3)), GV_MIN, GV_MAX)
        gv2 = clamp(int(round(gv2_ff + c2)), GV_MIN, GV_MAX)
        # gv4 = clamp(int(round(gv4_ff + c4)), GV_MIN, GV_MAX)

        # -----------------------------
        # Send velocities (ALL motors every loop)
        # This is what prevents drift + removes "handoff jumps".
        # -----------------------------
        # motor1.setGoalVelocity(gv1, verbose=False)
        # motor2.setGoalVelocity(gv2, verbose=False)
        gvs = {motor1.DXL_ID: gv1, motor2.DXL_ID: gv2}
        motors.sync_write_goal_velocity(gvs)
        # motor3.setGoalVelocity(gv3, verbose=False)
        # motor4.setGoalVelocity(gv4, verbose=False)
        
        # Continuous measured XY for viewer (no switching)
        x_meas = p1 + p3
        y_meas = p2 + p4

        if self.csv_sender is not None:
            self.csv_sender.add(int(x_meas), int(y_meas), self.theta*180/math.pi, getTime())
            self.csv_sender.update()
            if self.csv_logger is not None:
                self.csv_logger.set_csv_header("X_ticks,Y_ticks,Theta_deg,Time_s")
                self.csv_logger.log_CSV(self.csv_sender.getName())
        # Viz_Sender.add(int(x_meas), int(y_meas), int(self.r), self.state.name)
        # Viz_Sender.update()
        if self.viz_sender is not None:
            self.viz_sender.add(int(x_meas), int(y_meas), int(self.r), self.state.name)
            self.viz_sender.update()
        # CSV_Logger.visualize_xy()
        # Light debug (comment out if timing sensitive)
        print(f"theta={self.theta*180/math.pi:6.2f}  state={self.state.name}  gv=[{gv1},{gv2}] (X,Y) = ({x_meas}, {y_meas})")

    def run(self):
        try:
            while True:
                self.loop()
        except KeyboardInterrupt:
            print(f"Average iteration time over {iter_count} iterations: {avg_iter_time/iter_count:.6f} s")
            print("Routine stopped by user.")
            self.onStop()

    def onStop(self):
        motors.stop()


import argparse

def main():
    import argparse, os
    from Logic.Telemetry import Sender, Sniffer

    ap = argparse.ArgumentParser()
    ap.add_argument("--viz-port", type=int, default=9999)
    ap.add_argument("--csv-port", type=int, default=None)   # None => auto
    ap.add_argument("--no-viz", action="store_true")
    ap.add_argument("--no-csv", action="store_true")
    ap.add_argument("--r0", type=int, default=455626)
    ap.add_argument("--k", type=int, default=16875)
    ap.add_argument("--period", type=int, default=20)
    args = ap.parse_args()

    name = os.path.splitext(os.path.basename(__file__))[0]

    csv_sender = csv_logger = None
    if not args.no_csv:
        csv_sender = Sender(name=name, port=args.csv_port)     # auto if None
        csv_logger = Sniffer(port=csv_sender.getPort())

    viz_sender = None
    if not args.no_viz:
        viz_sender = Sender(name=f"{name}_viz", port=args.viz_port)

    routine = test934(csv_sender=csv_sender, csv_logger=csv_logger, viz_sender=viz_sender, r0 = args.r0, k = args.k, period = args.period)
    routine.run()

if __name__ == "__main__":
    main()


# if __name__ == "__main__":
#     routine = test933()
#     routine.run()
