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
# TICKS_PER_REV = 607_500.0  # H42P: 607,500 ticks per revolution (matches your deg() scale)
# GV_MIN, GV_MAX = -2920, 2920  # Goal Velocity units: 0.01 rev/min per unit


# # def clamp(v, lo, hi):
# #     return lo if v < lo else hi if v > hi else v


# def ticks_s_to_gv_units(v_ticks_s: float) -> float:
#     """
#     Convert ticks/s -> GoalVelocity units (0.01 rev/min).
#     """
#     rpm = (v_ticks_s / TICKS_PER_REV) * 60.0
#     return rpm * 100.0  # 0.01 rpm units



class fsm(Enum):
    POSXPOSY = 1
    NEGXPOSY = 2
    NEGXNEGY = 3
    POSXNEGY = 4

from Misc.PortPacketManager import PortPacketManager as ppm

port = ppm.getPortHandler()
packet = ppm.getPacketHandler()

motor1 = DCMotor(0, port = port, packet = packet, MODEL="H42P")
motor2 = DCMotor(1, port = port, packet = packet, MODEL="H42P")
motor3 = DCMotor(2, port = port, packet = packet, MODEL="L42")
# motor4 = DCMotor(3, port = port, packet = packet, MODEL="L42")
motor4 = DCMotor(3, port = port, packet = packet, MODEL="M42")
motors = MotorGroup(motor1, motor2, motor3, motor4)

def ticks_s_to_gv_units(v_ticks_s: float, model: str) -> float:
    """
    Convert ticks/s -> GoalVelocity units (0.01 rev/min).
    """
    ticks_per_rev = 0
    rev_min = 0
    if model == "H42P":
        # ticks_per_rev = 607_500.0
        ticks_per_rev = 360 / motor1.cfg["pos_deg_per_tick"]
        rev_min = motor1.cfg["vel_rpm_per_unit"]
    elif model == "L42":
        ticks_per_rev = 360 / motor3.cfg["pos_deg_per_tick"]
        rev_min = motor3.cfg["vel_rpm_per_unit"]
    elif model == "M42":
        ticks_per_rev = 360 / motor4.cfg["pos_deg_per_tick"]
        rev_min = motor4.cfg["vel_rpm_per_unit"]
    else:
        raise ValueError(f"Unknown model: {model}")

    rpm = (v_ticks_s / ticks_per_rev) * 60.0
    print("DEBUG: ticks_s_to_gv_units:", v_ticks_s, "ticks/s ->", rpm, "rpm for model", model, "output in GV units:", rpm / rev_min, "ticks_per_rev:", ticks_per_rev, "rev_min:", rev_min)
    return rpm / rev_min  


def ff_gv_for_axis(v_ticks_s: float, active: bool, model: str) -> float:
    return ticks_s_to_gv_units(v_ticks_s, model) if active else 0.0


H42P_DEG_PER_TICK = motor1.cfg["pos_deg_per_tick"]

def h42_ticks_to_deg(ticks: float) -> float:
    return float(ticks) * H42P_DEG_PER_TICK

def deg_s_to_rpm(deg_s: float) -> float:
    return (float(deg_s) / 360.0) * 60.0

def rpm_to_gv_units(rpm: float, motor: DCMotor) -> float:
    return float(rpm) / float(motor.cfg["vel_rpm_per_unit"])  # raw GoalVelocity units


from Logic.Telemetry import Sender, Sniffer

import os

avg_iter_time = 0.0
iter_count = 0


class test935(Routine):
    def __init__(self, csv_sender: Sender =None, csv_logger: Sniffer =None, viz_sender: Sender=None, r0: int = None, k: int = None, period: int = None):
        super().__init__()
        self.csv_sender = csv_sender
        self.csv_logger = csv_logger
        self.viz_sender = viz_sender

        motors.enableTorque()
        # motors.setReverseMode(False)

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

        self.PID_m1 = PIDController()
        self.PID_m2 = PIDController()
        self.PID_m3 = PIDController()
        self.PID_m4 = PIDController()

        kp_h42 = 0.0#15  # rough starting point for P gain; may need to be tuned based on the actual system response
        kd_h42 = 0.000000 # not necessary for t = 5.
        kp_l42 = 0.0#15  # rough starting point for P gain; may need to be tuned based on the actual system response
        kd_l42 = 0.000000 # not necessary for t = 5.
        kp_m42 = 0.0#15  # rough starting point for P gain; may need to be tuned based on the actual system response
        kd_m42 = 0.000000 # not necessary for t = 5.

        # kp units: (0.01 rpm units) / tick
        # kd units: (0.01 rpm units) * s / tick
    
        for pid in (self.PID_m1, self.PID_m2, self.PID_m3, self.PID_m4):
            # pid.setOutputLimit()       # correction limit; keep < MAX (2920) so FF still matters
            pid.setIntegralLimit(0.0)     # effectively disables integral accumulation

        self.PID_m1.setPID(kp_h42, 0.0, kd_h42)
        self.PID_m2.setPID(kp_h42, 0.0, kd_h42)
        self.PID_m3.setPID(kp_l42, 0.0, kd_l42)
        self.PID_m4.setPID(kp_m42, 0.0, kd_m42)

        self.PID_m1.setOutputLimit(motor1.cfg["vel_raw_max"])
        self.PID_m2.setOutputLimit(motor2.cfg["vel_raw_max"])
        self.PID_m3.setOutputLimit(motor3.cfg["vel_raw_max"])
        self.PID_m4.setOutputLimit(motor4.cfg["vel_raw_max"])

        print(f"Radius = {self.r} ticks")

        motors.setHomingOffset(0)
        motors.forceZero()

        # Wait for start pose and zeroing
        while True:
            # if (
            #     motors.
            #     # motor1.reachedGoalPosition(verbose=False, tolerance=1)
            #     # and motor2.reachedGoalPosition(verbose=False, tolerance=1)
            #     # and motor3.reachedGoalPosition(verbose=False, tolerance=1)
            #     # and motor4.reachedGoalPosition(verbose=False, tolerance=1)
            # ):
            if all(motors.reachedGoalPosition(verbose=True, tolerance=20).values()):
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
        

        # radius changing w.r.t omega (LATEST VER.)
        vx_des = -self.r * math.sin(self.theta) * omega + omega * math.cos(self.theta) * self.k         
        vy_des = self.r * math.cos(self.theta) * omega + omega * math.sin(self.theta) * self.k 
        
        x_deg = h42_ticks_to_deg(x_des)
        y_deg = h42_ticks_to_deg(y_des)

        vx_deg_s = h42_ticks_to_deg(vx_des)   # because vx_des is ticks/s in H42P ticks
        vy_deg_s = h42_ticks_to_deg(vy_des)

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
        # x3_des = min(x_des, 0.0)  # motor3 (-X)
        # y2_des = max(y_des, 0.0)  # motor2 (+Y)
        # y4_des = min(y_des, 0.0)  # motor4 (-Y)

        # # Which motors are "active" for FF this cycle?
        # x_pos_active = (x_des > 0.0)
        # x_neg_active = (x_des < 0.0)
        # y_pos_active = (y_des > 0.0)
        # y_neg_active = (y_des < 0.0)
        x1_des = max(x_deg, 0.0)     # +X motor1 (deg)
        x3_des = max(-x_deg, 0.0)    # -X motor3 (deg magnitude)
        y2_des = max(y_deg, 0.0)     # +Y motor2 (deg)
        y4_des = max(-y_deg, 0.0)    # -Y motor4 (deg magnitude)

        x_pos_active = (x_deg > 0.0)
        x_neg_active = (x_deg < 0.0)
        y_pos_active = (y_deg > 0.0)
        y_neg_active = (y_deg < 0.0)

        # print("DEBUG ACtive:", x_pos_active, x_neg_active, y_pos_active, y_neg_active)
        # gv1_ff = ff_gv_for_axis(vx_des, x_pos_active, "H42P")
        # gv3_ff = ff_gv_for_axis(vx_des, x_neg_active, "L42")
        # gv2_ff = ff_gv_for_axis(vy_des, y_pos_active, "H42P")
        # gv4_ff = ff_gv_for_axis(vy_des, y_neg_active, "M42")
        vx_rpm = deg_s_to_rpm(vx_deg_s)
        vy_rpm = deg_s_to_rpm(vy_deg_s)

        v1_rpm_ff = vx_rpm if x_pos_active else 0.0
        v3_rpm_ff = (-vx_rpm) if x_neg_active else 0.0
        v2_rpm_ff = vy_rpm if y_pos_active else 0.0
        v4_rpm_ff = (-vy_rpm) if y_neg_active else 0.0

        gv1_ff = rpm_to_gv_units(v1_rpm_ff, motor1)
        gv3_ff = rpm_to_gv_units(v3_rpm_ff, motor3)
        gv2_ff = rpm_to_gv_units(v2_rpm_ff, motor2)
        gv4_ff = rpm_to_gv_units(v4_rpm_ff, motor4)

        # -----------------------------
        # Read positions (ticks)
        # -----------------------------
        # poss = motors.sync_read_current_position()
        poss = motors.bulk_read_state(fast=False)
        # print("DEBUG: Read positions:", {motor1.DXL_ID: poss[motor1.DXL_ID]["pos"], motor2.DXL_ID: poss[motor2.DXL_ID]["pos"], motor3.DXL_ID: poss[motor3.DXL_ID]["pos"], motor4.DXL_ID: poss[motor4.DXL_ID]["pos"]})
        p1 = poss[motor1.DXL_ID]["pos"]
        p2 = poss[motor2.DXL_ID]["pos"]
        p3 = poss[motor3.DXL_ID]["pos"]
        p4 = poss[motor4.DXL_ID]["pos"]
        p1_deg = p1 * motor1.cfg["pos_deg_per_tick"]
        p2_deg = p2 * motor2.cfg["pos_deg_per_tick"]
        p3_deg = p3 * motor3.cfg["pos_deg_per_tick"]
        p4_deg = p4 * motor4.cfg["pos_deg_per_tick"]
        # -----------------------------
        # PID corrections (in GoalVelocity units)
        # PID.calculate(reference, target, dt)
        # reference = current position (ticks), target = desired position (ticks)
        # output = correction velocity in GV units (0.01 rpm)
        # -----------------------------
        # c1 = self.PID_m1.calculate(p1, x1_des, dt_s)
        # c3 = self.PID_m3.calculate(p3, x3_des, dt_s)
        # c2 = self.PID_m2.calculate(p2, y2_des, dt_s)
        # c4 = self.PID_m4.calculate(p4, y4_des, dt_s)
        c1 = self.PID_m1.calculate(p1_deg, x1_des, dt_s)
        c3 = self.PID_m3.calculate(p3_deg, x3_des, dt_s)
        c2 = self.PID_m2.calculate(p2_deg, y2_des, dt_s)
        c4 = self.PID_m4.calculate(p4_deg, y4_des, dt_s)

        gv1 = clamp(int(round(gv1_ff + c1)), motor1.cfg["vel_raw_min"], motor1.cfg["vel_raw_max"])
        # print("DEBUG: Clamping GV1:", gv1_ff, c1, gv1, "between", motor1.cfg["vel_raw_min"], "and", motor1.cfg["vel_raw_max"])
        gv3 = clamp(int(round(gv3_ff + c3)), motor3.cfg["vel_raw_min"], motor3.cfg["vel_raw_max"])
        gv2 = clamp(int(round(gv2_ff + c2)), motor2.cfg["vel_raw_min"], motor2.cfg["vel_raw_max"])
        gv4 = clamp(int(round(gv4_ff + c4)), motor4.cfg["vel_raw_min"], motor4.cfg["vel_raw_max"])

        # -----------------------------
        # Send velocities (ALL motors every loop)
        # This is what prevents drift + removes "handoff jumps".
        # -----------------------------
        gvs = {motor1.DXL_ID: gv1, motor2.DXL_ID: gv2, motor3.DXL_ID: gv3, motor4.DXL_ID: gv4}
        # motors.sync_write_goal_velocity(gvs)
        # print("DEBUG: Writing GVs:", gvs)
        motors.bulk_write_goal_velocity(gvs)
        
        # Continuous measured XY for viewer (no switching)
        # x_meas = p1 + p3
        # y_meas = p2 + p4
        x_meas = p1_deg - p3_deg
        y_meas = p2_deg - p4_deg

        if self.csv_sender is not None:
            # self.csv_sender.add(int(x_meas), int(y_meas), self.theta*180/math.pi, getTime())
            self.csv_sender.add(int(x_meas), int(y_meas), int(self.r), self.state.name, p1, p2, p3, p4, gv1, gv2, gv3, gv4)
            self.csv_sender.update()
            if self.csv_logger is not None:
                self.csv_logger.set_csv_header("X_ticks,Y_ticks,Theta_deg,Time_s, Motor1_pos, Motor2_pos, Motor3_pos, Motor4_pos, Motor1_gv, Motor2_gv, Motor3_gv, Motor4_gv")
                self.csv_logger.log_CSV(self.csv_sender.getName())
        if self.viz_sender is not None:
            # self.viz_sender.add(int(x_meas), int(y_meas), int(self.r), self.state.name)
            self.viz_sender.add(int(x_meas), int(y_meas), int(h42_ticks_to_deg(self.r)), self.state.name)
            self.viz_sender.update()
        # CSV_Logger.visualize_xy()
        # Light debug (comment out if timing sensitive)
        print(f"theta={self.theta*180/math.pi:6.2f}  state={self.state.name}  gv=[{gv1},{gv3},{gv2},{gv4}] (X,Y) = ({x_meas}, {y_meas})")

    def run(self):
        try:
            while True:
                self.loop()
        # except KeyboardInterrupt:
        except Exception as e:
            print(f"(failed to run routine loop: {e})")
            print(f"Average iteration time over {iter_count} iterations: {avg_iter_time/iter_count:.6f} s")
            print("Routine stopped by user.")
            self.onStop()

    def onStop(self):
        motors.stop()


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

    routine = test935(csv_sender=csv_sender, csv_logger=csv_logger, viz_sender=viz_sender, r0 = args.r0, k = args.k, period = args.period)
    routine.run()

if __name__ == "__main__":
    main()

