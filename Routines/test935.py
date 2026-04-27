from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Mechas.MotorGroup import MotorGroup
from Misc.OpModes import OpModes
from Misc.deg import deg
from Misc.Helpers import clamp, getTime
from Logic.PIDController import PIDController
from Logic.Telemetry import Sender, Sniffer
from Misc.PortPacketManager import PortPacketManager as ppm

import time
import math
from enum import Enum


class fsm(Enum):
    POSXPOSY = 1
    NEGXPOSY = 2
    NEGXNEGY = 3
    POSXNEGY = 4


port = ppm.getPortHandler()
packet = ppm.getPacketHandler()

# Axis mapping:
# X+ -> H42P (id 0)
# X- -> H42P (id 1)
# Y- -> L42  (id 2)
# Y+ -> M42  (id 3)

motor_x_pos = DCMotor(0, port=port, packet=packet, MODEL="H42P")
motor_x_neg = DCMotor(1, port=port, packet=packet, MODEL="H42P")
motor_y_neg = DCMotor(2, port=port, packet=packet, MODEL="L42")
motor_y_pos = DCMotor(3, port=port, packet=packet, MODEL="M42")

motors = MotorGroup(motor_x_pos, motor_x_neg, motor_y_neg, motor_y_pos)

H42P_DEG_PER_TICK = motor_x_pos.cfg["pos_deg_per_tick"]

avg_iter_time = 0.0
iter_count = 0


def h42_ticks_to_deg(ticks: float) -> float:
    return float(ticks) * H42P_DEG_PER_TICK


def deg_s_to_rpm(deg_s: float) -> float:
    return (float(deg_s) / 360.0) * 60.0


def rpm_to_gv_units(rpm: float, motor: DCMotor) -> float:
    return float(rpm) / float(motor.cfg["vel_rpm_per_unit"])


class test935(Routine):
    def __init__(
        self,
        csv_sender: Sender = None,
        csv_logger: Sniffer = None,
        viz_sender: Sender = None,
        r0: int = None,
        k: int = None,
        period: int = None,
    ):
        super().__init__()
        self.csv_sender = csv_sender
        self.csv_logger = csv_logger
        self.viz_sender = viz_sender

        motors.enableTorque()
        motors.setOpmode(OpModes.EXTENDED_POSITION)
        # motors.setHomingOffset(0)

        self.r = 1e-6
        # self.r0 = deg(360) if r0 is None else r0 
        self.r0 = deg(570) if r0 is None else r0
        self.k = deg(10) if k is None else k
        self.theta = 0.0
        self.period = 20 if period is None else period
        self.last_time_s = time.time()
        self.state = fsm.POSXPOSY

        self.PID_x_pos = PIDController()
        self.PID_x_neg = PIDController()
        self.PID_y_neg = PIDController()
        self.PID_y_pos = PIDController()

        kp_h42 = 0.0
        kd_h42 = 0.0
        kp_l42 = 0.0
        kd_l42 = 0.0
        kp_m42 = 0.0
        kd_m42 = 0.0

        for pid in (self.PID_x_pos, self.PID_x_neg, self.PID_y_neg, self.PID_y_pos):
            pid.setIntegralLimit(0.0)

        self.PID_x_pos.setPID(kp_h42, 0.0, kd_h42)
        self.PID_x_neg.setPID(kp_h42, 0.0, kd_h42)
        self.PID_y_neg.setPID(kp_l42, 0.0, kd_l42)
        self.PID_y_pos.setPID(kp_m42, 0.0, kd_m42)

        self.PID_x_pos.setOutputLimit(motor_x_pos.cfg["vel_raw_max"])
        self.PID_x_neg.setOutputLimit(motor_x_neg.cfg["vel_raw_max"])
        self.PID_y_neg.setOutputLimit(motor_y_neg.cfg["vel_raw_max"])
        self.PID_y_pos.setOutputLimit(motor_y_pos.cfg["vel_raw_max"])

        print(f"Radius = {self.r} ticks")

        # motor_x_pos.forceZero(verbose=True)
        # motor_x_neg.forceZero(verbose=True)
        # motor_y_neg.forceZero(verbose=True)
        # motor_y_pos.forceZero(verbose=True)

        while True:
            ready = (
                motor_x_pos.reachedGoalPositionDeg(tolerance_deg=1.0, verbose=True)
                and motor_x_neg.reachedGoalPositionDeg(tolerance_deg=1.0, verbose=True)
                and motor_y_neg.reachedGoalPositionDeg(tolerance_deg=1.0, verbose=True)
                and motor_y_pos.reachedGoalPositionDeg(tolerance_deg=1.0, verbose=True)
            )
            if ready:
                print("All motors ready at start pose.")
                print("Type C to continue...")
                if input().lower() == "c":
                    self.last_time_s = time.time()
                    motors.setOpmode(OpModes.VELOCITY)
                    print("Continuing in VELOCITY mode...")
                    break

    def loop(self):
        global avg_iter_time, iter_count

        now_s = time.time()
        dt_s = now_s - self.last_time_s
        self.last_time_s = now_s
        if dt_s <= 0.0 or dt_s > 0.45:
            dt_s = 0.01

        avg_iter_time += dt_s
        iter_count += 1

        t_quarter = 5.0 if self.period is None else self.period / 4.0
        omega = (math.pi / 2.0) / t_quarter

        self.theta += omega * dt_s
        self.r = min(self.k * self.theta, self.r0)

        x_des = self.r * math.cos(self.theta)
        y_des = self.r * math.sin(self.theta)

        vx_des = -self.r * math.sin(self.theta) * omega + omega * math.cos(self.theta) * self.k
        vy_des = self.r * math.cos(self.theta) * omega + omega * math.sin(self.theta) * self.k

        x_deg = h42_ticks_to_deg(x_des)
        y_deg = h42_ticks_to_deg(y_des)
        vx_deg_s = h42_ticks_to_deg(vx_des)
        vy_deg_s = h42_ticks_to_deg(vy_des)

        if x_des >= 0 and y_des >= 0:
            self.state = fsm.POSXPOSY
        elif x_des < 0 and y_des >= 0:
            self.state = fsm.NEGXPOSY
        elif x_des < 0 and y_des < 0:
            self.state = fsm.NEGXNEGY
        else:
            self.state = fsm.POSXNEGY

        x_pos_des = max(x_deg, 0.0)
        x_neg_des = max(-x_deg, 0.0)
        y_neg_des = max(-y_deg, 0.0)
        y_pos_des = max(y_deg, 0.0)

        x_pos_active = x_deg > 0.0
        x_neg_active = x_deg < 0.0
        y_neg_active = y_deg < 0.0
        y_pos_active = y_deg > 0.0

        vx_rpm = deg_s_to_rpm(vx_deg_s)
        vy_rpm = deg_s_to_rpm(vy_deg_s)

        vx_pos_rpm_ff = vx_rpm if x_pos_active else 0.0
        vx_neg_rpm_ff = -vx_rpm if x_neg_active else 0.0
        vy_neg_rpm_ff = -vy_rpm if y_neg_active else 0.0
        vy_pos_rpm_ff = vy_rpm if y_pos_active else 0.0

        gv_x_pos_ff = rpm_to_gv_units(vx_pos_rpm_ff, motor_x_pos)
        gv_x_neg_ff = rpm_to_gv_units(vx_neg_rpm_ff, motor_x_neg)
        gv_y_neg_ff = rpm_to_gv_units(vy_neg_rpm_ff, motor_y_neg)
        gv_y_pos_ff = rpm_to_gv_units(vy_pos_rpm_ff, motor_y_pos)

        poss = motors.bulk_read_present_position(fast=False)

        p_x_pos = poss[motor_x_pos.DXL_ID]
        p_x_neg = poss[motor_x_neg.DXL_ID]
        p_y_neg = poss[motor_y_neg.DXL_ID]
        p_y_pos = poss[motor_y_pos.DXL_ID]

        p_x_pos_deg = p_x_pos * motor_x_pos.cfg["pos_deg_per_tick"]
        p_x_neg_deg = p_x_neg * motor_x_neg.cfg["pos_deg_per_tick"]
        p_y_neg_deg = p_y_neg * motor_y_neg.cfg["pos_deg_per_tick"]
        p_y_pos_deg = p_y_pos * motor_y_pos.cfg["pos_deg_per_tick"]

        c_x_pos = self.PID_x_pos.calculate(p_x_pos_deg, x_pos_des, dt_s)
        c_x_neg = self.PID_x_neg.calculate(p_x_neg_deg, x_neg_des, dt_s)
        c_y_neg = self.PID_y_neg.calculate(p_y_neg_deg, y_neg_des, dt_s)
        c_y_pos = self.PID_y_pos.calculate(p_y_pos_deg, y_pos_des, dt_s)

        gv_x_pos = clamp(
            int(round(gv_x_pos_ff + c_x_pos)),
            motor_x_pos.cfg["vel_raw_min"],
            motor_x_pos.cfg["vel_raw_max"],
        )
        gv_x_neg = clamp(
            int(round(gv_x_neg_ff + c_x_neg)),
            motor_x_neg.cfg["vel_raw_min"],
            motor_x_neg.cfg["vel_raw_max"],
        )
        gv_y_neg = clamp(
            int(round(gv_y_neg_ff + c_y_neg)),
            motor_y_neg.cfg["vel_raw_min"],
            motor_y_neg.cfg["vel_raw_max"],
        )
        gv_y_pos = clamp(
            int(round(gv_y_pos_ff + c_y_pos)),
            motor_y_pos.cfg["vel_raw_min"],
            motor_y_pos.cfg["vel_raw_max"],
        )

        gvs = {
            motor_x_pos.DXL_ID: gv_x_pos,
            motor_x_neg.DXL_ID: gv_x_neg,
            motor_y_neg.DXL_ID: gv_y_neg,
            motor_y_pos.DXL_ID: gv_y_pos,
        }
        motors.bulk_write_goal_velocity(gvs)

        x_meas = p_x_pos_deg - p_x_neg_deg
        y_meas = p_y_pos_deg - p_y_neg_deg

        if self.csv_sender is not None:
            self.csv_sender.add(
                x_meas,
                y_meas,
                self.theta * 180.0 / math.pi,
                self.state.name,
                p_x_pos,
                p_x_neg,
                p_y_neg,
                p_y_pos,
                gv_x_pos,
                gv_x_neg,
                gv_y_neg,
                gv_y_pos,
                getTime(),
            )
            self.csv_sender.update()
            if self.csv_logger is not None:
                self.csv_logger.set_csv_header(
                    "X_deg,Y_deg,Theta_deg,State,"
                    "Xp_pos_raw,Xn_pos_raw,Yn_pos_raw,Yp_pos_raw,"
                    "Xp_gv,Xn_gv,Yn_gv,Yp_gv,Time_s"
                )
                self.csv_logger.log_CSV(self.csv_sender.getName())

        if self.viz_sender is not None:
            self.viz_sender.add(x_meas, y_meas, h42_ticks_to_deg(self.r), self.state.name)
            self.viz_sender.update()

        print(
            f"theta={self.theta * 180.0 / math.pi:6.2f}  "
            f"state={self.state.name}  "
            f"gv=[Xp:{gv_x_pos}, Xn:{gv_x_neg}, Yn:{gv_y_neg}, Yp:{gv_y_pos}]  "
            f"(X,Y)=({x_meas:.3f}, {y_meas:.3f})"
        )

        time.sleep(0.01)

    def run(self):
        try:
            while True:
                self.loop()
        except Exception as e:
            print(f"(failed to run routine loop: {e})")
            if iter_count > 0:
                print(f"Average iteration time over {iter_count} iterations: {avg_iter_time / iter_count:.6f} s")
            print("Routine stopped by user.")
            self.onStop()

    def onStop(self):
        motors.stop()


def main():
    import argparse
    import os

    ap = argparse.ArgumentParser()
    ap.add_argument("--viz-port", type=int, default=9999)
    ap.add_argument("--csv-port", type=int, default=None)
    ap.add_argument("--no-viz", action="store_true")
    ap.add_argument("--no-csv", action="store_true")
    ap.add_argument("--r0", type=int, default=638591) # deg(570)
    ap.add_argument("--k", type=int, default=16875)
    ap.add_argument("--period", type=int, default=20)
    args = ap.parse_args()

    name = os.path.splitext(os.path.basename(__file__))[0]

    csv_sender = csv_logger = None
    if not args.no_csv:
        csv_sender = Sender(name=name, port=args.csv_port)
        csv_logger = Sniffer(port=csv_sender.getPort())

    viz_sender = None
    if not args.no_viz:
        viz_sender = Sender(name=f"{name}_viz", port=args.viz_port)

    routine = test935(
        csv_sender=csv_sender,
        csv_logger=csv_logger,
        viz_sender=viz_sender,
        r0=args.r0,
        k=args.k,
        period=args.period,
    )
    routine.run()


if __name__ == "__main__":
    main()