# Jan 23 (updated): 2-motor lemniscate (Bernoulli) using a smooth parametric form
# Fixes:
#   1) Use continuous parametric lemniscate (no sqrt(cos 2θ) singularity, no half-curve gaps)
#   2) Never "return" mid-loop (always command velocities every cycle)
#   3) Turn on a small amount of PID feedback (kp != 0) so the crossing actually closes

from Routines.Routine import Routine
from Mechas.DCMotor import DCMotor
from Trash.MotorGroupOLD import MotorGroup
from Misc.OpModes import OpModes
from Misc.deg import deg
import time, math
from enum import Enum
from Logic.PIDController import PIDController
import socket


# -----------------------------
# Helpers / constants
# -----------------------------
TICKS_PER_REV = 607_500.0           # H42P: ticks per revolution (matches your deg() scale)
GV_MIN, GV_MAX = -2920, 2920          # GoalVelocity units: 0.01 rev/min per unit


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def ticks_s_to_gv_units(v_ticks_s: float) -> float:
    """
    Convert ticks/s -> GoalVelocity units (0.01 rev/min).
    """
    rpm = (v_ticks_s / TICKS_PER_REV) * 60.0
    return rpm * 100.0  # 0.01 rpm units


def ff_gv_for_axis(v_ticks_s: float, active: bool = True) -> float:
    return ticks_s_to_gv_units(v_ticks_s) if active else 0.0


class fsm(Enum):
    POSXPOSY = 1
    NEGXPOSY = 2
    NEGXNEGY = 3
    POSXNEGY = 4


# Two motors only (X and Y)
motor1 = DCMotor(0)  # X axis
motor2 = DCMotor(1)  # Y axis
motors = MotorGroup(motor1, motor2)


class test932L(Routine):
    def __init__(self):
        super().__init__()

        motors.enableTorque()
        motors.setReverseMode(False)

        # Start in EXTENDED_POSITION for setup,
        # then switch to VELOCITY and stay there.
        motors.setOpmode(OpModes.EXTENDED_POSITION)

        # Lemniscate amplitude (ticks)
        # At t=0: x=a, y=0 (nice start pose)
        self.a = deg(180)

        # Trajectory parameter (NOT theta in polar form anymore)
        self.t = 0.0
        self.last_time_s = time.time()

        # Logging/visualization state
        self.state = fsm.POSXPOSY

        # -----------------------------
        # PIDs (one per motor)
        # Output is in GoalVelocity units (0.01 rpm).
        # Error input is in ticks.
        # -----------------------------
        self.PID_m1 = PIDController()
        self.PID_m2 = PIDController()

        # Start with small P. Add small D only if needed.
        # NOTE: You MUST tune these on your hardware.
        kp = 0.002
        kd = 0.0001

        for pid in (self.PID_m1, self.PID_m2):
            pid.setPID(kp, 0.0, kd)
            pid.setOutputLimit(900)       # correction limit (GV units)
            pid.setIntegralLimit(0.0)     # disables integral

        # UDP setup for viewer_xy.py
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._viewer_addr = ("127.0.0.1", 9999)
        self._last_send = 0.0
        self._send_period = 1.0 / 100.0  # 100 Hz

        print(f"Lemniscate amplitude a = {self.a} ticks")

        motor1.setHomingOffset(0)
        motor2.setHomingOffset(0)

        # Move to start pose: (x=+a, y=0)
        motor2.forceZero()
        motor2.setGoalPosition(0)
        motor1.setGoalPosition(int(self.a))

        # Optional profile settings
        motor1.setProfileVelocity(1500, verbose=True)
        motor1.setProfileAcceleration(10765, verbose=True)
        motor2.setProfileVelocity(1500, verbose=True)
        motor2.setProfileAcceleration(10765, verbose=True)

        # Wait for start pose and zeroing
        while True:
            if (
                motor1.reachedGoalPosition(verbose=False, tolerance=1)
                and motor2.reachedGoalPosition(verbose=False, tolerance=1)
            ):
                print("All motors ready at start pose.")
                print("Type C to continue...")
                if input().lower() == "c":
                    self.last_time_s = time.time()
                    motors.setOpmode(OpModes.VELOCITY)  # stay in VELOCITY forever after this
                    print("Continuing in VELOCITY mode...")
                    break
            time.sleep(0.01)

    def loop(self):
        # -----------------------------
        # Timing
        # -----------------------------
        now_s = time.time()
        dt_s = now_s - self.last_time_s
        self.last_time_s = now_s
        if dt_s <= 0.0 or dt_s > 0.45:
            dt_s = 0.01

        # -----------------------------
        # Trajectory: smooth Bernoulli lemniscate (parametric, continuous)
        #
        # x(t)= a cos t / (1+sin^2 t)
        # y(t)= a sin t cos t / (1+sin^2 t)
        #
        # Derivatives wrt parameter t:
        # dx/dt = a * s*(s^2 - 3) / (1+s^2)^2
        # dy/dt = a * (1 - 3 s^2) / (1+s^2)^2
        #
        # Time derivatives: xdot = (dx/dt)*omega, ydot=(dy/dt)*omega
        # -----------------------------
        omega = math.pi / 10.0  # rad/s of the PARAMETER t (not polar theta)
        self.t += omega * dt_s

        a = float(self.a)
        s = math.sin(self.t)
        c = math.cos(self.t)
        den = 1.0 + s * s

        x_des = a * c / den
        y_des = a * s * c / den

        dx_dt = a * (s * (s * s - 3.0)) / (den * den)
        dy_dt = a * (1.0 - 3.0 * s * s) / (den * den)

        vx_des = dx_dt * omega  # ticks/s
        vy_des = dy_dt * omega  # ticks/s

        # Debug quadrant state
        if x_des >= 0 and y_des >= 0:
            self.state = fsm.POSXPOSY
        elif x_des < 0 and y_des >= 0:
            self.state = fsm.NEGXPOSY
        elif x_des < 0 and y_des < 0:
            self.state = fsm.NEGXNEGY
        else:
            self.state = fsm.POSXNEGY

        # -----------------------------
        # Feedforward in GoalVelocity units
        # -----------------------------
        gv1_ff = ff_gv_for_axis(vx_des, True)
        gv2_ff = ff_gv_for_axis(vy_des, True)

        # -----------------------------
        # Read positions (ticks)
        # -----------------------------
        p1 = motor1.getCurrentPosition(verbose=False)
        p2 = motor2.getCurrentPosition(verbose=False)

        # -----------------------------
        # PID corrections (GV units)
        # PID.calculate(reference=current ticks, target=desired ticks, dt)
        # -----------------------------
        c1 = self.PID_m1.calculate(p1, x_des, dt_s)
        c2 = self.PID_m2.calculate(p2, y_des, dt_s)

        gv1 = clamp(int(round(gv1_ff + c1)), GV_MIN, GV_MAX)
        gv2 = clamp(int(round(gv2_ff + c2)), GV_MIN, GV_MAX)

        # -----------------------------
        # Command velocities EVERY loop (no gaps)
        # -----------------------------
        motor1.setGoalVelocity(gv1, verbose=False)
        motor2.setGoalVelocity(gv2, verbose=False)

        # Viewer uses measured positions directly (2-motor case)
        x_meas = p1
        y_meas = p2

        # UDP viewer
        if now_s - self._last_send >= self._send_period:
            self._last_send = now_s
            msg = f"{int(x_meas)},{int(y_meas)},{int(self.a)},{self.state.name}\n"
            try:
                self._udp.sendto(msg.encode(), self._viewer_addr)
            except OSError:
                pass

        print(
            f"t={self.t:7.3f}  state={self.state.name}  "
            f"x_des={x_des:9.1f} y_des={y_des:9.1f}  "
            f"gv=[{gv1},{gv2}]"
        )

    def run(self):
        try:
            while True:
                self.loop()
        except KeyboardInterrupt:
            print("Routine stopped by user.")
            self.onStop()

    def onStop(self):
        motors.stop()


if __name__ == "__main__":
    routine = test932L()
    routine.run()
