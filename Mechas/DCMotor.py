from __future__ import annotations

from enum import IntEnum
from typing import Any

from dynamixel_sdk import PortHandler, PacketHandler

from Misc.ControlTable import ControlTable_H42P as CT_H42P
from Misc.ControlTable import ControlTable_L42 as CT_L42
from Misc.ControlTable import ControlTable_M42 as CT_M42
from Misc.Helpers import int32, int16, clamp
from Misc.OpModes import OpModes
from Misc.Colors import Colors

import platform


def get_tty_port() -> str:
    system = platform.system()
    if system == "Windows":
        return "COM3"
    if system == "Linux":
        return "/dev/ttyUSB0"
    if system == "Darwin":
        return "/dev/tty.usbserial-FT66WNPY"
    raise RuntimeError("Unsupported operating system")


class DXLModels(IntEnum):
    H42P = 0
    L42 = 1
    M42 = 2

class DXLConfig():
    def __init__(self, model: DXLModels):
        if model == DXLModels.H42P:
            self.pos_deg_per_tick = 0.00059259
            self.vel_rpm_per_unit = 0.01
            self.vel_raw_min = -2920
            self.vel_raw_max = 2920
            self.cur_mA_per_unit = 1.0
            self.cur_raw_min = -4500
            self.cur_raw_max = 4500
            # Profile acceleration exists; unit scale is 1 rev/min^2 per raw unit (as given)
            self.profile_accel_rpm2_per_unit = 1.0
            # Profile acceleration raw range not provided; we clamp to ACCELERATION_LIMIT if available.
        elif model == DXLModels.L42:
            self.pos_deg_per_tick = 0.087891
            self.vel_rpm_per_unit = 0.114
            self.vel_raw_min = -256
            self.vel_raw_max = 256
            # Goal acceleration exists; raw range 0..255, scale 6612 rev/min^2 per raw unit
            self.goal_accel_rpm2_per_unit = 6612.0
            self.goal_accel_raw_min = 0
            self.goal_accel_raw_max = 255
            # Uses Goal Torque (current-based). Scale 4.0283 mA per unit; raw range -987..987
            self.torque_mA_per_unit = 4.0283
            self.torque_raw_min = -987
            self.torque_raw_max = 987
        elif model == DXLModels.M42:
            self.pos_deg_per_tick = 0.0013678
            self.vel_rpm_per_unit = 0.0038908
            self.vel_raw_min = -7504
            self.vel_raw_max = 7504
            # Goal acceleration exists; raw range 0..255, scale 225.66 rev/min^2 per raw unit
            self.goal_accel_rpm2_per_unit = 225.66
            self.goal_accel_raw_min = 0
            self.goal_accel_raw_max = 255
            # Uses Goal Torque (current-based). Scale 4.0283 mA per unit; raw range -300..300
            self.torque_mA_per_unit = 4.0283
            self.torque_raw_min = -300
            self.torque_raw_max = 300
    
    # def from_model(model: DXLModels) -> DXLConfig:
    #     return DXLConfig(model)
    
    # def get_pos_deg_per_tick(self, model: str) -> float:
    #     config = self.from_model(model)
    #     return config.pos_deg_per_tick

class DCMotor:
    _MODEL_CFG = {
        "H42P": {
            "pos_deg_per_tick": 0.00059259,
            "vel_rpm_per_unit": 0.01,
            "vel_raw_min": -2920,
            "vel_raw_max": 2920,
            "cur_mA_per_unit": 1.0,
            "cur_raw_min": -4500,
            "cur_raw_max": 4500,
            # Profile acceleration exists; unit scale is 1 rev/min^2 per raw unit (as given)
            "profile_accel_rpm2_per_unit": 1.0,
            # Profile acceleration raw range not provided; we clamp to ACCELERATION_LIMIT if available.
        },
        "L42": {
            "pos_deg_per_tick": 0.087891,
            "vel_rpm_per_unit": 0.114,
            "vel_raw_min": -256,
            "vel_raw_max": 256,
            # "vel_raw_min": -1200,
            # "vel_raw_max": 1200,
            # Goal acceleration exists; raw range 0..255, scale 6612 rev/min^2 per raw unit
            "goal_accel_rpm2_per_unit": 6612.0,
            "goal_accel_raw_min": 0,
            "goal_accel_raw_max": 255,
            # Uses Goal Torque (current-based). Scale 4.0283 mA per unit; raw range -987..987
            "torque_mA_per_unit": 4.0283,
            "torque_raw_min": -987,
            "torque_raw_max": 987,
        },
        "M42": {
            "pos_deg_per_tick": 0.0013678,
            "vel_rpm_per_unit": 0.0038908,
            "vel_raw_min": -7504,
            "vel_raw_max": 7504,
            # Goal acceleration exists; raw range 0..255, scale 225.66 rev/min^2 per raw unit
            "goal_accel_rpm2_per_unit": 225.66,
            "goal_accel_raw_min": 0,
            "goal_accel_raw_max": 255,
            # Uses Goal Torque (current-based). Scale 4.0283 mA per unit; raw range -300..300
            "torque_mA_per_unit": 4.0283,
            "torque_raw_min": -300,
            "torque_raw_max": 300,
        },
    }

    def __init__(
        self,
        DXL_ID: int,
        MODEL: str,
        BAUD: int = 1_000_000,
        PORT: str | None = None,
        packet: PacketHandler | None = None,
        port: PortHandler | None = None,
        auto_open: bool = True,
        verbose: bool = False,
    ):
        assert MODEL in ("H42P", "L42", "M42"), "Supported models: H42P, L42, M42"

        self.DXL_ID = int(DXL_ID)
        self.MODEL = MODEL
        self.PORT = PORT if PORT is not None else get_tty_port()
        self.BAUD = int(BAUD)
        self.PROTOCOL_VER = 2

        self.CT: type[IntEnum] = CT_H42P if MODEL == "H42P" else CT_L42 if MODEL == "L42" else CT_M42
        self.cfg = self._MODEL_CFG[MODEL]

        self._owns_port = port is None
        self._owns_packet = packet is None

        self.port = port if port is not None else PortHandler(self.PORT)
        self.packet = packet if packet is not None else PacketHandler(self.PROTOCOL_VER)

        # if auto_open and self._owns_port:
        #     assert self.port.openPort(), f"Failed to open port {self.PORT}"
        #     assert self.port.setBaudRate(self.BAUD), f"Failed to set baud {self.BAUD} on {self.PORT}"
        if auto_open:
            # DynamixelSDK PortHandler has .ser (pyserial object). It's None until openPort() succeeds.
            if getattr(self.port, "ser", None) is None:
                assert self.port.openPort(), f"Failed to open port {self.PORT}"
            assert self.port.setBaudRate(self.BAUD), f"Failed to set baud {self.BAUD} on {self.PORT}"

        if verbose:
            self.getInfo(verbose=True)

        try:
            if self.DXL_ID == 0:
                self.setLEDColor(Colors.RED.value, verbose=False)
            elif self.DXL_ID == 1:
                self.setLEDColor(Colors.GREEN.value, verbose=False)
            elif self.DXL_ID == 2:
                self.setLEDColor(Colors.BLUE.value, verbose=False)
            elif self.DXL_ID == 3:
                self.setLEDColor(Colors.YELLOW.value, verbose=False)
            elif self.DXL_ID == 4:
                self.setLEDColor(Colors.MAGENTA.value, verbose=False)
        except AssertionError as e:
            print(f"Warning: Failed to set LED color for ID {self.DXL_ID}: {e}")
            pass
        
        # set velocity limit as per config:
        try:
            self.setVelocityLimit(self.cfg["vel_raw_max"], verbose=True)
        except AssertionError as e:
            print(f"Warning: Failed to set velocity limit for ID {self.DXL_ID}: {e}")
            pass
            
    def close(self) -> None:
        if self._owns_port:
            try:
                self.port.closePort()
            except Exception:
                pass

    # ---------- capability / low-level helpers ----------

    def _has(self, name: str) -> bool:
        return hasattr(self.CT, name)

    def _require(self, *names: str) -> None:
        missing = [n for n in names if not self._has(n)]
        assert not missing, f"{self.MODEL} does not support: {', '.join(missing)}"

    def _read1(self, addr: int) -> int:
        v, _, _ = self.packet.read1ByteTxRx(self.port, self.DXL_ID, addr)
        return int(v)

    def _read2(self, addr: int) -> int:
        v, _, _ = self.packet.read2ByteTxRx(self.port, self.DXL_ID, addr)
        return int(v)

    def _read4(self, addr: int) -> int:
        v, _, _ = self.packet.read4ByteTxRx(self.port, self.DXL_ID, addr)
        return int(v)

    def _write1(self, addr: int, value: int) -> None:
        self.packet.write1ByteTxRx(self.port, self.DXL_ID, addr, int(value) & 0xFF)

    def _write2(self, addr: int, value: int) -> None:
        self.packet.write2ByteTxRx(self.port, self.DXL_ID, addr, int(value) & 0xFFFF)

    def _write4(self, addr: int, value: int) -> None:
        self.packet.write4ByteTxRx(self.port, self.DXL_ID, addr, int(value))

    # ---------- unit conversion helpers ----------

    def _pos_raw_from_deg(self, deg: float) -> int:
        return int(round(float(deg) / float(self.cfg["pos_deg_per_tick"])))

    def _pos_deg_from_raw(self, raw: int) -> float:
        return float(raw) * float(self.cfg["pos_deg_per_tick"])

    def _vel_raw_from_rpm(self, rpm: float) -> int:
        scale = float(self.cfg["vel_rpm_per_unit"])
        raw = int(round(float(rpm) / scale))
        raw = clamp(raw, int(self.cfg["vel_raw_min"]), int(self.cfg["vel_raw_max"]))
        return int(raw)

    def _vel_rpm_from_raw(self, raw: int) -> float:
        return float(raw) * float(self.cfg["vel_rpm_per_unit"])

    def _goal_accel_raw_from_rpm2(self, rpm2: float) -> int:
        assert self.MODEL in ("L42", "M42"), "Goal Acceleration is only supported on L42/M42"
        scale = float(self.cfg["goal_accel_rpm2_per_unit"])
        raw = int(round(float(rpm2) / scale))
        raw = clamp(raw, int(self.cfg["goal_accel_raw_min"]), int(self.cfg["goal_accel_raw_max"]))
        return int(raw)

    def _goal_accel_rpm2_from_raw(self, raw: int) -> float:
        assert self.MODEL in ("L42", "M42"), "Goal Acceleration is only supported on L42/M42"
        return float(raw) * float(self.cfg["goal_accel_rpm2_per_unit"])

    def _profile_accel_raw_from_rpm2(self, rpm2: float) -> int:
        assert self.MODEL == "H42P", "Profile Acceleration helper is intended for H42P"
        scale = float(self.cfg["profile_accel_rpm2_per_unit"])
        raw = int(round(float(rpm2) / scale))
        # Clamp to ACCELERATION_LIMIT if available (EEPROM), otherwise no clamp.
        if self._has("ACCELERATION_LIMIT"):
            lim = int32(self._read4(self.CT.ACCELERATION_LIMIT.value))
            if lim > 0:
                raw = clamp(raw, 0, int(lim))
        return int(raw)

    def _current_raw_from_mA(self, mA: float) -> int:
        assert self.MODEL == "H42P", "Goal Current is only supported on H42P"
        scale = float(self.cfg["cur_mA_per_unit"])
        raw = int(round(float(mA) / scale))
        raw = clamp(raw, int(self.cfg["cur_raw_min"]), int(self.cfg["cur_raw_max"]))
        return int(raw)

    def _current_mA_from_raw(self, raw: int) -> float:
        assert self.MODEL == "H42P", "Goal Current is only supported on H42P"
        return float(raw) * float(self.cfg["cur_mA_per_unit"])

    def _torque_raw_from_mA(self, mA: float) -> int:
        assert self.MODEL in ("L42", "M42"), "Goal Torque (current-based) is only supported on L42/M42"
        scale = float(self.cfg["torque_mA_per_unit"])
        raw = int(round(float(mA) / scale))
        raw = clamp(raw, int(self.cfg["torque_raw_min"]), int(self.cfg["torque_raw_max"]))
        return int(raw)

    def _torque_mA_from_raw(self, raw: int) -> float:
        assert self.MODEL in ("L42", "M42"), "Goal Torque (current-based) is only supported on L42/M42"
        return float(raw) * float(self.cfg["torque_mA_per_unit"])

    # ---------- basic info / modes ----------

    def getInfo(self, verbose: bool = False) -> dict[str, Any]:
        model, _, _ = self.packet.ping(self.port, self.DXL_ID)
        op = self.getOpMode(verbose=False)
        if verbose:
            try:
                name = OpModes(op).name
            except Exception:
                name = str(op)
            print(f"ID {self.DXL_ID} model={model} opmode={op} ({name})")
        return {"model": model, "opmode": op}

    def getOpMode(self, verbose: bool = False) -> int:
        self._require("OPERATING_MODE")
        op = self._read1(self.CT.OPERATING_MODE.value)
        if verbose:
            try:
                name = OpModes(op).name
            except Exception:
                name = str(op)
            print(f"ID {self.DXL_ID}: OpMode {op} ({name})")
        return op

    def setOpMode(self, opmode: OpModes, verbose: bool = False) -> None:
        self._require("OPERATING_MODE", "TORQUE_ENABLE")
        torque = self.getTorque(verbose=False)
        if torque == 1:
            self.disableTorque(verbose=False)
        self._write1(self.CT.OPERATING_MODE.value, int(opmode.value))
        if torque == 1:
            self.enableTorque(verbose=False)
        if verbose:
            self.getOpMode(verbose=True)

    # ---------- motion status ----------

    def isMoving(self, verbose: bool = False) -> int:
        self._require("MOVING")
        status = self._read1(self.CT.MOVING.value)
        if verbose:
            print(f"ID {self.DXL_ID}: {'MOVING' if status else 'NOT moving'}")
        return status

    # ---------- torque enable ----------

    def getTorque(self, verbose: bool = False) -> int:
        self._require("TORQUE_ENABLE")
        tm = self._read1(self.CT.TORQUE_ENABLE.value)
        if verbose:
            print(f"ID {self.DXL_ID}: Torque {'ON' if tm else 'OFF'}")
        return tm

    def enableTorque(self, verbose: bool = False) -> None:
        self._require("TORQUE_ENABLE")
        self._write1(self.CT.TORQUE_ENABLE.value, 1)
        if verbose:
            print(f"ID {self.DXL_ID}: Torque enabled")

    def disableTorque(self, verbose: bool = False) -> None:
        self._require("TORQUE_ENABLE")
        self._write1(self.CT.TORQUE_ENABLE.value, 0)
        if verbose:
            print(f"ID {self.DXL_ID}: Torque disabled")

    # ---------- position (raw + degrees) ----------

    def getPresentPosition(self, verbose: bool = False) -> int:
        self._require("PRESENT_POSITION")
        pos = int32(self._read4(self.CT.PRESENT_POSITION.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Present Position(raw) {pos}")
        return pos

    def getPresentPositionDeg(self, verbose: bool = False) -> float:
        raw = self.getPresentPosition(verbose=False)
        deg = self._pos_deg_from_raw(raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Present Position(deg) {deg}")
        return deg

    def getGoalPosition(self, verbose: bool = False) -> int:
        self._require("GOAL_POSITION")
        goal = int32(self._read4(self.CT.GOAL_POSITION.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Goal Position(raw) {goal}")
        return goal

    def getGoalPositionDeg(self, verbose: bool = False) -> float:
        raw = self.getGoalPosition(verbose=False)
        deg = self._pos_deg_from_raw(raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Goal Position(deg) {deg}")
        return deg

    def getMaxPositionLimit(self, verbose: bool = False) -> int:
        self._require("MAX_POSITION_LIMIT")
        v = int32(self._read4(self.CT.MAX_POSITION_LIMIT.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Max Position Limit(raw) {v}")
        return v

    def getMinPositionLimit(self, verbose: bool = False) -> int:
        self._require("MIN_POSITION_LIMIT")
        v = int32(self._read4(self.CT.MIN_POSITION_LIMIT.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Min Position Limit(raw) {v}")
        return v

    def setGoalPosition(self, goal_raw: int, verbose: bool = False) -> None:
        self._require("GOAL_POSITION", "OPERATING_MODE")
        op = self.getOpMode(verbose=False)
        print(f"DEBUG: setGoalPosition got op mode {op}, must be in {OpModes.POSITION.value} or {OpModes.EXTENDED_POSITION.value}")
        assert op in (OpModes.POSITION.value, OpModes.EXTENDED_POSITION.value), "Must be POSITION or EXTENDED_POSITION"

        g = int(goal_raw)
        if op == OpModes.POSITION.value:
            g = min(g, self.getMaxPositionLimit(verbose=False))
            g = max(g, self.getMinPositionLimit(verbose=False))

        self._write4(self.CT.GOAL_POSITION.value, g)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Goal Position(raw) {g}")

    def setGoalPositionDeg(self, goal_deg: float, verbose: bool = False) -> None:
        raw = self._pos_raw_from_deg(goal_deg)
        # clamp to limits if in POSITION mode
        self.setGoalPosition(raw, verbose=verbose)

    # ---------- homing offset ----------

    def getHomingOffset(self, verbose: bool = False) -> int:
        self._require("HOMING_OFFSET")
        off = int32(self._read4(self.CT.HOMING_OFFSET.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Homing Offset(raw) {off}")
        return off

    def setHomingOffset(self, offset: int, verbose: bool = False) -> None:
        self._require("HOMING_OFFSET", "TORQUE_ENABLE")
        torque = self.getTorque(verbose=False)
        if torque == 1:
            self.disableTorque(verbose=False)
        self._write4(self.CT.HOMING_OFFSET.value, int(offset))
        if torque == 1:
            self.enableTorque(verbose=False)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Homing Offset(raw) {int(offset)}")

    def resetEncoder(self, verbose: bool = False) -> None:
        cur = self.getPresentPosition(verbose=False) - self.getHomingOffset(verbose=False)
        if cur == 0:
            if verbose:
                print(f"ID {self.DXL_ID}: Encoder already zero")
            return
        self.setHomingOffset(-cur, verbose=False)
        if verbose:
            print(f"ID {self.DXL_ID}: Encoder reset (delta raw was {cur})")

    # ---------- reach / rotate / zero ----------

    def reachedGoalPosition(self, tolerance_raw: int = 0, verbose: bool = False) -> bool:
        delta = abs(self.getPresentPosition(verbose=False) - self.getGoalPosition(verbose=False))
        ok = delta <= int(tolerance_raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Delta(raw) {delta} reached={ok}")
        return ok

    def reachedGoalPositionDeg(self, tolerance_deg: float = 0.0, verbose: bool = False) -> bool:
        tol_raw = abs(self._pos_raw_from_deg(tolerance_deg))
        return self.reachedGoalPosition(tolerance_raw=tol_raw, verbose=verbose)

    def reachedPosition(self, reference_raw: int, tolerance_raw: int = 0, verbose: bool = False) -> bool:
        delta = abs(self.getPresentPosition(verbose=False) - int(reference_raw))
        ok = delta <= int(tolerance_raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Delta(raw) {delta} reached={ok}")
        return ok

    def rotateByAngleDeg(self, angle_deg: float, verbose: bool = False) -> None:
        if self.isMoving(verbose=False):
            return
        pos = self.getPresentPosition(verbose=False)
        delta = self._pos_raw_from_deg(angle_deg)
        self.setGoalPosition(pos + delta, verbose=False)
        if verbose:
            print(f"ID {self.DXL_ID}: Rotate {angle_deg}deg -> delta_raw {delta}")

    def zero(self, verbose: bool = False) -> None:
        self._require("OPERATING_MODE", "GOAL_POSITION")
        op = self.getOpMode(verbose=False)
        assert op in (OpModes.POSITION.value, OpModes.EXTENDED_POSITION.value), "Must be POSITION or EXTENDED_POSITION"
        self.setGoalPosition(0, verbose=verbose)

    def forceZero(self, verbose: bool = False) -> None:
        self._require("OPERATING_MODE", "GOAL_POSITION")
        prev = self.getOpMode(verbose=False)
        if prev not in (OpModes.POSITION.value, OpModes.EXTENDED_POSITION.value):
            self.setOpMode(OpModes.EXTENDED_POSITION, verbose=True)
            self.setGoalPosition(0, verbose=False)
            self.setOpMode(OpModes(prev), verbose=False)
        else:
            self.setGoalPosition(0, verbose=False)
        if verbose:
            print(f"ID {self.DXL_ID}: Force zero complete")

    # ---------- velocity (raw + rpm) ----------

    def getPresentVelocity(self, verbose: bool = False) -> int:
        self._require("PRESENT_VELOCITY")
        v = int32(self._read4(self.CT.PRESENT_VELOCITY.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Present Velocity(raw) {v}")
        return v

    def getPresentVelocityRpm(self, verbose: bool = False) -> float:
        raw = self.getPresentVelocity(verbose=False)
        rpm = self._vel_rpm_from_raw(raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Present Velocity(rpm) {rpm}")
        return rpm

    def setGoalVelocity(self, velocity_raw: int, verbose: bool = False, force: bool = False) -> None:
        self._require("GOAL_VELOCITY", "OPERATING_MODE")
        op = self.getOpMode(verbose=False)
        assert force or op == OpModes.VELOCITY.value, f"Must be VELOCITY mode (got {op})"
        v = clamp(int(velocity_raw), int(self.cfg["vel_raw_min"]), int(self.cfg["vel_raw_max"]))
        self._write4(self.CT.GOAL_VELOCITY.value, v)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Goal Velocity(raw) {v}")

    def setGoalVelocityRpm(self, rpm: float, verbose: bool = False, force: bool = False) -> None:
        raw = self._vel_raw_from_rpm(rpm)
        self.setGoalVelocity(raw, verbose=verbose, force=force)

    def setVelocityLimit(self, velocity_raw: int, verbose: bool = False) -> None:
        self._require("VELOCITY_LIMIT")
        v = clamp(int(velocity_raw), int(self.cfg["vel_raw_min"]), int(self.cfg["vel_raw_max"]))
        self._write4(self.CT.VELOCITY_LIMIT.value, v)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Velocity Limit(raw) {v}")

    # ---------- acceleration (L42/M42 goal accel) + (H42P profile accel) ----------

    def getGoalAcceleration(self, verbose: bool = False) -> int:
        self._require("GOAL_ACCELERATION")
        assert self.MODEL in ("L42", "M42"), "Goal Acceleration only supported on L42/M42"
        a = int32(self._read4(self.CT.GOAL_ACCELERATION.value))
        # user-stated raw range is 0..255
        a = clamp(int(a), int(self.cfg["goal_accel_raw_min"]), int(self.cfg["goal_accel_raw_max"]))
        if verbose:
            print(f"ID {self.DXL_ID}: Goal Acceleration(raw) {a}")
        return a

    def getGoalAccelerationRpm2(self, verbose: bool = False) -> float:
        raw = self.getGoalAcceleration(verbose=False)
        rpm2 = self._goal_accel_rpm2_from_raw(raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Goal Acceleration(rpm^2) {rpm2}")
        return rpm2

    def setGoalAcceleration(self, accel_raw: int, verbose: bool = False) -> None:
        self._require("GOAL_ACCELERATION")
        assert self.MODEL in ("L42", "M42"), "Goal Acceleration only supported on L42/M42"
        a = clamp(int(accel_raw), int(self.cfg["goal_accel_raw_min"]), int(self.cfg["goal_accel_raw_max"]))
        self._write4(self.CT.GOAL_ACCELERATION.value, a)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Goal Acceleration(raw) {a}")

    def setGoalAccelerationRpm2(self, accel_rpm2: float, verbose: bool = False) -> None:
        raw = self._goal_accel_raw_from_rpm2(accel_rpm2)
        self.setGoalAcceleration(raw, verbose=verbose)

    def getProfileAcceleration(self, verbose: bool = False) -> int:
        self._require("PROFILE_ACCELERATION")
        assert self.MODEL == "H42P", "Profile Acceleration only supported on H42P"
        a = int32(self._read4(self.CT.PROFILE_ACCELERATION.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Profile Acceleration(raw) {a}")
        return a

    def getProfileAccelerationRpm2(self, verbose: bool = False) -> float:
        raw = self.getProfileAcceleration(verbose=False)
        rpm2 = float(raw) * float(self.cfg["profile_accel_rpm2_per_unit"])
        if verbose:
            print(f"ID {self.DXL_ID}: Profile Acceleration(rpm^2) {rpm2}")
        return rpm2

    def setProfileAcceleration(self, accel_raw: int, verbose: bool = False) -> None:
        self._require("PROFILE_ACCELERATION")
        assert self.MODEL == "H42P", "Profile Acceleration only supported on H42P"
        a = int(accel_raw)
        # optional clamp to ACCELERATION_LIMIT if present
        if self._has("ACCELERATION_LIMIT"):
            lim = int32(self._read4(self.CT.ACCELERATION_LIMIT.value))
            if lim > 0:
                a = clamp(a, 0, int(lim))
        self._write4(self.CT.PROFILE_ACCELERATION.value, a)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Profile Acceleration(raw) {a}")

    def setProfileAccelerationRpm2(self, accel_rpm2: float, verbose: bool = False) -> None:
        raw = self._profile_accel_raw_from_rpm2(accel_rpm2)
        self.setProfileAcceleration(raw, verbose=verbose)

    # ---------- profile velocity (H42P only) in rpm ----------

    def getProfileVelocity(self, verbose: bool = False) -> int:
        self._require("PROFILE_VELOCITY")
        assert self.MODEL == "H42P", "Profile Velocity only supported on H42P"
        v = int32(self._read4(self.CT.PROFILE_VELOCITY.value))
        v = clamp(int(v), int(self.cfg["vel_raw_min"]), int(self.cfg["vel_raw_max"]))
        if verbose:
            print(f"ID {self.DXL_ID}: Profile Velocity(raw) {v}")
        return v

    def getProfileVelocityRpm(self, verbose: bool = False) -> float:
        raw = self.getProfileVelocity(verbose=False)
        rpm = self._vel_rpm_from_raw(raw)
        if verbose:
            print(f"ID {self.DXL_ID}: Profile Velocity(rpm) {rpm}")
        return rpm

    def setProfileVelocity(self, velocity_raw: int, verbose: bool = False) -> None:
        self._require("PROFILE_VELOCITY")
        assert self.MODEL == "H42P", "Profile Velocity only supported on H42P"
        v = clamp(int(velocity_raw), int(self.cfg["vel_raw_min"]), int(self.cfg["vel_raw_max"]))
        self._write4(self.CT.PROFILE_VELOCITY.value, v)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Profile Velocity(raw) {v}")

    def setProfileVelocityRpm(self, rpm: float, verbose: bool = False) -> None:
        raw = self._vel_raw_from_rpm(rpm)
        self.setProfileVelocity(raw, verbose=verbose)

    # ---------- current (H42P goal current) ----------

    def getPresentCurrent(self, verbose: bool = False) -> int:
        self._require("PRESENT_CURRENT")
        cur = int16(self._read2(self.CT.PRESENT_CURRENT.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Present Current(raw) {cur}")
        return cur

    def setGoalCurrent(self, current_raw: int, verbose: bool = False) -> None:
        self._require("GOAL_CURRENT", "OPERATING_MODE")
        assert self.MODEL == "H42P", "Goal Current only supported on H42P"
        op = self.getOpMode(verbose=False)
        assert op == OpModes.CURRENT.value, f"Must be CURRENT mode (got {op})"
        c = clamp(int(current_raw), int(self.cfg["cur_raw_min"]), int(self.cfg["cur_raw_max"]))
        self._write2(self.CT.GOAL_CURRENT.value, c)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Goal Current(raw) {c}")

    def setGoalCurrentmA(self, current_mA: float, verbose: bool = False) -> None:
        raw = self._current_raw_from_mA(current_mA)
        self.setGoalCurrent(raw, verbose=verbose)

    # ---------- torque (L42/M42 goal torque as current-based) ----------

    def setGoalTorque(self, torque_raw: int, verbose: bool = False) -> None:
        self._require("GOAL_TORQUE", "OPERATING_MODE")
        assert self.MODEL in ("L42", "M42"), "Goal Torque only supported on L42/M42"
        op = self.getOpMode(verbose=False)
        assert op == OpModes.CURRENT.value, f"Must be CURRENT mode (got {op})"
        t = clamp(int(torque_raw), int(self.cfg["torque_raw_min"]), int(self.cfg["torque_raw_max"]))
        self._write2(self.CT.GOAL_TORQUE.value, t)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Goal Torque(raw) {t}")

    def setGoalTorquemA(self, torque_mA: float, verbose: bool = False) -> None:
        raw = self._torque_raw_from_mA(torque_mA)
        self.setGoalTorque(raw, verbose=verbose)

    # ---------- PWM (H42P only) ----------

    def getPWMLimit(self, verbose: bool = False) -> int:
        self._require("PWM_LIMIT")
        limit = int32(self._read2(self.CT.PWM_LIMIT.value))
        if verbose:
            print(f"ID {self.DXL_ID}: PWM Limit(raw) {limit}")
        return limit

    def getGoalPWM(self, verbose: bool = False) -> int:
        self._require("GOAL_PWM")
        pwm = int16(self._read2(self.CT.GOAL_PWM.value))
        if verbose:
            print(f"ID {self.DXL_ID}: Goal PWM(raw) {pwm}")
        return pwm

    def setGoalPWM(self, pwm_raw: int, verbose: bool = False, force: bool = False) -> None:
        self._require("GOAL_PWM", "OPERATING_MODE", "PWM_LIMIT")
        assert self.MODEL == "H42P", "Goal PWM only supported on H42P"
        op = self.getOpMode(verbose=False)
        assert force or op == OpModes.PWM.value, f"Must be PWM mode (got {op})"
        limit = self.getPWMLimit(verbose=False)
        p = int(pwm_raw)
        if abs(p) > limit:
            p = limit if p > 0 else -limit
        self._write2(self.CT.GOAL_PWM.value, p)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Goal PWM(raw) {p}")

    # ---------- drive mode (only if present) ----------

    def getDriveMode(self, verbose: bool = False) -> int:
        self._require("DRIVE_MODE")
        dm = self._read1(self.CT.DRIVE_MODE.value)
        if verbose:
            print(f"ID {self.DXL_ID}: Drive Mode 0x{dm:02X}")
        return dm

    def setDriveMode(self, value: int, verbose: bool = False) -> int:
        self._require("DRIVE_MODE", "TORQUE_ENABLE")
        torque = self.getTorque(verbose=False)
        if torque == 1:
            self.disableTorque(verbose=False)
        v = int(value) & 0xFF
        self._write1(self.CT.DRIVE_MODE.value, v)
        if torque == 1:
            self.enableTorque(verbose=False)
        if verbose:
            print(f"ID {self.DXL_ID}: Set Drive Mode 0x{v:02X}")
        return v

    def setReverseMode(self, enable: bool = True, verbose: bool = False) -> int:
        self._require("DRIVE_MODE")
        dm = self.getDriveMode(verbose=False)
        dm = (dm | 0x01) if enable else (dm & ~0x01)
        return self.setDriveMode(dm, verbose=verbose)

    def invert(self, verbose: bool = False) -> None:
        self._require("DRIVE_MODE")
        dm = self.getDriveMode(verbose=False)
        self.setReverseMode(enable=not bool(dm & 0x01), verbose=verbose)

    def setTorqueOnByGoalUpdate(self, enable: bool = True, verbose: bool = False) -> int:
        self._require("DRIVE_MODE")
        dm = self.getDriveMode(verbose=False)
        dm = (dm | 0x08) if enable else (dm & ~0x08)
        return self.setDriveMode(dm, verbose=verbose)

    def setProfileTimeBased(self, enable: bool = True, verbose: bool = False) -> int:
        self._require("DRIVE_MODE")
        dm = self.getDriveMode(verbose=False)
        dm = (dm | 0x04) if enable else (dm & ~0x04)
        return self.setDriveMode(dm, verbose=verbose)

    # ---------- LED ----------

    def getLEDColor(self, verbose: bool = False) -> tuple[int, int, int]:
        self._require("LED_RED", "LED_GREEN", "LED_BLUE")
        r = self._read1(self.CT.LED_RED.value)
        g = self._read1(self.CT.LED_GREEN.value)
        b = self._read1(self.CT.LED_BLUE.value)
        if verbose:
            print(f"ID {self.DXL_ID}: LED R{r} G{g} B{b}")
        return (r, g, b)

    def setLEDColor(self, color, verbose: bool = False) -> None:
        self._require("LED_RED", "LED_GREEN", "LED_BLUE")
        if isinstance(color, Colors):
            color = color.value
        r, g, b = int(color[0]), int(color[1]), int(color[2])
        self._write1(self.CT.LED_RED.value, r)
        self._write1(self.CT.LED_GREEN.value, g)
        self._write1(self.CT.LED_BLUE.value, b)
        if verbose:
            print(f"ID {self.DXL_ID}: LED set to R{r} G{g} B{b}")

    # ---------- stop ----------

    def stop(self, verbose: bool = False) -> None:
        if self._has("TORQUE_ENABLE"):
            self.disableTorque(verbose=False)

        if self._has("GOAL_PWM"):
            self._write2(self.CT.GOAL_PWM.value, 0)

        if self._has("GOAL_VELOCITY"):
            self._write4(self.CT.GOAL_VELOCITY.value, 0)

        if self._has("GOAL_CURRENT"):
            self._write2(self.CT.GOAL_CURRENT.value, 0)

        if self._has("GOAL_TORQUE"):
            self._write2(self.CT.GOAL_TORQUE.value, 0)

        if verbose:
            print(f"ID {self.DXL_ID}: Stop issued")
