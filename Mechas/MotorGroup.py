from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Union

from Mechas.DCMotor import DCMotor
from Misc.OpModes import OpModes
from Misc.ControlTable import ControlTable as CT
# from Misc.int32 import int32
from Misc.Helpers import int32

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite


def _int16(u16: int) -> int:
    """Convert unsigned 16-bit -> signed 16-bit."""
    return u16 - 0x10000 if (u16 & 0x8000) else u16


def _le_bytes(value: int, length: int) -> List[int]:
    """
    Pack signed/unsigned int into little-endian byte list of `length`.
    Negative values become two's-complement of that byte-width.
    """
    mask = (1 << (8 * length)) - 1
    v = value & mask
    return [(v >> (8 * i)) & 0xFF for i in range(length)]


class MotorGroup:
    """
    Convenience wrapper around multiple DCMotor objects + SDK SyncRead/SyncWrite helpers.

    IMPORTANT PERFORMANCE NOTE:
    - SyncRead/SyncWrite objects are cached per (address, length) to avoid re-allocations.
    - PortHandler/PacketHandler are taken from the first motor in the group.
    - For best results: create motors with a shared port/packet (like you do with PortPacketManager),
      then build MotorGroup(m1, m2, ...).
    """

    def __init__(self, *motors: DCMotor):
        self.motors: Tuple[DCMotor, ...] = tuple(motors)
        if not self.motors:
            raise ValueError("MotorGroup requires at least one motor")

        # Cache SyncRead/SyncWrite objects: key=(start_addr, data_len)
        self._syncread_cache: Dict[Tuple[int, int], GroupSyncRead] = {}
        self._syncwrite_cache: Dict[Tuple[int, int], GroupSyncWrite] = {}

    # -------------------------
    # Construction helpers
    # -------------------------
    @classmethod
    def build(
        cls,
        ids: Sequence[int],
        *,
        port=None,
        packet=None,
        **motor_kwargs,
    ) -> "MotorGroup":
        """
        Build a MotorGroup from IDs, optionally sharing a provided port/packet.

        Example:
            from Misc.PortPacketManager import PortPacketManager as ppm
            g = MotorGroup.build([0,1,2,3], port=ppm.getPortHandler(), packet=ppm.getPacketHandler())
        """
        motors = [DCMotor(dxl_id, port=port, packet=packet, **motor_kwargs) for dxl_id in ids]
        return cls(*motors)

    @property
    def ids(self) -> List[int]:
        return [m.DXL_ID for m in self.motors]

    def __iter__(self):
        return iter(self.motors)

    def _shared_port_packet(self):
        # Use first motor as source of truth.
        m0 = self.motors[0]
        return m0.getPortHandler(), m0.getPacketHandler()

    def setSharedPortPacket(self, port, packet):
        """
        Assign a shared PortHandler/PacketHandler to ALL motors (no re-open here).
        Useful if you built motors without passing shared handlers initially.
        """
        for m in self.motors:
            m.setPortHandler(port)
            m.setPacketHandler(packet)
        # invalidate caches (they hold old port/ph references)
        self._syncread_cache.clear()
        self._syncwrite_cache.clear()

    # -------------------------
    # Basic passthroughs
    # -------------------------
    def enableTorque(self):
        for motor in self.motors:
            print(f"Motor {motor.DXL_ID}:")
            motor.enableTorque()

    def disableTorque(self):
        for motor in self.motors:
            motor.disableTorque()

    def stop(self):
        for motor in self.motors:
            motor.stop()

    def getInfo(self, verbose: bool = False):
        infos = []
        for motor in self.motors:
            info = motor.getInfo(verbose=verbose)
            if verbose:
                print(f"Motor {motor.DXL_ID}: {info}")
            infos.append(info)
        return infos

    def getOpMode(self, verbose: bool = True):
        modes = {}
        for motor in self.motors:
            modes[motor.DXL_ID] = motor.getOpMode(verbose=verbose)
        return modes

    def setOpmode(self, opmode: OpModes):
        for motor in self.motors:
            motor.setOpMode(opmode)

    def isMoving(self, verbose: bool = True):
        states = {}
        for motor in self.motors:
            states[motor.DXL_ID] = motor.isMoving(verbose=verbose)
        return states

    # -------------------------
    # Missing-ish DCMotor methods (added to group)
    # -------------------------
    def setCurrentPosition(self, position: int, verbose: bool = False):
        for motor in self.motors:
            motor.setCurrentPosition(position, verbose=verbose)

    def getMaxPositionLimit(self, verbose: bool = False):
        return {m.DXL_ID: m.getMaxPositionLimit(verbose=verbose) for m in self.motors}

    def getMinPositionLimit(self, verbose: bool = False):
        return {m.DXL_ID: m.getMinPositionLimit(verbose=verbose) for m in self.motors}

    def getHomingOffset(self, verbose: bool = True):
        return {m.DXL_ID: m.getHomingOffset(verbose=verbose) for m in self.motors}

    def setHomingOffset(self, offset: int, verbose: bool = True):
        for m in self.motors:
            m.setHomingOffset(offset, verbose=verbose)

    def resetEncoder(self, verbose: bool = True):
        for m in self.motors:
            m.resetEncoder(verbose=verbose)

    def setProfileVelocity(self, velocity: int, verbose: bool = True):
        for m in self.motors:
            m.setProfileVelocity(velocity, verbose=verbose)

    def setProfileAcceleration(self, acceleration: int, verbose: bool = True):
        for m in self.motors:
            m.setProfileAcceleration(acceleration, verbose=verbose)

    def getProfileVelocity(self, verbose: bool = False):
        return {m.DXL_ID: m.getProfileVelocity(verbose=verbose) for m in self.motors}

    def getProfileAcceleration(self, verbose: bool = False):
        return {m.DXL_ID: m.getProfileAcceleration(verbose=verbose) for m in self.motors}

    def setTorqueOnByGoalUpdate(self, enable: bool = True, verbose: bool = True):
        for m in self.motors:
            m.setTorqueOnByGoalUpdate(enable=enable, verbose=verbose)

    def setProfileTimeBased(self, enable: bool = True, verbose: bool = True):
        for m in self.motors:
            m.setProfileTimeBased(enable=enable, verbose=verbose)

    # -------------------------
    # Position helpers
    # -------------------------
    def setGoalPosition(self, goal: int, verbose: bool = True):
        for motor in self.motors:
            motor.setGoalPosition(goal, verbose=verbose)

    def getCurrentPosition(self, verbose: bool = True):
        for motor in self.motors:
            print(f"Motor {motor.DXL_ID}: {motor.getCurrentPosition(verbose=verbose)}")

    def getGoalPosition(self):
        return {m.DXL_ID: m.getGoalPosition(verbose=False) for m in self.motors}

    def reachedGoalPosition(self, tolerance: int = 0, verbose: bool = True):
        return {m.DXL_ID: m.reachedGoalPosition(tolerance=tolerance, verbose=verbose) for m in self.motors}

    def reachedPosition(self, reference: int, tolerance: int = 0, verbose: bool = True):
        return {m.DXL_ID: m.reachedPosition(reference, tolerance=tolerance, verbose=verbose) for m in self.motors}

    def zero(self, verbose: bool = True):
        for motor in self.motors:
            motor.zero(verbose=verbose)

    def forceZero(self, verbose: bool = True):
        for motor in self.motors:
            motor.forceZero(verbose=verbose)

    def rotateByAngle(self, angle: float, verbose: bool = True):
        for motor in self.motors:
            if verbose:
                print(f"Rotating motor {motor.DXL_ID} by {angle} degrees")
            motor.rotateByAngle(angle, verbose=verbose)

    # -------------------------
    # Torque / drive
    # -------------------------
    def getTorque(self, verbose: bool = True):
        return {m.DXL_ID: m.getTorque(verbose=verbose) for m in self.motors}

    def getDriveMode(self, verbose: bool = True):
        return {m.DXL_ID: m.getDriveMode(verbose=verbose) for m in self.motors}

    def setDriveMode(self, value: int, verbose: bool = True):
        for m in self.motors:
            if verbose:
                print(f"Setting drive mode of motor {m.DXL_ID} to 0x{value & 0xFF:02X}")
            m.setDriveMode(value, verbose=verbose)

    def setReverseMode(self, enable: bool = True, verbose: bool = True):
        for m in self.motors:
            if verbose:
                print(f"{'Enabling' if enable else 'Disabling'} reverse mode for motor {m.DXL_ID}")
            m.setReverseMode(enable=enable, verbose=verbose)

    def invert(self, verbose: bool = True):
        for m in self.motors:
            if verbose:
                print(f"Inverting direction for motor {m.DXL_ID}")
            m.invert()

    # -------------------------
    # Velocity / PWM / Current
    # -------------------------
    def getCurrentVelocity(self, verbose: bool = True):
        return {m.DXL_ID: m.getCurrentVelocity(verbose=verbose) for m in self.motors}

    def setGoalVelocity(self, velocity: int, verbose: bool = True, force: bool = False):
        for m in self.motors:
            if verbose:
                print(f"Setting goal velocity of motor {m.DXL_ID} to {velocity}")
            m.setGoalVelocity(velocity, verbose=verbose, force=force)

    def getPWMLimit(self, verbose: bool = False):
        return {m.DXL_ID: m.getPWMLimit(verbose=verbose) for m in self.motors}

    def getGoalPWM(self, verbose: bool = True):
        return {m.DXL_ID: m.getGoalPWM(verbose=verbose) for m in self.motors}

    def setGoalPWM(self, pwm: int, verbose: bool = True, force: bool = False):
        for m in self.motors:
            if verbose:
                print(f"Setting goal PWM of motor {m.DXL_ID} to {pwm}")
            m.setGoalPWM(pwm, verbose=verbose, force=force)

    def getGoalCurrent(self, verbose: bool = True):
        return {m.DXL_ID: m.getGoalCurrent(verbose=verbose) for m in self.motors}

    def setGoalCurrent(self, current: int, verbose: bool = True):
        for m in self.motors:
            if verbose:
                print(f"Setting goal current of motor {m.DXL_ID} to {current}")
            m.setGoalCurrent(current, verbose=verbose)

    # -------------------------
    # LED
    # -------------------------
    def getLEDColor(self, verbose: bool = True):
        return {m.DXL_ID: m.getLEDColor(verbose=verbose) for m in self.motors}

    def setLEDColor(self, color, verbose: bool = True):
        for m in self.motors:
            if verbose:
                print(f"Setting LED color for motor {m.DXL_ID} to {color}")
            m.setLEDColor(color, verbose=verbose)

    # =====================================================================
    # SYNC READ (GroupSyncRead)
    # =====================================================================
    def _get_sync_reader(self, start_address: int, data_length: int) -> GroupSyncRead:
        key = (int(start_address), int(data_length))
        if key not in self._syncread_cache:
            port, ph = self._shared_port_packet()
            sr = GroupSyncRead(port, ph, int(start_address), int(data_length))
            for m in self.motors:
                sr.addParam(m.DXL_ID)
            self._syncread_cache[key] = sr
        else:
            sr = self._syncread_cache[key]
            # ensure all IDs are present (if group changed)
            for m in self.motors:
                if m.DXL_ID not in sr.data_dict:
                    sr.addParam(m.DXL_ID)
        return sr

    def sync_read_raw(self, start_address: int, data_length: int) -> Dict[int, int]:
        """
        Generic SyncRead for a contiguous block.
        Returns dict {id: raw_unsigned_value_for_that_len}.
        """
        sr = self._get_sync_reader(start_address, data_length)
        result = sr.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"GroupSyncRead txRxPacket failed: {result}")

        out: Dict[int, int] = {}
        for m in self.motors:
            dxl_id = m.DXL_ID
            raw = sr.getData(dxl_id, start_address, data_length)
            out[dxl_id] = raw
        return out

    # Common “nice” sync-reads
    def sync_read_current_position(self) -> Dict[int, int]:
        raw = self.sync_read_raw(CT.CURRENT_POSITION.value, 4)
        return {dxl_id: int32(v) for dxl_id, v in raw.items()}

    def sync_read_goal_position(self) -> Dict[int, int]:
        raw = self.sync_read_raw(CT.GOAL_POSITION.value, 4)
        return {dxl_id: int32(v) for dxl_id, v in raw.items()}

    def sync_read_current_velocity(self) -> Dict[int, int]:
        raw = self.sync_read_raw(CT.CURRENT_VELOCITY.value, 4)
        return {dxl_id: int32(v) for dxl_id, v in raw.items()}

    def sync_read_moving(self) -> Dict[int, int]:
        # MOVING is 1 byte
        return self.sync_read_raw(CT.MOVING.value, 1)

    def sync_read_goal_velocity(self) -> Dict[int, int]:
        raw = self.sync_read_raw(CT.GOAL_VELOCITY.value, 4)
        return {dxl_id: int32(v) for dxl_id, v in raw.items()}

    def sync_read_goal_pwm(self) -> Dict[int, int]:
        raw = self.sync_read_raw(CT.GOAL_PWM.value, 2)
        return {dxl_id: _int16(v) for dxl_id, v in raw.items()}

    def sync_read_goal_current(self) -> Dict[int, int]:
        raw = self.sync_read_raw(CT.GOAL_CURRENT.value, 2)
        return {dxl_id: _int16(v) for dxl_id, v in raw.items()}

    # =====================================================================
    # SYNC WRITE (GroupSyncWrite)
    # =====================================================================
    def _get_sync_writer(self, start_address: int, data_length: int) -> GroupSyncWrite:
        key = (int(start_address), int(data_length))
        if key not in self._syncwrite_cache:
            port, ph = self._shared_port_packet()
            sw = GroupSyncWrite(port, ph, int(start_address), int(data_length))

            # Pre-add IDs with zeros so we can update via changeParam() each call.
            zero_bytes = [0] * int(data_length)
            for m in self.motors:
                sw.addParam(m.DXL_ID, zero_bytes)
            self._syncwrite_cache[key] = sw
        else:
            sw = self._syncwrite_cache[key]
            # ensure all IDs present
            for m in self.motors:
                if m.DXL_ID not in sw.data_dict:
                    sw.addParam(m.DXL_ID, [0] * int(data_length))
        return sw

    def sync_write_raw(self, start_address: int, data_length: int, values_by_id: Dict[int, int]) -> None:
        """
        Generic SyncWrite for a contiguous register of fixed length.
        values_by_id: {dxl_id: int_value}
        """
        sw = self._get_sync_writer(start_address, data_length)
        for dxl_id, val in values_by_id.items():
            sw.changeParam(dxl_id, _le_bytes(int(val), int(data_length)))
        result = sw.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"GroupSyncWrite txPacket failed: {result}")

    # Practical sync-writes:
    def sync_write_goal_velocity(self, velocities_by_id: Dict[int, int]) -> None:
        self.sync_write_raw(CT.GOAL_VELOCITY.value, 4, velocities_by_id)

    def sync_write_goal_position(self, positions_by_id: Dict[int, int]) -> None:
        self.sync_write_raw(CT.GOAL_POSITION.value, 4, positions_by_id)

    def sync_write_goal_pwm(self, pwm_by_id: Dict[int, int]) -> None:
        self.sync_write_raw(CT.GOAL_PWM.value, 2, pwm_by_id)

    def sync_write_goal_current(self, current_by_id: Dict[int, int]) -> None:
        self.sync_write_raw(CT.GOAL_CURRENT.value, 2, current_by_id)

    def sync_write_profile_velocity(self, profile_vel_by_id: Dict[int, int]) -> None:
        self.sync_write_raw(CT.PROFILE_VELOCITY.value, 4, profile_vel_by_id)

    def sync_write_profile_acceleration(self, profile_acc_by_id: Dict[int, int]) -> None:
        self.sync_write_raw(CT.PROFILE_ACCELERATION.value, 4, profile_acc_by_id)
