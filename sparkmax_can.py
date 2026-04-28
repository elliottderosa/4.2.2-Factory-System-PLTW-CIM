"""
sparkmax_can.py — Python/SocketCAN driver for REV Robotics SPARK MAX

Targets firmware 24.x. sparkcan reports incompatibility with 25.x; behavior
on that firmware is unverified.

Protocol references
-------------------
[1] https://docs.revrobotics.com/brushless/spark-max/control-interfaces
    29-bit extended-ID layout, periodic status frame contents.
[2] https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
    Underlying FRC CAN spec: device-type / manufacturer tables, roboRIO
    universal heartbeat (0x01011840) and its 8-byte bit layout.
[3] https://github.com/grayson-arendt/sparkcan
    C++ Linux SocketCAN reference for SPARK MAX. Authoritative source for any
    API Class / API Index that REV does not document publicly. Constants in
    this file marked "VERIFY" should be checked against
    include/SparkBase.hpp / src/SparkBase.cpp before depending on them.

Bus bring-up (Raspberry Pi, MCP2515 hat or CANable):
    sudo ip link set can0 up type can bitrate 250000

Quick test on a virtual bus (no hardware):
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up
    python3 sparkmax_can.py --channel vcan0 --device-id 1 --demo

Dependencies:
    pip install python-can
"""
from __future__ import annotations

import argparse
import struct
import sys
import threading
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional

import can


# ============================================================================
# §1  FRC CAN — fixed identifiers for SPARK MAX
# ============================================================================

DEVICE_TYPE_MOTOR_CONTROLLER = 2    # FRC Device Type table
MFR_REV_ROBOTICS             = 5    # FRC Manufacturer ID table

# roboRIO universal heartbeat. SPARK MAX listens for this to permit motor
# output. If absent for >100 ms, the controller treats the bus as dead and
# disables the motor. Real roboRIO sends this every 20 ms.
HEARTBEAT_ARB_ID = 0x01011840

DEFAULT_BITRATE = 1_000_000           # FRC standard


# ============================================================================
# §2  Extended-ID packing (well-documented; reference [1] §"CAN Packet Structure")
#
# 29-bit extended ID:
#   [28:24] Device Type     (5 bits)
#   [23:16] Manufacturer    (8 bits)
#   [15:10] API Class       (6 bits)
#   [ 9: 6] API Index       (4 bits)
#   [ 5: 0] Device ID       (6 bits)
# ============================================================================

def make_arb_id(api_class: int,
                api_index: int,
                device_id: int,
                device_type: int = DEVICE_TYPE_MOTOR_CONTROLLER,
                manufacturer: int = MFR_REV_ROBOTICS) -> int:
    """Pack a 29-bit extended arbitration ID for a SPARK MAX command/status."""
    if not 0 <= device_id <= 0x3F:
        raise ValueError(f"device_id must be 0–63, got {device_id}")
    if not 0 <= api_class <= 0x3F:
        raise ValueError(f"api_class must be 0–63, got {api_class}")
    if not 0 <= api_index <= 0x0F:
        raise ValueError(f"api_index must be 0–15, got {api_index}")
    return (
        ((device_type  & 0x1F) << 24)
        | ((manufacturer & 0xFF) << 16)
        | ((api_class   & 0x3F) << 10)
        | ((api_index   & 0x0F) <<  6)
        | ( device_id   & 0x3F)
    )


def unpack_arb_id(arb_id: int) -> tuple[int, int, int, int, int]:
    """Inverse of make_arb_id — returns (dev_type, mfr, api_class, api_idx, dev_id)."""
    return (
        (arb_id >> 24) & 0x1F,
        (arb_id >> 16) & 0xFF,
        (arb_id >> 10) & 0x3F,
        (arb_id >>  6) & 0x0F,
         arb_id        & 0x3F,
    )


# ============================================================================
# §3  Control-mode constants
#     From REV's "Configuration Parameters" docs, the kCtrlType enum.
# ============================================================================

class ControlType(IntEnum):
    DUTY_CYCLE     = 0
    VELOCITY       = 1
    VOLTAGE        = 2
    POSITION       = 3
    SMART_MOTION   = 4
    CURRENT        = 5
    SMART_VELOCITY = 6


# ============================================================================
# §4  Roborio heartbeat
#
# 8-byte payload, struct from [2]:
#   bit  0– 7  matchTimeSeconds
#   bit  8–17  matchNumber
#   bit 18–23  replayNumber
#   bit 24     redAlliance
#   bit 25     enabled            ← must be 1 for motor to run
#   bit 26     autonomous
#   bit 27     testMode
#   bit 28     systemWatchdog     ← must be 1 for motor to run
#   bit 29–31  tournamentType
#   bit 32–63  time-of-day fields (irrelevant for our use)
#
# To enable: byte[3] = (enabled<<1) | (systemWatchdog<<4) = 0x12
# ============================================================================

def build_heartbeat_payload(enabled: bool = True) -> bytes:
    """8-byte heartbeat payload. With enabled=True the SPARK MAX will run."""
    byte3 = 0
    if enabled:
        byte3 |= (1 << 1)   # enabled
        byte3 |= (1 << 4)   # systemWatchdog
    return bytes([0x00, 0x00, 0x00, byte3, 0x00, 0x00, 0x00, 0x00])


class HeartbeatTask:
    """
    Schedules the roboRIO heartbeat at 20 ms intervals using SocketCAN's
    in-kernel BCM. Drift is ~0 because the kernel handles the timer.

    Use as a context manager or call start()/stop() explicitly.
    """
    def __init__(self, bus: can.BusABC, period_s: float = 0.020,
                 enabled: bool = True):
        self._bus = bus
        self._period = period_s
        self._msg = can.Message(
            arbitration_id=HEARTBEAT_ARB_ID,
            is_extended_id=True,
            data=build_heartbeat_payload(enabled),
        )
        self._task: Optional[can.broadcastmanager.CyclicSendTaskABC] = None

    def start(self) -> None:
        if self._task is None:
            self._task = self._bus.send_periodic(self._msg, self._period)

    def stop(self) -> None:
        if self._task is not None:
            self._task.stop()
            self._task = None

    def set_enabled(self, enabled: bool) -> None:
        self._msg.data = bytearray(build_heartbeat_payload(enabled))
        if self._task is not None:
            # python-can supports modify_data on most periodic tasks
            try:
                self._task.modify_data(self._msg)
            except (AttributeError, NotImplementedError):
                # Fallback: stop & restart
                self.stop()
                self.start()

    def __enter__(self) -> "HeartbeatTask":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


# ============================================================================
# §5  Periodic status frames sent BY the device
#     Contents are documented in [1] under "Periodic Status N". Bit-level
#     packing of the 12-bit voltage/current fields in Status 1 is the most
#     fiddly part — the layout below matches REV's roboRIO SDK decoder
#     (verify against sparkcan's status decoders if you find disagreement
#     on the wire).
#
# Per the FRC CAN spec [2], API Class 6 is "Periodic Status". Each periodic
# status frame uses API Class = 6 with a distinct API Index. The values
# "0x060", "0x061", … "0x066" you'll see in REV-related docs are the
# combined 10-bit API ID = (api_class << 4) | api_index.
# ============================================================================

API_CLASS_PERIODIC_STATUS = 6

class StatusAPIIndex(IntEnum):
    STATUS_0 = 0    # Applied output, faults
    STATUS_1 = 1    # Velocity, temperature, voltage, current
    STATUS_2 = 2    # Position
    STATUS_3 = 3    # Analog sensor
    STATUS_4 = 4    # Alternate encoder
    STATUS_5 = 5    # Duty-cycle absolute encoder position / angle
    STATUS_6 = 6    # Duty-cycle absolute encoder velocity / frequency


@dataclass
class Status0:
    applied_output: float          # [-1.0, 1.0]
    faults: int                    # 16-bit bitfield
    sticky_faults: int             # 16-bit bitfield
    is_follower: bool


@dataclass
class Status1:
    velocity_rpm: float            # IEEE float32
    motor_temp_c: int              # 8-bit unsigned, °C  (firmware ≥1.x)
    bus_voltage_v: float           # 12-bit fixed-point → V
    motor_current_a: float         # 12-bit fixed-point → A


@dataclass
class Status2:
    position_rot: float            # IEEE float32, motor rotations


@dataclass
class Status3:
    analog_voltage_v: float
    analog_velocity_rpm: float
    analog_position_rot: float


@dataclass
class Status4:
    alt_encoder_velocity_rpm: float
    alt_encoder_position_rot: float


@dataclass
class Status5:
    abs_encoder_position_rot: float
    abs_encoder_angle: int         # 16-bit


@dataclass
class Status6:
    abs_encoder_velocity_rpm: float
    abs_encoder_frequency_hz: int  # 16-bit unsigned


def _i16_le(b: bytes, off: int) -> int:
    return struct.unpack_from("<h", b, off)[0]

def _u16_le(b: bytes, off: int) -> int:
    return struct.unpack_from("<H", b, off)[0]

def _f32_le(b: bytes, off: int) -> float:
    return struct.unpack_from("<f", b, off)[0]


def parse_status_0(data: bytes) -> Status0:
    """8-byte Periodic Status 0."""
    if len(data) < 8:
        raise ValueError(f"Status 0 needs 8 bytes, got {len(data)}")
    applied_raw = _i16_le(data, 0)
    faults       = _u16_le(data, 2)
    sticky       = _u16_le(data, 4)
    is_follower  = bool(data[6] & 0x01)
    return Status0(
        applied_output = applied_raw / 32767.0,
        faults         = faults,
        sticky_faults  = sticky,
        is_follower    = is_follower,
    )


def parse_status_1(data: bytes) -> Status1:
    """
    8-byte Periodic Status 1.
      bytes 0-3: float32 velocity (RPM)
      byte    4: uint8 motor temperature (°C)
      bytes 5-7: 12-bit voltage + 12-bit current, packed:
                   byte5      = voltage[7:0]
                   byte6 low4 = voltage[11:8]
                   byte6 high4= current[3:0]
                   byte7      = current[11:4]
    Voltage scale: REV's roboRIO SDK divides the 12-bit raw by 128.0 → volts.
    Current scale: 12-bit raw divided by 32.0 → amps.
    (Both scaling factors are the documented REV conventions; verify against
    sparkcan if a hardware reading looks off.)
    """
    if len(data) < 8:
        raise ValueError(f"Status 1 needs 8 bytes, got {len(data)}")
    velocity = _f32_le(data, 0)
    temp_c   = data[4]
    v_raw = data[5] | ((data[6] & 0x0F) << 8)
    i_raw = (data[6] >> 4) | (data[7] << 4)
    return Status1(
        velocity_rpm    = velocity,
        motor_temp_c    = temp_c,
        bus_voltage_v   = v_raw / 128.0,
        motor_current_a = i_raw / 32.0,
    )


def parse_status_2(data: bytes) -> Status2:
    """8-byte Periodic Status 2 — position only uses first 4 bytes."""
    if len(data) < 4:
        raise ValueError(f"Status 2 needs ≥4 bytes, got {len(data)}")
    return Status2(position_rot=_f32_le(data, 0))


def parse_status_4(data: bytes) -> Status4:
    if len(data) < 8:
        raise ValueError(f"Status 4 needs 8 bytes, got {len(data)}")
    return Status4(
        alt_encoder_velocity_rpm = _f32_le(data, 0),
        alt_encoder_position_rot = _f32_le(data, 4),
    )


def parse_status_5(data: bytes) -> Status5:
    if len(data) < 6:
        raise ValueError(f"Status 5 needs ≥6 bytes, got {len(data)}")
    return Status5(
        abs_encoder_position_rot = _f32_le(data, 0),
        abs_encoder_angle        = _u16_le(data, 4),
    )


def parse_status_6(data: bytes) -> Status6:
    if len(data) < 6:
        raise ValueError(f"Status 6 needs ≥6 bytes, got {len(data)}")
    return Status6(
        abs_encoder_velocity_rpm = _f32_le(data, 0),
        abs_encoder_frequency_hz = _u16_le(data, 4),
    )


_PARSERS = {
    StatusAPIIndex.STATUS_0: parse_status_0,
    StatusAPIIndex.STATUS_1: parse_status_1,
    StatusAPIIndex.STATUS_2: parse_status_2,
    StatusAPIIndex.STATUS_4: parse_status_4,
    StatusAPIIndex.STATUS_5: parse_status_5,
    StatusAPIIndex.STATUS_6: parse_status_6,
    # Status 3 (analog) intentionally omitted — its 10/22-bit packing is
    # rare in practice; add it from sparkcan if you need analog feedback.
}


def parse_periodic_status(arb_id: int, data: bytes):
    """
    Try to parse a CAN frame as a SPARK MAX periodic status. Returns
    (device_id, parsed_dataclass) or (None, None) if the frame isn't one
    of ours.
    """
    dt, mfr, api_class, api_index, dev_id = unpack_arb_id(arb_id)
    if dt != DEVICE_TYPE_MOTOR_CONTROLLER or mfr != MFR_REV_ROBOTICS:
        return None, None
    if api_class != API_CLASS_PERIODIC_STATUS:
        return None, None
    parser = _PARSERS.get(api_index)
    if parser is None:
        return None, None
    return dev_id, parser(data)


# ============================================================================
# §6  Setpoint / configuration commands sent TO the device
#
# !! VERIFY !! — REV does not publish the SPARK MAX command API IDs in a
# clean table. The constants below follow the FRC CAN motor-controller
# convention from spec [2] (API Class = control mode, API Index = command),
# which sparkcan also follows. Before trusting any of these on real
# hardware, grep sparkcan/include/SparkBase.hpp and src/SparkBase.cpp for
# the corresponding constants and reconcile.
#
# The setpoint command for each control mode is a single 8-byte frame:
#     bytes 0-3 : float32 setpoint (little-endian)
#     byte 4    : auxiliary setpoint (control-mode dependent; usually 0)
#     byte 5    : pidSlot[3:0] | arbFFUnits[7:4]
#     bytes 6-7 : int16 arbitrary feed-forward, scaled
#
# FRC convention (from WPILib FRC CAN spec, Table "API Class - Motor
# Controller"): each control mode gets its own 6-bit API Class. API Index 2
# within a class is "Set Setpoint".
#
# Duty-cycle mode is REV-specific (not in the FRC standard table). 0x08 is
# a plausible REV-assigned class but is NOT confirmed — verify.
# ============================================================================

API_INDEX_SET_SETPOINT = 2

class CmdAPIClass(IntEnum):
    VOLTAGE     = 0     # FRC: "Voltage Control Mode"
    SPEED       = 1     # FRC: "Speed Control Mode"
    VOLTAGE_CMP = 2     # FRC: "Voltage Compensation Mode"
    POSITION    = 3     # FRC: "Position Control Mode"
    CURRENT     = 4     # FRC: "Current Control Mode"
    DUTY_CYCLE  = 8     # ⚠ REV-specific; verify against sparkcan
    SMART_VEL   = 9     # ⚠ verify
    SMART_MOT   = 10    # ⚠ verify


def build_setpoint_payload(setpoint: float,
                           pid_slot: int = 0,
                           arb_ff: float = 0.0,
                           arb_ff_units: int = 0,
                           aux_setpoint: int = 0) -> bytes:
    """
    Encode an 8-byte SPARK MAX setpoint frame.

    arb_ff is scaled to int16 by /32.0 (volts) by REV's SDK convention; if
    you're using a different scale, encode the int16 yourself and pass the
    raw value via a custom build.
    """
    if not 0 <= pid_slot <= 0x0F:
        raise ValueError(f"pid_slot must be 0–15, got {pid_slot}")
    if not 0 <= arb_ff_units <= 0x0F:
        raise ValueError(f"arb_ff_units must be 0–15, got {arb_ff_units}")
    arb_ff_i16 = max(-32768, min(32767, int(round(arb_ff * 32.0))))
    flags = (pid_slot & 0x0F) | ((arb_ff_units & 0x0F) << 4)
    return struct.pack("<fBBh",
                       float(setpoint),
                       aux_setpoint & 0xFF,
                       flags,
                       arb_ff_i16)


# ============================================================================
# §7  High-level SparkMax wrapper
# ============================================================================

class SparkMax:
    """
    One instance per controller on the bus. Several SparkMax objects can
    share a single bus; only one HeartbeatTask is needed for the whole bus.

    Typical usage:
        bus = can.Bus(interface="socketcan", channel="can0")
        with HeartbeatTask(bus):
            motor = SparkMax(bus, device_id=1)
            motor.set_duty_cycle(0.25)
            time.sleep(2)
            motor.set_duty_cycle(0.0)
    """
    def __init__(self, bus: can.BusABC, device_id: int):
        if not 1 <= device_id <= 62:
            # 0 is the factory-default ID and 63 is reserved for broadcast.
            # Out-of-the-box devices are at ID 0 — change it via REV Hardware
            # Client before relying on multi-device addressing.
            raise ValueError(f"device_id should be 1–62, got {device_id}")
        self.bus = bus
        self.device_id = device_id

    # ----- low-level send -----
    def _send(self, api_class: int, api_index: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError(f"CAN payload max 8 bytes, got {len(data)}")
        self.bus.send(can.Message(
            arbitration_id=make_arb_id(api_class, api_index, self.device_id),
            is_extended_id=True,
            data=data,
        ))

    # ----- setpoint commands -----
    def set_duty_cycle(self, duty: float, **kw) -> None:
        """duty in [-1.0, 1.0]."""
        duty = max(-1.0, min(1.0, duty))
        self._send(CmdAPIClass.DUTY_CYCLE,
                   API_INDEX_SET_SETPOINT,
                   build_setpoint_payload(duty, **kw))

    def set_velocity(self, rpm: float, **kw) -> None:
        self._send(CmdAPIClass.SPEED,
                   API_INDEX_SET_SETPOINT,
                   build_setpoint_payload(rpm, **kw))

    def set_voltage(self, volts: float, **kw) -> None:
        self._send(CmdAPIClass.VOLTAGE,
                   API_INDEX_SET_SETPOINT,
                   build_setpoint_payload(volts, **kw))

    def set_position(self, rotations: float, **kw) -> None:
        self._send(CmdAPIClass.POSITION,
                   API_INDEX_SET_SETPOINT,
                   build_setpoint_payload(rotations, **kw))

    def stop(self) -> None:
        self.set_duty_cycle(0.0)


def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description="SPARK MAX SocketCAN demo")
    p.add_argument("--channel",   default="can0",
                   help="SocketCAN interface (e.g. can0, vcan0)")
    p.add_argument("--device-id", type=int, default=1,
                   help="SPARK MAX device ID (1–62)")
    p.add_argument("--demo", action="store_true",
                   help="Run the ramp demo")
    args = p.parse_args(argv)

    if args.demo:
        return _demo(args.channel, args.device_id)

    p.print_help()
    return 0

if __name__ == "__main__":
    sys.exit(main())
