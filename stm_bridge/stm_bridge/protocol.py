"""CAN frame of the STM32 servo node (team-jax/CAN_STM).

Pure functions, standard library only, so the byte layout is checked without a bus
(tests/check_protocol.py).

team-jax/CAN_STM (Stm32servosepc.md section 3, src/main.c):
  standard 11-bit id 5, data [motor_id][angle_x10 H][angle_x10 L], motor_id 0..9,
  angle_x10 0..1800 (0.0..180.0 deg). The firmware clips out-of-range angles and drops
  motor_id > 9; we clip and reject before sending so the caller knows.
"""
from dataclasses import dataclass

SERVO_STDID = 5
SERVO_COUNT = 10
SERVO_X10_MAX = 1800


@dataclass(frozen=True)
class Frame:
    arb_id: int
    extended: bool
    data: bytes

    def __str__(self):
        # cansend/candump notation, so a printed frame can be compared with `candump can0`
        ident = f"{self.arb_id:08X}" if self.extended else f"{self.arb_id:03X}"
        return f"{ident}#{self.data.hex().upper()}"


def servo_frame(motor_id: int, angle_deg: float) -> Frame:
    """One servo command. The angle is clipped to 0..180 deg like the firmware does."""
    if not 0 <= motor_id < SERVO_COUNT:
        raise ValueError(f"servo motor_id {motor_id} is not 0..{SERVO_COUNT - 1}")
    x10 = min(max(round(angle_deg * 10), 0), SERVO_X10_MAX)
    return Frame(SERVO_STDID, False, bytes([motor_id, x10 >> 8, x10 & 0xFF]))


# Pressure frame from the STM32 (user spec, 2026-11-19): standard id 6, 6 data bytes = 6 sensors, each
# 0..255, every 100 ms. A byte is the top 8 bits of a 10-bit ADC, so the ADC fraction is byte * 4 / 1023.
PRESSURE_STDID = 6
PRESSURE_COUNT = 6


def pressure_adc(arb_id: int, extended: bool, data: bytes):
    """The 6 ADC values (byte * 4 / 1023, 0..~0.997) of a pressure frame, or None if it is not one
    (other id, extended id, or not exactly 6 bytes)."""
    if arb_id != PRESSURE_STDID or extended or len(data) != PRESSURE_COUNT:
        return None
    from .pressure import adc_from_byte
    return [adc_from_byte(b) for b in data]
