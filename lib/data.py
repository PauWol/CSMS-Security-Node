import struct
from micropython import const
import time

from core.comms.mesh import mesh

from ld2410c import TargetFrame
from constants import GATEWAY_NODE_ID


def pack(entries: list[tuple[str, int, str]]) -> bytearray:
    """Encode list of (sensor, timestamp, value) → bytearray."""
    buf = bytearray()
    buf.append(len(entries))  # entry count (1 B)

    for sensor, ts, value in entries:
        s = sensor.encode()
        v = value.encode()
        buf.append(len(s))  # sensor str length (1 B)
        buf.extend(s)  # sensor bytes
        buf.extend(struct.pack(">I", ts))  # timestamp uint32 (4 B)
        buf.append(len(v))  # value str length (1 B)
        buf.extend(v)  # value bytes

    return buf


def unpack(buf: bytearray) -> list[tuple[str, int, str]]:
    """Decode bytearray → list of (sensor, timestamp, value)."""
    offset = 0
    n = buf[offset]
    offset += 1
    entries = []

    for _ in range(n):
        s_len = buf[offset]
        offset += 1
        sensor = buf[offset : offset + s_len].decode()
        offset += s_len
        ts = struct.unpack_from(">I", buf, offset)[0]
        offset += 4
        v_len = buf[offset]
        offset += 1
        value = buf[offset : offset + v_len].decode()
        offset += v_len
        entries.append((sensor, ts, value))

    return entries


PIR_IDX = const(0)
PIR_INDICATOR = const("P")

PHC_IDX = const(1)
PHC_INDICATOR = const("C")

RADAR_IDX = const(2)
RADAR_INDICATOR = const("R")


class SensorData:
    def __init__(self):
        self._data = [] * 3

    def pir(self, value: int):
        self._data[PIR_IDX] = (PIR_INDICATOR, time.ticks_ms(), "ON" if value else "OFF")

    def photo_cell(self, value: float):
        self._data[PHC_IDX] = (PHC_INDICATOR, time.ticks_ms(), str(value))

    def radar(self, target: TargetFrame):
        self._data[RADAR_IDX] = (
            RADAR_INDICATOR,
            target.timestamp,
            f"{target.dominant_energy},{target.dominant_distance}",
        )

    async def send(self):
        _p = pack(self._data)
        await mesh().async_send_data(GATEWAY_NODE_ID, payload=_p)


_sensor_data = SensorData()


def data():
    global _sensor_data
    return _sensor_data
