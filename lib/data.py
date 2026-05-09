import struct
from micropython import const
import time

from core.comms.mesh import mesh
from lib.constants import GATEWAY_NODE_ID

# ── Protocol ────────────────────────────────────────────────────────────────
# Byte layout (big-endian):
#   [1] PKT_VERSION
#   [1] flags
#   [4] base_ts  (ticks_ms at send time; all ts_delta values are relative)
#   if FLAG_PIR:    [1] value (0/1)   [2] ts_delta ms
#   if FLAG_PHC:    [2] u16 (0–65535) [2] ts_delta ms
#   if FLAG_RADAR:  [2] distance cm   [2] energy   [2] ts_delta ms
#   if FLAG_THREAT: [4] float32 score
#   if FLAG_SLEEP:  [4] uint32 next sleep ms
#   ── max 26 bytes ──────────────────────────────────────────────────────────

PKT_VERSION = const(1)

FLAG_PIR = const(0x01)
FLAG_PHC = const(0x02)
FLAG_RADAR = const(0x04)
FLAG_THREAT = const(0x08)
FLAG_SLEEP = const(0x10)
FLAG_VOLT = const(0x20)  # u16 millivolts + u16 ts_delta  (4 bytes)
FLAG_TTE = const(0x40)  # u32 seconds to empty           (4 bytes)

_MAX_DELTA = const(65535)
# constants at top
_PHASE_ENC = {"DAY": 0, "DUSK": 1, "NIGHT": 2}
_PHASE_DEC = {0: "DAY", 1: "DUSK", 2: "NIGHT"}


def _delta(ts, base):
    d = time.ticks_diff(ts, base)
    return d if d < _MAX_DELTA else _MAX_DELTA


def pack(
    base_ts,
    flags,
    pir=None,
    phc=None,
    radar=None,
    threat=None,
    sleep_ms=None,
    volt=None,
    tte_s=None,
):
    buf = bytearray(6)
    struct.pack_into(">BBL", buf, 0, PKT_VERSION, flags, base_ts)

    if flags & FLAG_PIR:
        val, ts = pir
        buf.extend(struct.pack(">BH", val, _delta(ts, base_ts)))

    if flags & FLAG_PHC:
        val, ts = phc
        buf.extend(struct.pack(">HH", int(val * 65535), _delta(ts, base_ts)))

    if flags & FLAG_RADAR:
        dist, energy, ts = radar
        buf.extend(struct.pack(">HHH", dist, energy, _delta(ts, base_ts)))

    if flags & FLAG_THREAT:
        score, threshold, phase = threat
        buf.extend(struct.pack(">ffB", score, threshold, _PHASE_ENC[phase]))
    if flags & FLAG_SLEEP:
        buf.extend(struct.pack(">L", sleep_ms))

    if flags & FLAG_VOLT:
        val, ts = volt
        buf.extend(struct.pack(">HH", int(val * 1000), _delta(ts, base_ts)))

    if flags & FLAG_TTE:
        buf.extend(struct.pack(">L", tte_s))

    return buf


def unpack(buf):
    """Gateway-side unpack. Returns a dict of present fields."""
    version, flags, base_ts = struct.unpack_from(">BBL", buf, 0)
    offset = 6
    out = {"version": version, "flags": flags, "base_ts": base_ts}

    if flags & FLAG_PIR:
        val, delta = struct.unpack_from(">BH", buf, offset)
        out["pir"] = {"value": bool(val), "ts": base_ts + delta}
        offset += 3

    if flags & FLAG_PHC:
        raw, delta = struct.unpack_from(">HH", buf, offset)
        out["phc"] = {"value": raw / 65535, "ts": base_ts + delta}
        offset += 4

    if flags & FLAG_RADAR:
        dist, energy, delta = struct.unpack_from(">HHH", buf, offset)
        out["radar"] = {"distance": dist, "energy": energy, "ts": base_ts + delta}
        offset += 6

    if flags & FLAG_THREAT:
        score, threshold, phase = struct.unpack_from(">ffB", buf, offset)
        out["threat"] = {
            "score": score,
            "threshold": threshold,
            "phase": _PHASE_DEC.get(phase, "UNKNOWN"),
        }
        offset += 9

    if flags & FLAG_SLEEP:
        (ms,) = struct.unpack_from(">L", buf, offset)
        out["sleep_ms"] = ms
        offset += 4

    if flags & FLAG_VOLT:
        raw, delta = struct.unpack_from(">HH", buf, offset)
        out["volt"] = {"value": raw / 1000, "ts": base_ts + delta}
        offset += 4

    if flags & FLAG_TTE:
        (s,) = struct.unpack_from(">L", buf, offset)
        out["tte_s"] = s

    return out


# ── SensorData ───────────────────────────────────────────────────────────────


class SensorData:
    __slots__ = ("_pir", "_phc", "_radar", "_threat", "_sleep_ms", "_volt", "_tte_s")

    def __init__(self):
        self._pir = None  # (value: int, ts: int)
        self._phc = None  # (value: float, ts: int)
        self._radar = None  # (distance: int, energy: int, ts: int)
        self._threat = None  # float
        self._sleep_ms = None  # int
        self._volt = None  # (value: float V, ts: int)
        self._tte_s = None  # int seconds to empty

    def pir(self, value):
        """value: 0 or 1"""
        self._pir = (int(bool(value)), time.ticks_ms())

    def photo_cell(self, value):
        """value: float 0.0–1.0"""
        self._phc = (value, time.ticks_ms())

    def radar(self, distance, energy):
        """distance: int cm, energy: int 0–100"""
        self._radar = (int(distance), int(energy), time.ticks_ms())

    def threat(self, score, threshold, phase):
        """score: float, threshold: float, phase: str DAY/DUSK/NIGHT"""
        self._threat = (float(score), float(threshold), phase)

    def sleep_interval(self, ms):
        """Next sleep duration in ms — call when sleep logic is implemented."""
        self._sleep_ms = int(ms)

    def voltage(self, volts):
        """volts: float V (stored as millivolts u16, max ~65.535 V)"""
        self._volt = (float(volts), time.ticks_ms())

    def tte(self, seconds):
        """Time-to-empty in whole seconds (u32)."""
        self._tte_s = int(seconds)

    def _build_flags(self):
        flags = 0
        if self._pir is not None:
            flags |= FLAG_PIR
        if self._phc is not None:
            flags |= FLAG_PHC
        if self._radar is not None:
            flags |= FLAG_RADAR
        if self._threat is not None:
            flags |= FLAG_THREAT
        if self._sleep_ms is not None:
            flags |= FLAG_SLEEP
        if self._volt is not None:
            flags |= FLAG_VOLT
        if self._tte_s is not None:
            flags |= FLAG_TTE
        return flags

    async def send(self):
        flags = self._build_flags()
        if not flags:
            return

        base_ts = time.ticks_ms()
        payload = pack(
            base_ts,
            flags,
            pir=self._pir,
            phc=self._phc,
            radar=self._radar,
            threat=self._threat,
            sleep_ms=self._sleep_ms,
            volt=self._volt,
            tte_s=self._tte_s,
        )

        print("TX %d bytes flags=0x%02x" % (len(payload), flags))
        await mesh().async_send_data(GATEWAY_NODE_ID, payload=payload)

        self._pir = None
        self._phc = None
        self._radar = None
        self._threat = None
        self._sleep_ms = None
        self._volt = None
        self._tte_s = None


_sensor_data = None


def data():
    global _sensor_data

    if _sensor_data:
        return _sensor_data

    _sensor_data = SensorData()
    return _sensor_data
