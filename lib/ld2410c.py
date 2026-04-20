"""
LD2410C-P Radar Driver for MicroPython
=======================================
24 GHz mmWave presence & motion radar — chicken coop security layer

Protocol summary (UART, 256000 baud, 8N1)
------------------------------------------
  Data frame  : FD FC FB FA | len(2LE) | AA | payload | 55 00 | 04 03 02 01
  Command frame: FD FC FB FA | len(2LE) | cmd(2LE) | value | 04 03 02 01
  ACK frame   : FD FC FB FA | len(2LE) | cmd|0x01(2LE) | status(2) | 04 03 02 01
"""

from asyncio import sleep_ms
from time import ticks_ms, ticks_diff, ticks_add
from machine import UART, Pin

# ── Frame constants ────────────────────────────────────────────────────────────
# Data reporting frames  (sensor → MCU)  use F4/F8 magic bytes
_DATA_HEADER = b"\xf4\xf3\xf2\xf1"
_DATA_FOOTER = b"\xf8\xf7\xf6\xf5"
# Command / ACK frames   (MCU → sensor)  use FD/04 magic bytes
_CMD_HEADER = b"\xfd\xfc\xfb\xfa"
_CMD_FOOTER = b"\x04\x03\x02\x01"

# ── Target state flags ─────────────────────────────────────────────────────────
TARGET_NONE = 0x00
TARGET_MOVING = 0x01
TARGET_STATIONARY = 0x02
TARGET_BOTH = 0x03

# ── Command words ──────────────────────────────────────────────────────────────
CMD_ENABLE_CONFIG = 0xFF00
CMD_DISABLE_CONFIG = 0xFE00
CMD_SET_MAX_GATE = 0x6000
CMD_READ_PARAMS = 0x6100
CMD_ENABLE_ENG_MODE = 0x6200
CMD_DISABLE_ENG_MODE = 0x6300
CMD_SET_GATE_SENS = 0x6400
CMD_READ_VERSION = 0xA000
CMD_FACTORY_RESET = 0xA200
CMD_RESTART = 0xA300


# ══════════════════════════════════════════════════════════════════════════════
class TargetFrame:
    """One parsed radar snapshot."""

    __slots__ = (
        "timestamp",
        "state",
        "move_dist",
        "move_energy",
        "stat_dist",
        "stat_energy",
        "detect_dist",
        "gate_move_energy",
        "gate_stat_energy",  # engineering mode lists
    )

    def __init__(self):
        self.timestamp = 0
        self.state = TARGET_NONE
        self.move_dist = 0  # cm
        self.move_energy = 0  # 0-100
        self.stat_dist = 0  # cm
        self.stat_energy = 0  # 0-100
        self.detect_dist = 0  # cm  (max of both targets)
        self.gate_move_energy = []  # per-gate energies (eng mode)
        self.gate_stat_energy = []

    # ── Convenience properties ─────────────────────────────────────────────────
    @property
    def has_moving(self):
        return bool(self.state & TARGET_MOVING)

    @property
    def has_stationary(self):
        return bool(self.state & TARGET_STATIONARY)

    @property
    def has_any(self):
        return self.state != TARGET_NONE

    @property
    def dominant_distance(self):
        """Distance of the highest-energy target."""
        if self.has_moving and self.has_stationary:
            return (
                self.move_dist
                if self.move_energy >= self.stat_energy
                else self.stat_dist
            )
        if self.has_moving:
            return self.move_dist
        if self.has_stationary:
            return self.stat_dist
        return 0

    @property
    def dominant_energy(self):
        return max(self.move_energy, self.stat_energy)

    def state_str(self):
        labels = {
            TARGET_NONE: "none",
            TARGET_MOVING: "moving",
            TARGET_STATIONARY: "stationary",
            TARGET_BOTH: "moving+stationary",
        }
        return labels.get(self.state, "unknown")

    def __repr__(self):
        return (
            f"<TargetFrame t={self.timestamp} state={self.state_str()} "
            f"move={self.move_dist}cm/{self.move_energy}% "
            f"stat={self.stat_dist}cm/{self.stat_energy}%>"
        )


class LD2410C:
    """
    Full driver for the LD2410C-P 24 GHz radar module.

    Parameters
    ----------
    uart_id   : UART bus number (e.g. 1)
    tx_pin    : TX GPIO number
    rx_pin    : RX GPIO number
    presence_pin : optional GPIO wired to OUT pin of sensor (fast interrupt path)
    engineering  : boot into engineering mode (per-gate energies)
    on_target    : optional callback(TargetFrame) called on every new frame
    on_threat    : optional callback(dict signature) when a signature is ready
    """

    # ── Construction ───────────────────────────────────────────────────────────
    def __init__(
        self,
        uart_id: int = 1,
        tx_pin: int = 4,
        rx_pin: int = 5,
        presence_pin: int | None = None,
        engineering: bool = False,
        on_target=None,
        on_threat=None,
    ):
        # ── UART init (ESP32-safe, v1.27+) ────────────────────────────────────────
        # On v1.27 the UART() constructor with tx=/rx= kwargs installs the ESP-IDF
        # driver immediately. Calling .init() on an already-live driver → ESP_FAIL.
        # Fix: always deinit() first (guarded), then init() exactly once.
        self._uart = UART(uart_id, tx=Pin(tx_pin), rx=Pin(rx_pin))

        try:
            self._uart.deinit()  # tears down stale driver, if any
        except Exception:
            pass  # fresh boot: driver not yet installed — fine

        self._uart.init(
            baudrate=256000,
            bits=8,
            parity=None,
            stop=1,
            timeout=100,
            rxbuf=256,
        )

        self._eng_mode = engineering
        self._on_target = on_target
        self._on_threat = on_threat

        # Presence OUT pin (hardware fast-path, optional)
        self._pres_pin = Pin(presence_pin, Pin.IN) if presence_pin is not None else None

        # Internal state
        self._last_frame: TargetFrame | None = None
        self._active_event = False  # are we mid-detection event?
        self._absence_ms = 0  # ms since last target seen
        self._absence_thr = 3000  # ms quiet → event ends

        # Stats
        self.stats = {
            "frames_rx": 0,
            "parse_errors": 0,
            "events": 0,
        }

        self._initialised = False

    # ── Initialisation ─────────────────────────────────────────────────────────
    async def init_sensor(self):
        await sleep_ms(100)
        await self._enter_config()
        if self._eng_mode:
            await self._send_cmd(CMD_ENABLE_ENG_MODE)
        else:
            await self._send_cmd(CMD_DISABLE_ENG_MODE)
        await self._exit_config()
        await sleep_ms(50)
        self._initialised = True

    # ── Public API ─────────────────────────────────────────────────────────────

    def update(self) -> TargetFrame | None:
        """
        Call this in your main loop as fast as possible (~10–20 ms tick).
        Reads any pending UART bytes, parses frames, fires callbacks,
        and manages the signature-collection lifecycle.

        Returns the latest TargetFrame or None if nothing new arrived.
        """
        frame = self._read_frame()
        if frame is None:
            self._tick_absence()
            return None

        self._last_frame = frame
        self.stats["frames_rx"] += 1

        if self._on_target:
            self._on_target(frame)

        self._manage_event(frame)
        return frame

    def flush(self):
        """Drain any stale bytes sitting in the RX buffer."""
        while self._uart.any():
            self._uart.read(256)

    async def warm_up_read(
        self, count: int = 8, interval_ms: int = 100
    ) -> TargetFrame | None:
        """
        Flush stale buffer, then read `count` fresh frames.
        Returns the last valid frame, or None.
        Intended for on-demand (non-continuous) use.
        """
        self.flush()
        last_valid = None
        for _ in range(count):
            frame = self._read_frame()
            if frame is not None:
                self._last_frame = frame
                last_valid = frame
            await sleep_ms(interval_ms)
        return last_valid

    @property
    def last_frame(self) -> TargetFrame | None:
        return self._last_frame

    @property
    def presence(self) -> bool:
        """
        Fast presence check.
        Uses hardware OUT pin if wired, otherwise falls back to last frame.
        """
        if self._pres_pin is not None:
            return bool(self._pres_pin.value())
        return self._last_frame.has_any if self._last_frame else False

    @property
    def is_moving(self) -> bool:
        return self._last_frame.has_moving if self._last_frame else False

    @property
    def distance_cm(self) -> int:
        """Dominant target distance in centimetres, 0 if no target."""
        return self._last_frame.dominant_distance if self._last_frame else 0

    @property
    def energy(self) -> int:
        """Dominant target signal energy 0-100."""
        return self._last_frame.dominant_energy if self._last_frame else 0

    # ── Configuration helpers ──────────────────────────────────────────────────

    async def configure(
        self,
        max_move_gate: int = 8,  # 0-8  (each gate ≈ 0.75 m)
        max_stat_gate: int = 8,
        absence_delay: int = 5,  # seconds sensor reports absence
    ):
        """
        Set detection range gates and unmanned delay.

        Gates translate roughly as: gate × 0.75 m
          gate 2 ≈ 1.5 m,  gate 4 ≈ 3 m,  gate 8 ≈ 6 m
        """
        await self._enter_config()
        payload = bytearray(18)
        payload[0] = 0x00
        payload[1] = 0x00  # cmd value start
        # max moving gate
        payload[0] = max_move_gate & 0xFF
        # max stationary gate
        payload[2] = 0x01
        payload[3] = 0x00
        payload[4] = max_stat_gate & 0xFF
        payload[5] = 0x00
        # absence delay
        payload[6] = 0x02
        payload[7] = 0x00
        payload[8] = absence_delay & 0xFF
        payload[9] = 0x00
        await self._send_cmd(CMD_SET_MAX_GATE, payload[:10])
        await self._exit_config()

    async def set_gate_sensitivity(self, gate: int, move_sens: int, stat_sens: int):
        """
        Set sensitivity for a single gate (0-8).
        Sensitivity range: 0 (most sensitive) – 100.
        """
        await self._enter_config()
        payload = bytearray(6)
        payload[0] = gate & 0xFF
        payload[2] = move_sens & 0xFF
        payload[4] = stat_sens & 0xFF
        await self._send_cmd(CMD_SET_GATE_SENS, payload)
        await self._exit_config()

    async def read_params(self) -> dict:
        """Read and return current sensor configuration."""
        await self._enter_config()
        await self._send_cmd(CMD_READ_PARAMS)
        resp = await self._read_ack(timeout_ms=300)
        await self._exit_config()
        if resp and len(resp) >= 18:
            return {
                "max_move_gate": resp[4],
                "max_stat_gate": resp[6],
                "move_sensitivities": list(resp[8:17]),
                "stat_sensitivities": list(resp[17:26]) if len(resp) >= 26 else [],
                "absence_delay": resp[26] if len(resp) > 26 else 0,
            }
        return {}

    async def read_firmware_version(self) -> str:
        await self._enter_config()
        await self._send_cmd(CMD_READ_VERSION)
        resp = await self._read_ack(timeout_ms=300)
        await self._exit_config()
        if resp and len(resp) >= 12:
            major = resp[10]
            minor = resp[11]
            bug = int.from_bytes(resp[12:16], "little") if len(resp) >= 16 else 0
            return f"{major}.{minor}.{bug}"
        return "unknown"

    async def factory_reset(self):
        await self._enter_config()
        await self._send_cmd(CMD_FACTORY_RESET)
        await self._read_ack()
        await self._exit_config()

    async def restart(self):
        await self._enter_config()
        await self._send_cmd(CMD_RESTART)
        await sleep_ms(1500)

    def set_absence_threshold(self, ms: int):
        """How long after last detection we consider an event over."""
        self._absence_thr = ms

    def _manage_event(self, frame: TargetFrame):
        self._absence_ms = 0  # reset absence timer on any frame

        if frame.has_any:
            if not self._active_event:
                self._active_event = True
                self.stats["events"] += 1

    def _tick_absence(self):
        """Called when update() received no frame; advances absence counter."""
        if not self._active_event:
            return
        self._absence_ms += 20  # approximate tick ~20 ms
        if self._absence_ms >= self._absence_thr:
            self._close_event()

    def _close_event(self):
        self._active_event = False
        self._absence_ms = 0

    # ── Internal: UART framing ─────────────────────────────────────────────────

    def _read_frame(self) -> TargetFrame | None:
        if not self._uart.any():
            return None

        raw = self._uart.read(128)
        if not raw:
            return None

        # Locate data frame header (F4 F3 F2 F1 — NOT the command header FD FC FB FA)
        idx = self._find_bytes(raw, _DATA_HEADER)
        if idx < 0:
            self.stats["parse_errors"] += 1
            return None

        data = raw[idx:]
        if len(data) < 17:  # hdr(4)+len(2)+min_payload(7)+footer(4)
            return None

        length = data[4] | (data[5] << 8)

        # Total frame = 4 (hdr) + 2 (len) + length + 4 (footer)
        total = 4 + 2 + length + 4
        if len(data) < total:
            return None

        payload = data[6 : 6 + length]

        # Validate footer
        footer_pos = 6 + length
        if data[footer_pos : footer_pos + 4] != _DATA_FOOTER:
            self.stats["parse_errors"] += 1
            return None

        return self._parse_data_payload(payload)

    @staticmethod
    def _find_bytes(buf: bytes, magic: bytes) -> int:
        """Return index of first occurrence of 4-byte magic in buf, or -1."""
        end = len(buf) - 3
        for i in range(end):
            if buf[i : i + 4] == magic:
                return i
        return -1

    def _parse_data_payload(self, payload: bytes) -> TargetFrame | None:
        """
        Standard data frame payload layout:
          [0]    0x02  (data type)
          [1]    0xAA  (data head)
          [2]    target state
          [3:5]  moving target distance (LE, cm)
          [5]    moving energy
          [6:8]  stationary target distance (LE, cm)
          [8]    stationary energy
          [9:11] detection distance (LE, cm)
          [11]   0x55  (tail)
          [12]   0x00  (check)

        Engineering mode appends per-gate energies after [12].
        """
        try:
            if len(payload) < 13:
                return None
            if payload[0] != 0x02 or payload[1] != 0xAA:
                return None

            f = TargetFrame()
            f.timestamp = ticks_ms()
            f.state = payload[2]
            f.move_dist = payload[3] | (payload[4] << 8)
            f.move_energy = payload[5]
            f.stat_dist = payload[6] | (payload[7] << 8)
            f.stat_energy = payload[8]
            f.detect_dist = payload[9] | (payload[10] << 8)

            # Engineering mode: gate energies (optional)
            if self._eng_mode and len(payload) > 15:
                eng = payload[13:]  # after 55 00
                n_gates = 9
                if len(eng) >= n_gates * 2:
                    f.gate_move_energy = list(eng[:n_gates])
                    f.gate_stat_energy = list(eng[n_gates : n_gates * 2])

            return f

        except Exception:
            self.stats["parse_errors"] += 1
            return None

    # ── Internal: command framing ──────────────────────────────────────────────

    @staticmethod
    def _build_cmd_frame(cmd_word: int, value: bytes = b"") -> bytes:
        """
        Command frame:
          FD FC FB FA | len(2LE) | cmd_lo cmd_hi | value | 04 03 02 01
        """
        cmd_bytes = bytes([cmd_word & 0xFF, (cmd_word >> 8) & 0xFF])
        inner = cmd_bytes + value
        length = len(inner)
        frame = (
            _CMD_HEADER
            + bytes([length & 0xFF, (length >> 8) & 0xFF])
            + inner
            + _CMD_FOOTER
        )
        return frame

    async def _send_cmd(self, cmd_word: int, value: bytes = b""):
        frame = self._build_cmd_frame(cmd_word, value)
        self._uart.write(frame)
        await sleep_ms(50)

    async def _read_ack(self, timeout_ms: int = 200) -> bytes | None:
        deadline = ticks_add(ticks_ms(), timeout_ms)
        buf = b""
        while ticks_diff(deadline, ticks_ms()) > 0:
            if self._uart.any():
                buf += self._uart.read(64)
            await sleep_ms(10)
        idx = self._find_bytes(buf, _CMD_HEADER)
        if idx < 0:
            return None
        return buf[idx:]

    async def _enter_config(self):
        await self._send_cmd(CMD_ENABLE_CONFIG, b"\x01\x00")
        await sleep_ms(50)

    async def _exit_config(self):
        await self._send_cmd(CMD_DISABLE_CONFIG)
        await sleep_ms(50)


# ══════════════════════════════════════════════════════════════════════════════
#  EXAMPLE USAGE
# ══════════════════════════════════════════════════════════════════════════════
"""
from ld2410c import LD2410C, TARGET_MOVING
from asyncio import sleep_ms

def on_threat(sig):
    print("=== THREAT SIGNATURE ===")
    print(f"  Label      : {sig.get('label', 'unclassified')}")
    print(f"  Duration   : {sig['duration_ms']} ms")
    print(f"  Distance   : {sig['min_dist']}–{sig['max_dist']} cm (mean {sig['mean_dist']})")
    print(f"  Peak energy: {sig['max_energy']}%")
    print(f"  Move ratio : {sig['move_ratio']:.0%}")

radar = LD2410C(
    uart_id      = 1,
    tx_pin       = 4,
    rx_pin       = 5,
    presence_pin = 6,       # optional OUT pin for fast check
    engineering  = True,    # enable per-gate energy (richer signatures)
    on_threat    = on_threat,
)

# Tune detection zone for a small coop (≈3 m max)
radar.configure(
    max_move_gate = 4,    # 4 × 0.75 m ≈ 3 m
    max_stat_gate = 4,
    absence_delay = 5,
)

# Raise sensitivity on near gates, lower on far gates
for gate in range(0, 3):
    radar.set_gate_sensitivity(gate, move_sens=30, stat_sens=40)
for gate in range(3, 5):
    radar.set_gate_sensitivity(gate, move_sens=50, stat_sens=60)

# Mark the event as done after 4 s of silence
radar.set_absence_threshold(4000)

# ── Main loop ────────────────────────────────────────────────────────────────
while True:
    frame = radar.update()          # call every 10-20 ms

    if frame and frame.has_any:
        print(f"[RADAR] {frame}")

    # PIR already fired → cross-check with radar
    # if pir_triggered and radar.presence:
    #     start_recording()

    sleep_ms(15)
"""
