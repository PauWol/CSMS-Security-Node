import time
import asyncio
from machine import Pin
from micropython import const

from core import add_task

from ld2410c import LD2410C, TargetFrame
from photocell import Photo
from data import data

from constants import (
    S_PIR_MOTION_PIN,
    S_RADAR_MOTION_PIN_TX,
    S_RADAR_MOTION_PIN_RX,
    S_RADAR_MOTION_ID_UART,
)

WHILE_SAFETY_TIMEOUT = const(5)  # seconds
MOTION_CHECK_INTERVAL = const("4s")


# On-demand radar poll config
RADAR_WARMUP_FRAMES = const(8)  # discard-then-read this many frames
RADAR_FRAME_INTERVAL = const(100)  # ms between frames
RADAR_CONFIRM_HITS = const(3)  # need this many "real" hits to confirm


class Monitor:
    def __init__(self):
        self._radar: LD2410C | None = None
        self._pir_pin: Pin | None = None
        self._phc_pin: Photo | None = None

        self._motion_flag: bool = False
        self._cool_down: bool = False

        self._motion_counter: int = 0
        self._last_trigger_time: int = 0

        self.debounce_time = 6000

    async def init(self):
        self._radar = LD2410C(
            S_RADAR_MOTION_ID_UART, S_RADAR_MOTION_PIN_TX, S_RADAR_MOTION_PIN_RX
        )
        await self._radar.init_sensor()

        # Init Pir Pin and set interrupt request Pin with handler
        self._pir_pin = Pin(S_PIR_MOTION_PIN, Pin.IN)
        self._pir_pin.irq(self._on_pir_irq, Pin.IRQ_RISING)

        # Init Photocell Pin
        self._phc_pin = Photo()

        add_task(self.motion_check, MOTION_CHECK_INTERVAL)
        await asyncio.sleep_ms(0)

    def _on_pir_irq(self, pin):
        if self._cool_down:
            return

        self._motion_flag = True

    @staticmethod
    def radar_conditions(frame: TargetFrame):
        """
        Returns (detected: bool, conclusive: bool).
        conclusive=False means the frame was garbage/inconclusive — skip it.
        """
        if not frame:
            return False, False

        dist = frame.dominant_distance
        if dist < 20 or dist > 400:
            return False, False  # garbage — don't count

        if frame.has_moving and frame.move_energy > 25:
            return True, True  # confident positive

        if frame.has_stationary and frame.stat_energy > 85:
            return True, True  # confident stationary

        if frame.has_any and frame.dominant_energy > 10:
            return False, True  # conclusive negative

        return False, False  # inconclusive — skip

    async def check_radar(self):
        self._radar.flush()

        hits = 0
        conclusive = 0

        for _ in range(RADAR_WARMUP_FRAMES):
            self._radar.update()
            frame = self._radar.last_frame
            detected, is_conclusive = self.radar_conditions(frame)

            if is_conclusive:
                conclusive += 1
                if detected:
                    hits += 1
            data().radar(frame)
            await asyncio.sleep_ms(RADAR_FRAME_INTERVAL)

        return conclusive >= 4 and hits >= RADAR_CONFIRM_HITS

    async def wait_for_pir_cooldown(self):
        """Block until PIR goes low, or safety timeout expires."""
        start = time.ticks_ms()
        while self._pir_pin.value() == 1:
            # ticks_diff(now, start) is correct: gives elapsed ms
            if time.ticks_diff(time.ticks_ms(), start) > WHILE_SAFETY_TIMEOUT:
                print("[PIR] safety timeout — forcing cooldown exit")
                break
            await asyncio.sleep_ms(200)

    def _debounce_time(self):
        now = time.ticks_ms()

        if time.ticks_diff(now, self._last_trigger_time) < self.debounce_time:
            return False

        self._last_trigger_time = now
        return True

    async def motion_check(self):

        # if pir reports negative return
        if not self._motion_flag:
            return

        data().pir(self._pir_pin.value())
        self._motion_flag = False

        # if to close to other call return
        if not self._debounce_time():
            return

        # cooldown for pir to prevent multiple triggers on event
        self._cool_down = True

        detected = await self.check_radar()

        if not detected:
            self._cool_down = False
            return

        # slot for detected logic

        await self.wait_for_pir_cooldown()
        self._cool_down = False
