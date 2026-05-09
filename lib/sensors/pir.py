from micropython import const
from machine import Pin
import time
import asyncio

from core.logging import logger

from lib.constants import S_PIR_MOTION_PIN

DEBOUNCE_TIME = const(6000)
WHILE_SAFETY_TIMEOUT = const(5000)  # 5 s in ms


class Pir:
    __slots__ = ("pin", "_irq_handler", "_cool_down", "_last_trigger_time")

    def __init__(self):
        self.pin = Pin(S_PIR_MOTION_PIN, Pin.IN)
        self._irq_handler = None
        self._cool_down = False
        self._last_trigger_time = 0

    def _debounce_ok(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_trigger_time) < DEBOUNCE_TIME:
            return False
        self._last_trigger_time = now
        return True

    def cooldown(self, value=True):
        self._cool_down = bool(value)

    def _irq(self, pin):
        if self._cool_down:
            return
        if not self._debounce_ok():
            return
        try:
            if self._irq_handler:
                self._irq_handler()
        except Exception as e:
            logger().error("Pir interrupt error: %s" % e)

    def register_interrupt(self):
        self.pin = Pin(S_PIR_MOTION_PIN, Pin.IN)
        self.pin.irq(self._irq, Pin.IRQ_RISING)

    def register_callback(self, fn):
        self._irq_handler = fn

    async def wait_for_pir_cooldown(self):
        start = time.ticks_ms()
        while self.pin.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start) > WHILE_SAFETY_TIMEOUT:
                break
            await asyncio.sleep_ms(200)

    def value(self):
        return self.pin.value()

    async def is_connected(self):
        pulled = Pin(S_PIR_MOTION_PIN, Pin.IN, Pin.PULL_UP)
        await asyncio.sleep_ms(10)
        v = pulled.value()

        self.pin = Pin(S_PIR_MOTION_PIN, Pin.IN)
        connected = v == 0
        return connected, {"reason": "ok" if connected else "floating_high", "value": v}


_pir = None


def get_pir():
    global _pir
    if _pir:
        return _pir
    _pir = Pir()
    return _pir
