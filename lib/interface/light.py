import time

from machine import Pin, PWM
import uasyncio as asyncio

from lib.constants import IF_LED_PANEL_PIN


class Light:
    def __init__(self):
        self.pin = PWM(Pin(IF_LED_PANEL_PIN), freq=1000)
        self._brightness = 0

        self.set_brightness(0)

        self.running = False
        self._pulse_task_buf = None
        self._auto_off_task = None

    # -------------------------
    # core setter
    # -------------------------
    def set_brightness(self, percent: float):
        percent = max(0, min(100, percent))
        self._brightness = percent
        duty = int((percent / 100) * 65535)
        self.pin.duty_u16(duty)

    def get_brightness(self):
        return self._brightness

    # -------------------------
    # instant controls (awaitable for consistency)
    # -------------------------
    async def on(self):
        self.set_brightness(100)

    async def off(self):
        self.set_brightness(0)

    # -------------------------
    # async fade
    # -------------------------
    async def fade_to(self, target: float, duration_ms=500, steps=50):
        target = max(0, min(100, target))
        start = int(self._brightness)

        if steps <= 0:
            self.set_brightness(target)
            return

        step_delay = int(duration_ms // steps)

        for i in range(steps):
            t = (i + 1) / steps

            # smoothstep curve (nice visual fade)
            t = t * t * (3 - 2 * t)

            value = start + (target - start) * t
            self.set_brightness(value)

            await asyncio.sleep_ms(step_delay)

    async def fade_in(self, duration_ms=500, steps=50):
        await self.fade_to(100, duration_ms, steps)

    async def fade_out(self, duration_ms=500, steps=50):
        await self.fade_to(0, duration_ms, steps)

    # -------------------------
    # pulse (awaitable loop)
    # -------------------------
    async def pulse(
        self, min_brightness=10, max_brightness=60, period_ms=2000, cycles=None
    ):
        """
        cycles=None → infinite
        cycles=N → run N full pulses
        """

        half = period_ms // 2
        count = 0

        while cycles is None or count < cycles:
            await self.fade_to(max_brightness, half)
            await self.fade_to(min_brightness, half)
            count += 1

    async def _pulse_task(self, max_duration_ms=10000):
        """
        Run a background pulse loop while `self.running` is True.

        Stops when:
        - `self.running` becomes False, or
        - `max_duration_ms` is exceeded (safety timeout).

        Ensures the light is off on exit.
        """
        start = time.ticks_ms()

        while self.running:
            # global timeout check
            if time.ticks_diff(time.ticks_ms(), start) > max_duration_ms:
                break

            await self.pulse(cycles=1)

        await self.off()

    def start_pulse(self):
        """
        Start the pulse task in the background.

        Sets `self.running` and creates an asyncio task.
        """
        self.running = True
        self._pulse_task_buf = asyncio.create_task(self._pulse_task())

    async def stop_pulse(self):
        """
        Stop the pulse task and wait for it to finish.

        Clears the task reference after completion.
        """
        self.running = False
        await self._pulse_task_buf
        self._pulse_task_buf = None

    def is_on(self):
        """
        Return the current state; is brightness 0 (so led off) or higher (on).
        :return:
        """
        return self._brightness != 0

    async def _auto_off(self, delay):
        try:
            await asyncio.sleep(delay)  # seconds
            await self.off()
        except asyncio.CancelledError:
            # timer was reset
            pass

    async def turn_on_for(self, brightness: float = 100, delay: int = 12):
        """
        Turns LED on and keeps it on for `delay` seconds.
        If called again, timer resets.
        """

        # 1. always turn on immediately
        await self.fade_to(brightness)
        self.running = True

        # 2. cancel previous timer if it exists
        if self._auto_off_task:
            self._auto_off_task.cancel()
            try:
                await self._auto_off_task
            except:
                pass

        # 3. start new timer
        self._auto_off_task = asyncio.create_task(self._auto_off(delay))


_light = None


def get_light():
    global _light

    if _light:
        return _light

    _light = Light()
    return _light
