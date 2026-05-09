from machine import Pin, reset
from micropython import const
import time

from core import add_task

from lib.interface.light import get_light
from lib.sensors.photocell import get_photocell
from lib.constants import IF_REBOOT_PIN

LONG_PRESS_MS = const(2000)
DEBOUNCE_MS = const(200)


class Button:
    def __init__(self):
        self.button = Pin(IF_REBOOT_PIN, Pin.IN, Pin.PULL_UP)
        self._press_time = 0
        self._last_irq = 0

    def is_pressed(self):
        return not self.button.value()

    def button_irq(self, pin):
        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_irq) < DEBOUNCE_MS:
            return
        self._last_irq = now

        if pin.value() == 0:
            self._press_time = now
        else:
            duration = time.ticks_diff(now, self._press_time)

            async def task_handle_press():
                await self.handle_press(duration)

            add_task(task_handle_press, interval=0, onetime=True)

    @staticmethod
    async def handle_press(duration):
        if duration > LONG_PRESS_MS:
            print("Long press -> reboot")
            reset()
        else:
            print("Short press -> toggle LED")

            light = get_light()
            if not light.is_on():
                await light.fade_to(await get_photocell().update() * 100)
            else:
                await light.fade_out(0)

    def register_irq(self):
        self.button.irq(
            trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.button_irq
        )


_button = None


def get_button():
    global _button

    if _button:
        return _button

    _button = Button()
    return _button
