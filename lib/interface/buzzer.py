from machine import Pin
import asyncio

from lib.constants import IF_BUZZER_PIN


class Buzzer:
    def __init__(self):
        self.buzzer = Pin(IF_BUZZER_PIN, Pin.OUT)

    def on(self):
        self.buzzer.value(1)

    def off(self):
        self.buzzer.value(0)

    async def beep(self, times: int = 3, delay: float = 0.2):
        """
        Non-blocking beep

        :param times:
        :param delay:
        """
        for _ in range(times):
            self.on()
            await asyncio.sleep(delay)
            self.off()
            await asyncio.sleep(delay)


_buzz = None


def get_buzzer():
    global _buzz

    if _buzz:
        return _buzz

    _buzz = Buzzer()
    return _buzz
