import time
import asyncio
from micropython import const

from core.logging import logger
from core.queue import RingBuffer
from core import add_task
from lib.data import data

from lib.interface.light import get_light
from lib.sensors.photocell import get_photocell
from lib.sensors.pir import get_pir
from lib.sensors.radar import get_radar
import lib.threat_assessment as ta

MOTION_CHECK_INTERVAL = const("4s")
MAX_SAMPLE_BUFFER_LENGTH = const(10)


class Monitor:
    __slots__ = ("_motion_flag", "_buf", "threat_score")

    def __init__(self):
        self._motion_flag = False
        # Each entry is (confidence, timestamp) stored as a plain 2-tuple.
        self._buf = RingBuffer(MAX_SAMPLE_BUFFER_LENGTH, True)
        self.threat_score = 0.0

    async def init(self):
        pir = get_pir()
        pir.register_callback(self._on_pir_irq)
        pir.register_interrupt()
        add_task(self.motion_check, MOTION_CHECK_INTERVAL)
        await asyncio.sleep_ms(0)

    def _on_pir_irq(self):
        self._motion_flag = True

    def _samples(self):
        return [e[0] for e in self._buf]

    def _timestamps(self):
        return [e[1] for e in self._buf]

    async def motion_check(self):
        now = time.ticks_ms() // 1000

        # clear all data that is oder then 10s
        if self._buf:
            gap = now - self._buf.peek_latest()[1]
            if gap > 10:
                self._buf.clear()

        if not self._motion_flag:
            return

        data().pir(get_pir().value())
        self._motion_flag = False

        try:
            # update the data
            confidence = await get_radar().check_radar()
            light = await get_photocell().update()

            self._buf.put((confidence, now))

            # decide
            score, threshold, phase = ta.calculate_score(
                self._samples(),
                self._timestamps(),
                light,
            )

            data().threat(score, threshold, phase)
            logger().data(
                "", f"{get_pir().value()},{confidence},{light},{score},{threshold}"
            )
            """
            get_csv().write_row(
                {
                    "time": now,
                    "conf": confidence,
                    "pir": get_pir().value(),
                    "phc": light,
                }
            )
            """

            # act on it

            if score > threshold * 0.5 and phase == "NIGHT":
                await get_light().turn_on_for(100)

            if score > threshold:
                print("PRE-ALARM")

            if score > threshold * 1.5:
                print("ALARM")

            await get_pir().wait_for_pir_cooldown()

        finally:
            self._motion_flag = False
