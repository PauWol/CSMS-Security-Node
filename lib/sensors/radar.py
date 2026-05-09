from micropython import const
import asyncio

from lib.data import data

from lib.sensors.ld2410c import LD2410C
from lib.constants import (
    S_RADAR_MOTION_ID_UART,
    S_RADAR_MOTION_PIN_RX,
    S_RADAR_MOTION_PIN_TX,
)

# On-demand radar poll config
RADAR_WARMUP_FRAMES = const(8)  # discard-then-read this many frames
RADAR_FRAME_INTERVAL = const(100)  # ms between frames
RADAR_CONFIRM_HITS = const(3)  # need this many "real" hits to confirm


class Radar(LD2410C):
    def __init__(self):
        super().__init__(
            S_RADAR_MOTION_ID_UART, S_RADAR_MOTION_PIN_TX, S_RADAR_MOTION_PIN_RX
        )

        self._initialized = False

    async def init(self):
        await self.init_sensor()
        self._initialized = True

    @staticmethod
    def radar_conditions(frame):
        """
        Returns (detected: bool, conclusive: bool).
        conclusive=False means the frame was garbage/inconclusive — skip it.

        :param frame:
        :type frame: TargetFrame
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
        if not self._initialized:
            raise RuntimeWarning("Radar needs to be initialized first!")

        self.flush()

        hits = 0
        conclusive = 0

        for _ in range(RADAR_WARMUP_FRAMES):
            self.update()
            frame = self.last_frame
            detected, is_conclusive = self.radar_conditions(frame)

            if is_conclusive:
                conclusive += 1
                if detected:
                    hits += 1
                    data().radar(frame.dominant_distance, frame.dominant_energy)
            await asyncio.sleep_ms(RADAR_FRAME_INTERVAL)

        if conclusive == 0:
            return 0.0

        confidence = hits / conclusive  # 0.0 → 1.0
        return confidence


_radar = None


def get_radar():
    global _radar

    if _radar:
        return _radar

    _radar = Radar()
    return _radar
