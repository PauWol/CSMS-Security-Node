from micropython import const

from core.io import ADC

from constants import (
    V_PHOTO_CELL_NIGHT,
    V_PHOTO_CELL_DAYLIGHT,
    V_PHOTO_CELL_TWILIGHT,
    V_PHOTO_CELL_DAWN_DUSK,
    V_PHOTO_CELL_BRIGHT,
    S_PHOTO_CELL_PIN,
)


class Photo:
    """
    Photoresistor (light sensor) abstraction using an ADC pin.
    Provides normalized readings and simple light-level classification.
    """

    def __init__(self):
        """
        Initialize the Photo sensor.

        :return: None
        """
        self._pin = ADC(S_PHOTO_CELL_PIN)

        self._min_val = const(0)
        self._max_val = const(65535)

        self._last_val: float = 0.0

    def _normalize(self, _value: float):
        return (_value - self._min_val) / (self._max_val - self._min_val)

    async def _get_raw(self) -> float:
        """
        Read and normalize the current sensor value.

        :return: Normalized sensor value (0.0–1.0)
        """
        return self._normalize(await self._pin.async_mean(_type="raw"))

    async def update(self) -> None:
        """
        Update the cached sensor value.
        Must be called once before using the is_... functions.

        :return: None
        """
        self._last_val = await self._get_raw()

    def _mapping(self) -> int:
        """
        Map the normalized value to a discrete light level.

        Levels:
        0 = night
        1 = twilight
        2 = dawn/dusk
        3 = daylight
        4 = bright
        5 = very bright

        :return: Integer light level (0–5)
        """
        n = self._last_val

        if n < V_PHOTO_CELL_NIGHT:
            return 0
        elif n < V_PHOTO_CELL_TWILIGHT:
            return 1
        elif n < V_PHOTO_CELL_DAWN_DUSK:
            return 2
        elif n < V_PHOTO_CELL_DAYLIGHT:
            return 3
        elif n < V_PHOTO_CELL_BRIGHT:
            return 4
        else:
            return 5

    def is_day(self) -> bool:
        return self._mapping() == 3

    def is_night(self) -> bool:
        return self._mapping() == 0

    def is_dawn_dusk(self) -> bool:
        return self._mapping() == 2

    def is_twilight(self) -> bool:
        return self._mapping() == 1
