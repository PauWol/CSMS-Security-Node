from micropython import const

from core import add_task
from core.queue import RingBuffer
from core.io import VoltageDivider
from core.config import get_config
from core.constants import (
    POWER_BATTERY_VOLTAGE_MAX,
    POWER_BATTERY_VOLTAGE_NOMINAL,
    POWER_BATTERY_VOLTAGE_CUT_OFF,
    POWER_BATTERY_AH,
    POWER_VOLTAGE_DIVIDER_R1,
    POWER_VOLTAGE_DIVIDER_R2,
)

from lib.constants import SYS_BATTERY_VOLTAGE_PIN

INFLECTION_POINT_Pb_plus_AVERAGE = const(0.08)  # For my battery
MAX_VOLTAGE_BUFFER_SIZE = const(20)
T_MEASUREMENT_INTERVAL = const(30_000)  # 10s
_TTE_UNKNOWN = const(0xFFFFFFFF)


# Dataclass (slotted class) is used to lower ram usage with cpu time trade-off as this is a none time critical calculation
class Battery:
    __slots__ = ("V_max", "V_nominal", "V_min", "ah")

    def __init__(self, v_max: float, v_nominal: float, v_min: float, ah: float):
        self.V_max = v_max
        self.V_nominal = v_nominal
        self.V_min = v_min
        self.ah = ah


class TTE(VoltageDivider):
    """
    TTE – Time Till End

    This class is used to estimate the time till a battery system reaches it's  critical low point.
    """

    def __init__(
        self,
        battery: Battery,
        T: float,
        r1: float,
        r2: float,
        p_inflection: float = None,
        v_max_buffer: int = None,
    ):
        super().__init__(SYS_BATTERY_VOLTAGE_PIN, r1, r2)

        self.battery = battery
        self.T = T
        self._last_t = None

        _p_inflection = (
            INFLECTION_POINT_Pb_plus_AVERAGE if p_inflection is None else p_inflection
        )
        self.v_end: float = battery.V_min * (1 + _p_inflection)

        n = MAX_VOLTAGE_BUFFER_SIZE if v_max_buffer is None else v_max_buffer
        self._sample_buffer = RingBuffer(n, True)

        add_task(self.sample, T_MEASUREMENT_INTERVAL)
        del p_inflection
        del n

    async def sample(self) -> None:
        """
        Take a sample from the battery and write it to the buffer.
        :return:
        """
        s = await self.async_mean_real_voltage()
        self._sample_buffer.put(s)

    async def test_pin(self):
        return await self.async_is_pin_connected()

    def _calc_v_dot(self) -> float:
        n = len(self._sample_buffer)
        if n < 2:
            return 0.0

        dt = self.T
        total = 0.0

        # average slope across all consecutive samples
        for i in range(1, n):
            dv = float(self._sample_buffer.peek(i - 1)) - float(
                self._sample_buffer.peek(i)
            )
            total += dv / dt

        return total / (n - 1)

    async def get_voltage(self):
        return await self.async_mean_real_voltage()

    def ltr(self) -> float:
        if len(self._sample_buffer) < 2:
            return 0.0

        v_now = float(self._sample_buffer.peek_latest())
        v_dot = self._calc_v_dot()

        # critical: avoid division by near-zero
        if abs(v_dot) < 1e-7:
            return _TTE_UNKNOWN

        t = (v_now - self.v_end) / v_dot

        # optional: clamp insane values
        if t < 0:
            return 0.0  # already below cutoff

        if t > 60 * 60 * 24 * 30:  # >30 days
            return _TTE_UNKNOWN

        if self._last_t is None:
            self._last_t = t
        else:
            self._last_t = 0.8 * self._last_t + 0.2 * t

        return self._last_t


def init():
    cfg = get_config()

    v_max = float(cfg.get(POWER_BATTERY_VOLTAGE_MAX))
    v_nominal = float(cfg.get(POWER_BATTERY_VOLTAGE_NOMINAL))
    v_cut_off = float(cfg.get(POWER_BATTERY_VOLTAGE_CUT_OFF))
    ah = float(cfg.get(POWER_BATTERY_AH))

    r1 = float(cfg.get(POWER_VOLTAGE_DIVIDER_R1))
    r2 = float(cfg.get(POWER_VOLTAGE_DIVIDER_R2))
    _batt = Battery(v_max, v_nominal, v_cut_off, ah)

    return TTE(_batt, T_MEASUREMENT_INTERVAL, r1, r2)


_tte = None


def get_tte():
    global _tte

    if _tte:
        return _tte

    _tte = init()
    return _tte
