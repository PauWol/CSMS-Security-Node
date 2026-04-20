from micropython import const

from core.queue import RingBuffer
from core.io import VoltageDivider

from constants import SYS_BATTERY_VOLTAGE_PIN

INFLECTION_POINT_NIMH_AVERAGE = const(0.2)
MAX_VOLTAGE_BUFFER_SIZE = const(50)

R1 = const(1_000)
R2 = const(3_00)


# Dataclass (slotted class) is used to lower ram usage with cpu time trade-off as this is a none time critical calculation
class Battery:
    __slots__ = ('V_max','V_nominal','V_min','ah')

    def __init__(self, v_max:float, v_nominal:float, v_min:float, ah:float):
        self.V_max = v_max
        self.V_nominal = v_nominal
        self.V_min = v_min
        self.ah = ah

class TTE(VoltageDivider):
    """
    TTE – Time Till End

    This class is used to estimate the time till a battery system reaches it's  critical low point.
    """
    def __init__(self, battery: Battery, T: float, p_inflection:float = None, v_max_buffer: int = None):
        super().__init__(SYS_BATTERY_VOLTAGE_PIN, R1, R2)

        self.battery = battery
        self.T = T

        _p_inflection = INFLECTION_POINT_NIMH_AVERAGE if p_inflection is None else p_inflection
        self.v_end: float = battery.V_min * (1 + _p_inflection)


        n = MAX_VOLTAGE_BUFFER_SIZE if v_max_buffer is None else v_max_buffer
        self._sample_buffer = RingBuffer(n,True)

        del p_inflection
        del n

    async def sample(self) -> None:
        """
        Take a sample from the battery and write it to the buffer.
        :return:
        """
        s = await self.async_mean_real_voltage()
        self._sample_buffer.put(s)


    def _calc_v_dot(self) -> float:
        dv = float(self._sample_buffer.peek(0)) - float(self._sample_buffer.peek_latest())
        dt = self.T * (len(self._sample_buffer) - 1)
        return dv / dt

    def ltr(self) -> float:
        """
        Calculate the Linear Time Remaining
        :return:
        """
        if len(self._sample_buffer) < 2:
            return 0.0

        return (float(self._sample_buffer.peek_latest()) - self.v_end) / abs(self._calc_v_dot())