from micropython import const
from power import Battery, TTE

# Constants
V_MAX = const(5.4)
V_NOMINAL = const(5.2)
V_MIN = const(3.6)
AH = const(2.4)
T = const(10 * 60)

battery = Battery(V_MAX, V_NOMINAL, V_MIN, AH)





tte = TTE(battery, T)
