import asyncio


from core.constants import POWER_BATTERY_VOLTAGE_MAX, POWER_BATTERY_VOLTAGE_CUT_OFF
from core.config import get_config
from core.logging import logger
from lib.interface.button import get_button

from lib.power import get_tte


from lib.sensors.photocell import get_photocell
from lib.sensors.pir import get_pir
from lib.sensors.radar import get_radar


async def power_test():
    tte = get_tte()

    c, r = await tte.test_pin()

    if not c:
        logger().warn(f"Power ADC Pin not connected: {r}")
        return False

    cfg = get_config()
    max_v = cfg.get(POWER_BATTERY_VOLTAGE_MAX)
    min_v = cfg.get(POWER_BATTERY_VOLTAGE_CUT_OFF)

    current_v = await tte.async_mean_real_voltage()

    if not current_v > min_v and not current_v < max_v:
        logger().warn(f"Battery in abnormal range: {current_v}")
        return False

    return True


async def photocell_test():
    phc = get_photocell()

    c, v = await phc.is_connected()

    if not c:
        logger().warn(f"Photocell not connected: {v}")
        return False

    lhg = await phc.update()

    if lhg > 1 or lhg < 0:
        logger().warn(f"Photocell out of range: {lhg}")
        return False

    n_d = 0.1  # normal deviation
    _v = lhg
    for i in range(5):
        v = await phc.update()

        if abs(v - _v) > n_d:
            logger().warn(f"Photocell out of normal deviation: {v},{_v}")
            return False

        _v = v

    return True


async def pir_test():
    pir = get_pir()

    c, v = await pir.is_connected()

    if not c:
        logger().warn(f"PIR pin not connected: {v}")
        return False

    values = []
    transitions = 0

    last = None

    for _ in range(50):
        v = pir.value()

        # 1. Validate output
        if v not in (0, 1):
            logger().warn(f"PIR invalid value: {v}")
            return False

        values.append(v)

        # 2. Count transitions
        if last is not None and v != last:
            transitions += 1

        last = v
        await asyncio.sleep(0.1)

    # 4. Check excessive flickering
    if transitions > 20:
        logger().warn(f"PIR too noisy: {transitions} transitions")
        return False

    return True


async def radar_test():
    rad = get_radar()

    await rad.init()

    rad.flush()

    frames_before = rad.stats["frames_rx"]
    errors_before = rad.stats["parse_errors"]

    # collect a few frames
    for _ in range(10):
        rad.update()
        await asyncio.sleep_ms(50)

    frames_after = rad.stats["frames_rx"]
    errors_after = rad.stats["parse_errors"]

    frames_got = frames_after - frames_before
    errors = errors_after - errors_before

    if frames_got == 0:
        logger().warn("Radar: no frames received")
        return False

    if errors > frames_got * 0.3:
        logger().warn(f"Radar: too many parse errors ({errors}/{frames_got})")
        return False

    return True


async def sensor_tests():

    if not await photocell_test():
        return False

    if not await pir_test():
        return False

    if not await radar_test():
        return False

    return True


async def button_test():
    btn = get_button()

    # 1. Check for stuck pressed at boot
    if btn.is_pressed():
        logger().warn("Button is pressed during boot")
        return False

    # 2. Observe for state changes (no assumption about user interaction)
    initial = btn.is_pressed()
    changed = False

    for _ in range(20):  # ~2 seconds
        if btn.is_pressed() != initial:
            changed = True
            break
        await asyncio.sleep_ms(100)

    # If no change, we don't fail (user may not press it)
    # but we can flag low confidence
    if not changed:
        logger().info("Button: no state change observed")

    return True
