import gc

from core import start, task, get_onboard_led, stop
from core.comms.mesh import mesh_callback, mesh
from core.io.NeoLED import NeoLed, RED
from lib.detect import Monitor

from lib.interface.button import get_button
from lib.interface.buzzer import get_buzzer

from lib.power import get_tte, T_MEASUREMENT_INTERVAL
from lib.interface.light import get_light
from lib.data import data
from lib.sensors.photocell import get_photocell
from lib.sensors.pir import get_pir
from lib.sensors.radar import get_radar
import lib.test


DATA_UPDATE_INTERVAL = "10s"

neo = NeoLed(get_onboard_led()[1])


async def _test_fail():
    global neo
    neo.set_color(RED)
    await neo.async_blink()
    neo.on()
    stop()


@task(0, boot=True)
async def init():
    get_tte()
    light = get_light()

    await get_radar().init()

    get_pir()
    get_photocell()
    get_button()
    get_buzzer()

    light.start_pulse()

    success = True

    if not await lib.test.power_test():
        success = False
    elif not await lib.test.sensor_tests():
        success = False
    elif not await lib.test.button_test():
        success = False

    await light.stop_pulse()

    if not success:
        await _test_fail()
        return

    await get_light().off()
    await get_buzzer().beep()

    global neo
    del neo
    del lib.test

    # Free boot-time allocations before the mesh and monitor start
    gc.collect()

    mesh().rx_enable()


@task(0, parallel=True, onetime=True)
async def start_mon():
    get_button().register_irq()
    m = Monitor()
    await m.init()


@task(DATA_UPDATE_INTERVAL)
async def send_update():
    print("Update")
    await data().send()


@task(T_MEASUREMENT_INTERVAL)
async def power():
    tte = get_tte()
    dat = data()
    await tte.sample()
    dat.tte(tte.ltr())
    v = await tte.get_voltage()
    dat.voltage(v)


@mesh_callback()
async def mesh_callback(host, msg):
    print(msg)


start()
