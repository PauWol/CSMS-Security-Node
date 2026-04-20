from machine import Pin
from core import start, task
import time
import asyncio

from ld2410c import LD2410C

# --- PINS ---
PIR_PIN = 4
BUZZER_PIN = 26
RADAR_RX_PIN = 16
RADAR_TX_PIN = 17

pir = Pin(PIR_PIN, Pin.IN)
buzzer = Pin(BUZZER_PIN, Pin.OUT)
radar = LD2410C(2, RADAR_TX_PIN, RADAR_RX_PIN)

# --- STATE FLAGS ---
motion_flag = False
last_trigger = 0


# --- INTERRUPT ---
def pir_irq(pin):
    global motion_flag, last_trigger

    now = time.ticks_ms()

    # simple debounce (ignore triggers within 2s)
    if time.ticks_diff(now, last_trigger) > 2000:
        motion_flag = True
        last_trigger = now


pir.irq(trigger=Pin.IRQ_RISING, handler=pir_irq)


# --- BUZZER ---
def buzz(duration_ms=1500):
    buzzer.on()
    time.sleep_ms(duration_ms)
    buzzer.off()


@task(None, async_task=True)
async def motion_check():
    global motion_flag

    if not motion_flag:
        await asyncio.sleep(0)
        return

    print("PIR triggered")
    motion_flag = False

    await asyncio.sleep_ms(200)

    radar.update()

    if radar.distance_cm > 0:
        print("✅ Person confirmed")
        buzzer.on()
        await asyncio.sleep_ms(1500)
        buzzer.off()


start()
