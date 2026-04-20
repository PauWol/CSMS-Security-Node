from machine import reset, disable_irq, Pin

from time import sleep_ms

from constants import IF_REBOOT_PIN


def reboot_task():
    # used to guard for re-entry
    disable_irq()

    # give some time
    sleep_ms(50)

    reset()


def register_reboot_btn():
    pin = Pin(IF_REBOOT_PIN, Pin.IN, Pin.PULL_UP)
    return pin.irq(Pin.IRQ_FALLING, handler=reboot_task)
