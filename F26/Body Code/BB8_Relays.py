#!/usr/bin/env python3
"""Body Pi GPIO relay outputs for BB-8 (active HIGH by default).

REQUIRES external hardware pull-downs / fail-safe relay driver: software cannot
force a GPIO LOW when the Pi is unpowered, crashed, or booting.
"""
import time
from gpiozero import OutputDevice

RELAY_GPIO_PINS = (20, 21)  # BCM numbering; original Body Pi relay wiring
RELAY_ACTIVE_HIGH = True
RELAY_SETTLE_SECONDS = 0.15

class BB8Relays:
    def __init__(self):
        self.outputs = []
        try:
            for pin in RELAY_GPIO_PINS:
                self.outputs.append(OutputDevice(pin, active_high=RELAY_ACTIVE_HIGH, initial_value=False))
        except Exception:
            self.close()
            raise
        self.enabled = False

    def enable(self):
        if not self.enabled:
            for output in self.outputs:
                output.on()
            self.enabled = True
            time.sleep(RELAY_SETTLE_SECONDS)

    def disable(self):
        for output in self.outputs:
            output.off()
        self.enabled = False

    def close(self):
        self.disable()
        for output in self.outputs:
            output.close()
        self.outputs = []
