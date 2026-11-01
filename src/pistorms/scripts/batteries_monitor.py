#!/usr/bin/env python3

#   Run as a module (systemd service):  python -m pistorms.scripts.batteries_monitor

import time
import subprocess
import threading

from pistorms import PiStorms

# Initialize PiStorms without resetting motors/encoders or touching the
# screen — this service may (re)start while a robot program is running.
psm = PiStorms(reset=False, screen=False)

# Fixed voltage threshold
SAFE_VOLTAGE = 6.0  # volts for your NiMH pack
CHECK_INTERVAL = 60  # seconds
# A reading below this is a bus glitch, not a real battery (the Pi would
# already be off). Ignored like a failed read.
MIN_PLAUSIBLE_VOLTAGE = 3.0
# Shut down only after this many consecutive valid low readings, so one bad
# sample can't power off the robot mid-run.
LOW_READINGS_TO_SHUTDOWN = 3
LOW_RECHECK_INTERVAL = 5  # seconds between confirmation readings

def blink_led():
    while True:
        psm.led(1, 0, 0, 255)  # Turn on LED
        psm.led(2, 0, 0, 255)
        time.sleep(0.5)
        psm.led(1, 0, 0, 0)  # Turn off LED
        psm.led(2, 0, 0, 0)
        time.sleep(0.5)



def main():
    global CHECK_INTERVAL
    blink_thread = None
    low_count = 0

    while True:
        voltage = psm.battVoltage()

        if voltage is None or voltage < MIN_PLAUSIBLE_VOLTAGE:
            print(f"Ignoring invalid battery reading: {voltage}")
            time.sleep(LOW_RECHECK_INTERVAL)
            continue

        if voltage < SAFE_VOLTAGE:
            low_count += 1
            print(f"Battery low: {voltage:.2f} V ({low_count}/{LOW_READINGS_TO_SHUTDOWN})")
            if low_count >= LOW_READINGS_TO_SHUTDOWN:
                print("Battery too low! Shutting down...")
                psm.Shutdown()
                break
            time.sleep(LOW_RECHECK_INTERVAL)
            continue
        low_count = 0

        if voltage < SAFE_VOLTAGE + 0.4:
            if blink_thread is None or not blink_thread.is_alive():
                blink_thread = threading.Thread(target=blink_led, daemon=True)
                blink_thread.start()
            CHECK_INTERVAL = 10  # Check more frequently when in warning range

        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
