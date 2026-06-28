#!/usr/bin/env python3

import time
import os
import sys
import subprocess
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sys'))
from PiStorms import PiStorms

# Initialize PiStorms
psm = PiStorms()

# Fixed voltage threshold
SAFE_VOLTAGE = 6.0  # volts for your NiMH pack
CHECK_INTERVAL = 60  # seconds

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

    while True:
        voltage = psm.battVoltage()

        if voltage < SAFE_VOLTAGE:
            print("Battery too low! Shutting down...")
            psm.Shutdown()
            break
        elif voltage < SAFE_VOLTAGE + 0.4:
            if blink_thread is None or not blink_thread.is_alive():
                blink_thread = threading.Thread(target=blink_led, daemon=True)
                blink_thread.start()
            CHECK_INTERVAL = 10  # Check more frequently when in warning range

        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()