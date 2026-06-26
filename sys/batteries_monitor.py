#!/usr/bin/env python3

import time
import os
import subprocess
from PiStorms import PiStorms
import threading

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

def shutdown():

    # 1. Flush bash history (best-effort)
    real_user = os.getenv("SUDO_USER") or os.getenv("USER") or "robot"
    try:
        subprocess.run(
            ["sudo", "-u", real_user, "bash", "-c", "history -a"],
            timeout=5
        )
    except Exception as e:
        print(f"History flush failed: {e}")

    # 2. Flush everything to disk
    subprocess.run(["/usr/bin/sync"], timeout=10)
    subprocess.run(["/usr/bin/journalctl", "--flush"], timeout=5)

    # 3. Start PiStorms countdown — everything important is on disk now
    psm.Shutdown()

    # 4. Signal systemd — may or may not finish before power cuts,
    #    but the important data is already safe from steps 1-2
    subprocess.run(["/bin/systemctl", "poweroff"])

def main():
    blink_thread = None

    while True:
        voltage = psm.battVoltage()

        if voltage < SAFE_VOLTAGE:
                    print("Battery too low! Shutting down...")
                    shutdown()
                    break
        elif voltage < SAFE_VOLTAGE + 0.4:  # Start blinking if voltage is below xv (indicating charging)
            if blink_thread is None or not blink_thread.is_alive():
                blink_thread = threading.Thread(target=blink_led, daemon=True)
                blink_thread.start()
                CHECK_INTERVAL = 10  # Check more frequently when in warning range
            

        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()