#!/usr/bin/env python3

#   Run as a module (systemd service):  python -m pistorms.scripts.batteries_monitor
#
# At boot, asks on the screen which battery is in the robot (NiMH or LiPo):
# its type sets the shutdown voltage. Without an answer within
# CHOICE_TIMEOUT seconds, the last choice is kept. Asked once per boot: a
# restart of the service while a robot program runs doesn't take the screen.

import json
import os
import threading
import time

from pistorms import PiStorms
from pistorms.ui.mindsensorsUI import mindsensorsUI

# Initialize PiStorms without resetting motors/encoders or touching the
# screen — this service may (re)start while a robot program is running.
psm = PiStorms(reset=False, screen=False)

# Shutdown voltage per battery type
SAFE_VOLTAGES: dict[str, float] = {
    "NiMH": 6.6,   # volts for your NiMH pack
    "LiPo": 7.0,   # volts for your LiPo pack
}
DEFAULT_BATTERY = "NiMH"
WARNING_MARGIN = 0.4   # LEDs blink below safe voltage + this

# Last choice and the boot it was made at
SETTINGS_PATH = os.path.expanduser("~/.config/pistorms/battery.json")
BOOT_ID_PATH = "/proc/sys/kernel/random/boot_id"
CHOICE_TIMEOUT = 30  # seconds

CHECK_INTERVAL = 60  # seconds
# A reading below this is a bus glitch, not a real battery (the Pi would
# already be off). Ignored like a failed read.
MIN_PLAUSIBLE_VOLTAGE = 3.0
# Shut down only after this many consecutive valid low readings, so one bad
# sample can't power off the robot mid-run.
LOW_READINGS_TO_SHUTDOWN = 3
LOW_RECHECK_INTERVAL = 5  # seconds between confirmation readings

# Screen layout (320×240)
BUTTON_Y = 90
BUTTON_WIDTH = 140
BUTTON_HEIGHT = 80
BUTTON_X: dict[str, int] = {"NiMH": 15, "LiPo": 165}


def blink_led() -> None:
    while True:
        psm.led(1, 0, 0, 255)  # Turn on LED
        psm.led(2, 0, 0, 255)
        time.sleep(0.5)
        psm.led(1, 0, 0, 0)  # Turn off LED
        psm.led(2, 0, 0, 0)
        time.sleep(0.5)


# ── Battery type ────────────────────────────────────────────────────────────

def current_boot_id() -> str:
    with open(BOOT_ID_PATH) as f:
        return f.read().strip()


def load_settings() -> dict:
    try:
        with open(SETTINGS_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_settings(battery: str) -> None:
    os.makedirs(os.path.dirname(SETTINGS_PATH), exist_ok=True)
    with open(SETTINGS_PATH, "w") as f:
        json.dump({"battery": battery, "boot_id": current_boot_id()}, f)


def ask_battery_type(default: str) -> str:
    """Show the two buttons and return the one touched, or default after
    CHOICE_TIMEOUT seconds."""
    screen = mindsensorsUI("Battery")
    screen.drawAutoText("Type de batterie ?", 20, 20, size=28, display=False)
    for battery, x in BUTTON_X.items():
        screen.drawButton(x, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT, text=battery,
                          align="xcenter", font_size=32, display=False)
    deadline = time.monotonic() + CHOICE_TIMEOUT
    shown_seconds = None
    try:
        while time.monotonic() < deadline:
            seconds_left = int(deadline - time.monotonic()) + 1
            if seconds_left != shown_seconds:
                shown_seconds = seconds_left
                screen.fillRect(0, 195, 320, 45, fill=(0, 0, 0), display=False)
                screen.drawAutoText(f"{default} dans {seconds_left} s", 20, 200, size=22)
            for battery, x in BUTTON_X.items():
                if screen.checkButton(x, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT):
                    return battery
            time.sleep(0.05)
        return default
    finally:
        screen.clearScreen()


def battery_type() -> str:
    """Battery type of this boot: asked on the screen the first time."""
    settings = load_settings()
    last_battery = settings.get("battery", DEFAULT_BATTERY)
    if last_battery not in SAFE_VOLTAGES:
        last_battery = DEFAULT_BATTERY
    if settings.get("boot_id") == current_boot_id():
        return last_battery
    try:
        battery = ask_battery_type(last_battery)
    except Exception as e:
        # No screen or touch: keep monitoring with the last choice
        print(f"Could not ask the battery type ({e!r}), using {last_battery}")
        battery = last_battery
    save_settings(battery)
    return battery


# ── Monitoring ──────────────────────────────────────────────────────────────

def main() -> None:
    check_interval = CHECK_INTERVAL
    blink_thread = None
    low_count = 0

    battery = battery_type()
    safe_voltage = SAFE_VOLTAGES[battery]
    print(f"Battery {battery}: shutdown below {safe_voltage:.1f} V")

    while True:
        voltage = psm.battVoltage()

        if voltage is None or voltage < MIN_PLAUSIBLE_VOLTAGE:
            print(f"Ignoring invalid battery reading: {voltage}")
            time.sleep(LOW_RECHECK_INTERVAL)
            continue

        if voltage < safe_voltage:
            low_count += 1
            print(f"Battery low: {voltage:.2f} V ({low_count}/{LOW_READINGS_TO_SHUTDOWN})")
            if low_count >= LOW_READINGS_TO_SHUTDOWN:
                print("Battery too low! Shutting down...")
                psm.Shutdown()
                break
            time.sleep(LOW_RECHECK_INTERVAL)
            continue
        low_count = 0

        if voltage < safe_voltage + WARNING_MARGIN:
            if blink_thread is None or not blink_thread.is_alive():
                blink_thread = threading.Thread(target=blink_led, daemon=True)
                blink_thread.start()
            check_interval = 10  # Check more frequently when in warning range

        time.sleep(check_interval)


if __name__ == "__main__":
    main()
