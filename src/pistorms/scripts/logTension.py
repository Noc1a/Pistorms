#!/usr/bin/env python3
"""Écrit la tension de la batterie toutes les 5 s, à l'écran et dans
/home/robot/voltage_log.txt. Ctrl-C pour arrêter.

    voltage                                  # depuis n'importe où (voir pyproject.toml)
    python -m pistorms.scripts.logTension
"""
import time

from pistorms import PiStorms

LOGFILE = "/home/robot/voltage_log.txt"
INTERVAL = 5  # secondes


def main() -> None:
    psm = PiStorms(reset=False, screen=False)  # ne pas toucher aux moteurs ni à l'écran d'un programme en cours

    with open(LOGFILE, "a", buffering=1) as f:  # buffering=1 = line buffered
        f.write(f"=== Log démarré {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
        try:
            while True:
                voltage = psm.battVoltage()
                if voltage is None:
                    line = f"{time.strftime('%H:%M:%S')}  erreur de lecture I2C\n"
                else:
                    line = f"{time.strftime('%H:%M:%S')}  {voltage:.3f} V\n"
                f.write(line)
                print(line, end="")
                time.sleep(INTERVAL)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
