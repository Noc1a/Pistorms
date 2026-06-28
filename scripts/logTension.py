#!/usr/bin/env python3
# log_voltage.py

import time
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'sys'))
from PiStorms import PiStorms

LOGFILE = "/home/robot/voltage_log.txt"
INTERVAL = 5  # secondes

psm = PiStorms()

with open(LOGFILE, "a", buffering=1) as f:  # buffering=1 = line buffered
    f.write(f"=== Log démarré {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")
    while True:
        voltage = psm.battVoltage()
        line = f"{time.strftime('%H:%M:%S')}  {voltage:.3f} V\n"
        f.write(line)
        print(line, end="")
        time.sleep(INTERVAL)