"""The two I2C banks of the PiStorms board, shared by the whole process.

Each bank is created on first use (importing the library opens nothing) and
only once: the per-motor state and the lock are stored on the bank object,
so every motor, sensor port and screen of the process must use the same one.
"""
import threading

from . import registers
from .mindsensors_i2c import mindsensors_i2c

_banks = {}
_creation_lock = threading.Lock()


def _bank(address):
    with _creation_lock:
        if address not in _banks:
            _banks[address] = mindsensors_i2c(address >> 1)
        return _banks[address]


def bank_a():
    """Bank A: motors BAM1/BAM2, ports BAS1/BAS2, buttons, touchscreen."""
    return _bank(registers.PS_A_ADDRESS)


def bank_b():
    """Bank B: motors BBM1/BBM2, ports BBS1/BBS2, battery voltage."""
    return _bank(registers.PS_B_ADDRESS)
