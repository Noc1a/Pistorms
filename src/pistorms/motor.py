#!/usr/bin/env python3
#
# Copyright (c) 2015 mindsensors.com
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

#mindsensors.com invests time and resources providing this open source code,
#please support mindsensors.com  by purchasing products from mindsensors.com!
#Learn more product option visit us @  http://www.mindsensors.com/

import threading
import time

from . import registers
from .mindsensors_i2c import mindsensors_i2c


## PiStormsMotor: one motor port of the PiStorms (psm.BAM1, BAM2, BBM1, BBM2).
#  The board runs the motor itself (speed and encoder control); this class
#  writes its commands and adds acceleration ramps.
#  @code
#  from pistorms import PiStorms
#  psm = PiStorms()
#  psm.BAM1.setSpeedSync(50, acceleration=100)    # both wheels of bank A
#  psm.BAM1.brakeSync(acceleration=100)
#  @endcode
#  Speeds are percentages, -100 to 100: outside this range a ValueError is
#  raised (smbus keeps only the low byte, so -160 would become +96).
class PiStormsMotor():
    STOP_ATTEMPTS = 3   # outer retries for brake/float (each writeByte retries too)

    def __init__(self, bank: mindsensors_i2c, num: int = 0) -> None:
        self.bank: mindsensors_i2c = bank
        self.motornum: int = num
        self.index: int = num - 1    # this motor in its bank: 0 = M1, 1 = M2
        self._own: tuple[int, ...] = (self.index,)   # the motors a single-motor command acts on
        # Per-motor state is stored on the BANK — not on this object — because
        # BAM1 and BAM2 are separate objects that both drive the bank's M1/M2
        # (sync commands write both). Per-object copies drifted apart and
        # broke _needs_reset.
        #   motor_speeds:   last commanded speed of M1, M2 (0 after a
        #                   runSecs/runDegs move, which stops on its own)
        #   motor_commands: command counter of M1, M2 — every public command
        #                   increments it, which stops a ramp still running
        # Every command made of several I2C writes holds bank.lock, so that a
        # command from another thread (e.g. a brake) cannot slip between them.
        if not hasattr(bank, 'motor_speeds'):
            bank.motor_speeds = [0, 0]
            bank.motor_commands = [0, 0]

    @staticmethod
    def _checkSpeed(speed):
        if not -100 <= speed <= 100:
            raise ValueError(f"speed must be between -100 and 100, got {speed}")
        return int(round(speed))      # smbus only accepts ints

    def _cancel(self, indices):
        """Start a new command on motors `indices` (0 = M1, 1 = M2 of the
        bank): a ramp still running on any of them stops at its next step.
        Returns the command numbers of these motors at that moment: a ramp
        keeps them and stops as soon as they change.
        The counters are per bank, so BAM2.brake() also stops a ramp that
        BAM1.setSpeedSync() started."""
        with self.bank.lock:
            for i in indices:
                self.bank.motor_commands[i] += 1
            return [self.bank.motor_commands[i] for i in indices]

    def _setSpeed(self, speeds, acceleration, blocking, write_fn, callback=None, stop_fn=None):
        """Send `speeds` through write_fn: (speed,) for this motor, or
        (speed_m1, speed_m2) for both motors of the bank. With an
        acceleration (percent per second, applied to the motor whose speed
        changes most) the speeds ramp from their current values
        proportionally, so they all arrive together. blocking=False ramps in
        a background thread. A newer command on the same motors stops the
        ramp, and its callback is then not called.
        stop_fn (brake, brakeSync): called instead of the last write of
        speed 0 — writing 0 would float the motor just before braking.
        (The board's own ramp, control bit RAMP, is not used: measured with
        tests/testHWRampUp.py it accelerated like no ramp at all, and once
        the wheels didn't start.)"""
        speeds = [self._checkSpeed(speed) for speed in speeds]
        if acceleration is not None and acceleration <= 0:
            raise ValueError(f"acceleration must be > 0 percent per second, got {acceleration}")
        indices = self._own if len(speeds) == 1 else (0, 1)
        my_commands = self._cancel(indices)
        bank = self.bank

        def is_cancelled():
            # A newer command on these motors has changed their counters
            current_commands = [bank.motor_commands[i] for i in indices]
            return current_commands != my_commands

        def send_final_command():
            # Checked and sent under the bank lock, so that a brake from
            # another thread cannot slip between the check and the write.
            with bank.lock:
                if is_cancelled():
                    return False
                if stop_fn is not None:
                    stop_fn()
                else:
                    write_fn(*speeds)
                return True

        if acceleration is None:
            if send_final_command() and callback:
                callback()
            return

        def ramp():
            start_speeds = [self.getSpeed(i) for i in indices]
            changes = [speeds[n] - start_speeds[n] for n in range(len(speeds))]

            # 1 % per step on the motor whose speed changes most;
            # the other motor follows proportionally, so both arrive together
            nb_steps = int(max(abs(change) for change in changes))
            step_duration = 1.0 / acceleration
            ramp_start = time.monotonic()

            # All the steps but the last; the last one is send_final_command()
            for step in range(1, nb_steps):
                progress = step / nb_steps          # goes from 0 to 1
                step_speeds = [round(start_speeds[n] + changes[n] * progress)
                               for n in range(len(speeds))]
                with bank.lock:
                    if is_cancelled():
                        return          # a newer command took over: stop, no callback
                    write_fn(*step_speeds)  # one or two arguments depending on the motor count
                # Wait until this step's scheduled time, not a fixed delay:
                # the I2C writes take 3–4 ms, which would otherwise add up
                # and make the ramp ~30 % slower than the acceleration asked.
                next_step_time = ramp_start + step * step_duration
                time.sleep(max(0.0, next_step_time - time.monotonic()))

            if send_final_command() and callback:
                callback()

        if blocking:
            ramp()
        else:
            threading.Thread(target=ramp, daemon=True).start()

    def _reg(self, name, index=None):
        """Register PS_<name>_M1 / _M2 of motor `index` of the bank — this
        motor by default. Saves an if/else per method:
            _reg('Speed')     → PS_Speed_M1 for M1, PS_Speed_M2 for M2
            _reg('Speed', 1)  → PS_Speed_M2, whatever this motor is"""
        i = self.index if index is None else index
        return getattr(registers, f"PS_{name}_M{i + 1}")

    def _stop_command(self, cmd, indices):
        """Write a brake/float command, then record the speed of the motors
        `indices` as 0. Stopping the motors must not be lost to a bus glitch,
        so retry harder than ordinary writes and log failures."""
        with self.bank.lock:
            for attempt in range(self.STOP_ATTEMPTS):
                try:
                    self.bank.writeByte(registers.PS_Command, cmd)
                    break
                except OSError as e:
                    print(f"Motor stop command 0x{cmd:02X} failed ({e}) "
                          f"[{attempt + 1}/{self.STOP_ATTEMPTS}]", flush=True)
                    if attempt == self.STOP_ATTEMPTS - 1:
                        raise
                    time.sleep(0.01)
            for i in indices:
                self.bank.motor_speeds[i] = 0

    def pos(self) -> int:
        """Encoder position, in degrees.
        The board doesn't freeze the 4 position bytes while they are read: a
        read that falls exactly when the counter carries (…FF → …00) returns
        the old low byte with the new high bytes, 256 off. Measured with the
        wheels turning: 1 read in ~2000, always +256 with a low byte of 0xFF
        (going forward; going backward it would be -256 with 0x00). Such a
        read is the only one with a low byte of 0xFF or 0x00, so only then
        read again."""
        first_read = self._readPosition()
        low_byte = first_read & 0xFF
        if low_byte not in (0x00, 0xFF):
            return first_read
        second_read = self._readPosition()
        if abs(second_read - first_read) < 128:
            return second_read
        # One of the two reads is 256 off: a third read settles it
        third_read = self._readPosition()
        return sorted([first_read, second_read, third_read])[1]

    def _readPosition(self) -> int:
        return self.bank.readLongSigned(self._reg('Position'))
    def resetPos(self):
        if self.index == 0:
            reset_command = registers.r
        else:
            reset_command = registers.s
        self.bank.writeByte(registers.PS_Command, reset_command)

    # ── firmware tacho-setpoint reset ────────────────────────────────────────
    # The PiStorms firmware keeps a latched tacho setpoint from the previous
    # move. When a fresh speed cmd is issued in a new direction (from stop OR
    # reversing sign), the position PID briefly pulls the wheel toward that
    # stale target before the speed loop wins — the "~1 s wrong-way twitch".
    # Writing SetPoint with TACHO+RELATIVE+GO (a runDegs(0) burst) arms the PID
    # with target = current position, effectively clearing the stale state.
    # SPEED-only writes to Speed register don't clear it — the TACHO bit must
    # be exercised. Once cleared, the actual setSpeed / sync-fire overrides.
    @staticmethod
    def _needs_reset(prev, new):
        if new == 0:
            return False
        return prev == 0 or prev * new < 0

    def _reset_setpoint(self, index):
        ctrl = (registers.PS_CONTROL_SPEED | registers.PS_CONTROL_TACHO
                | registers.PS_CONTROL_RELATIVE | registers.PS_CONTROL_GO)
        self.bank.writeArray(self._reg('SetPoint', index), [0, 0, 0, 0, 100, 0, 0, ctrl])

    # ── immediate writes, used by the public commands and their ramps ──────
    def _writeSpeed(self, speed):
        """Run this motor at `speed` now (0 = float)."""
        if speed == 0:
            self._floatNow()
            return
        ctrl = registers.PS_CONTROL_SPEED | registers.PS_CONTROL_GO
        with self.bank.lock:
            if self._needs_reset(self.getSpeed(), speed):
                self._reset_setpoint(self.index)
            self.bank.writeArray(self._reg('Speed'), [speed, 0, 0, ctrl])
            self.bank.motor_speeds[self.index] = speed

    def _writeSpeedPair(self, speed_m1, speed_m2):
        """Write both motor speeds and fire them together with 'S'.
        Bypasses PS_CONTROL_GO so neither motor starts until the 'S' command,
        eliminating the I2C-transaction delay between the two writes that
        otherwise makes one wheel start visibly later than the other."""
        speeds = (speed_m1, speed_m2)
        ctrl = registers.PS_CONTROL_SPEED
        with self.bank.lock:
            for i, speed in enumerate(speeds):
                if self._needs_reset(self.getSpeed(i), speed):
                    self._reset_setpoint(i)
            for i, speed in enumerate(speeds):
                self.bank.writeArray(self._reg('Speed', i), [speed, 0, 0, ctrl])
                self.bank.motor_speeds[i] = speed
            self.bank.writeByte(registers.PS_Command, registers.S)

    def _brakeNow(self):
        if self.index == 0:
            brake_command = registers.A    # 'A' = brake motor 1
        else:
            brake_command = registers.B    # 'B' = brake motor 2
        self._stop_command(brake_command, self._own)

    def _floatNow(self):
        if self.index == 0:
            float_command = registers.a    # 'a' = float motor 1
        else:
            float_command = registers.b    # 'b' = float motor 2
        self._stop_command(float_command, self._own)

    def _brakeSyncNow(self):
        self._stop_command(registers.C, (0, 1))   # 'C' = brake both motors

    # ── public commands ─────────────────────────────────────────────────────
    # The speed commands take the same optional arguments:
    #   acceleration  percent per second (of the motor whose speed changes
    #                 most); None = change the speed at once
    #   blocking      True: return when the speed is reached; False: ramp in
    #                 a background thread and return at once
    #   callback      function called when the speed is reached (not called
    #                 if a newer command on the same motors stopped the ramp)

    ## Last commanded speed of motor `index` of the bank (0 = M1, 1 = M2) — this motor by default
    def getSpeed(self, index=None):
        return self.bank.motor_speeds[self.index if index is None else index]

    ## Run this motor at `speed` for an unlimited duration
    def setSpeed(self, speed, acceleration=None, blocking=True, callback=None):
        self._setSpeed((speed,), acceleration, blocking, self._writeSpeed, callback)

    ## Run both motors of the bank at `speed`, started at the same instant
    #  (call it on either motor of the bank)
    def setSpeedSync(self, speed, acceleration=None, blocking=True, callback=None):
        self._setSpeed((speed, speed), acceleration, blocking, self._writeSpeedPair, callback)

    ## Run both motors of the bank at opposite speeds (M1 at speed, M2 at
    #  -speed), started at the same instant — e.g. a pivot turn
    def setSpeedSyncOpposite(self, speed, acceleration=None, blocking=True, callback=None):
        self._setSpeed((speed, -speed), acceleration, blocking, self._writeSpeedPair, callback)

    ## Run both motors of the bank at two speeds, started at the same instant
    #  (e.g. driving along an arc). With an acceleration both are ramped
    #  proportionally, so they keep the same ratio and arrive together.
    def setSpeedSyncPair(self, speed_m1, speed_m2, acceleration=None, blocking=True, callback=None):
        self._setSpeed((speed_m1, speed_m2), acceleration, blocking, self._writeSpeedPair, callback)

    ## Stop this motor with the brake (shaft hard to turn); with an
    #  acceleration, slow down first, then brake
    def brake(self, acceleration=None, blocking=True):
        self._setSpeed((0,), acceleration, blocking, self._writeSpeed, stop_fn=self._brakeNow)

    ## Stop both motors of the bank with the brake, at the same instant
    def brakeSync(self, acceleration=None, blocking=True):
        self._setSpeed((0, 0), acceleration, blocking, self._writeSpeedPair, stop_fn=self._brakeSyncNow)

    ## Stop this motor and let it turn freely
    def float(self):
        self._cancel(self._own)
        self._floatNow()

    ## Stop both motors of the bank and let them turn freely
    def floatSync(self):
        self._cancel((0, 1))
        self._stop_command(registers.c, (0, 1))   # 'c' = float both motors

    ## Stop this motor and hold its position against external force
    def hold(self):
        self._cancel(self._own)
        # Target = 0 degrees RELATIVE to the current position, i.e. stay here,
        # and keep holding it (ON). Without RELATIVE the target would be the
        # ABSOLUTE encoder position 0 and the motor would drive back to it.
        ctrl = registers.PS_CONTROL_BRK | \
               registers.PS_CONTROL_ON | \
               registers.PS_CONTROL_TACHO | \
               registers.PS_CONTROL_RELATIVE | \
               registers.PS_CONTROL_GO
        with self.bank.lock:
            self.bank.writeArray(self._reg('SetPoint'), [0, 0, 0, 0])
            self.bank.writeByte(self._reg('CMDA'), ctrl)
            self.bank.motor_speeds[self.index] = 0
    ## Run this motor at `speed` for `secs` seconds (0–255), then brake
    #  (brakeOnCompletion) or float. Returns at once: see waitUntilNotBusy().
    def runSecs(self, secs, speed, brakeOnCompletion=False):
        speed = self._checkSpeed(speed)
        secs = max(0, min(255, int(round(secs))))    # one byte
        self._cancel(self._own)
        ctrl = registers.PS_CONTROL_SPEED | \
               registers.PS_CONTROL_TIME | \
               registers.PS_CONTROL_GO
        if(brakeOnCompletion):
            ctrl |= registers.PS_CONTROL_BRK
        with self.bank.lock:
            if self._needs_reset(self.getSpeed(), speed):
                self._reset_setpoint(self.index)
            self.bank.writeArray(self._reg('Speed'), [speed, secs, 0, ctrl])
            # The motor stops on its own at the end: record 0, so that the
            # next command re-arms the setpoint and a ramp starts from 0.
            self.bank.motor_speeds[self.index] = 0
    def status(self):
        return self.bank.readByte(self._reg('Status'))
    def statusBit(self, bitno=0):
        return ((self.status() >> bitno) & 1) == 1
    # Bits of the motor status register (PiStorms Advanced Development Guide)
    STATUS_SPEED_CONTROL    = 0   # running in speed mode
    STATUS_RAMPING          = 1
    STATUS_POWERED          = 2   # powered, not necessarily moving
    STATUS_POSITION_CONTROL = 3   # moving to an encoder target (runDegs) or holding it
    STATUS_BRAKE            = 4   # 1 = brake mode, 0 = float
    STATUS_OVERLOADED       = 5
    STATUS_TIMED            = 6   # running a timed move (runSecs)
    STATUS_STALLED          = 7

    def isBusy(self):
        # Note: a motor started with setSpeed (unlimited duration) keeps the
        # speed-control bit set, so it stays busy until it is stopped.
        busy_bits = (self.STATUS_SPEED_CONTROL, self.STATUS_RAMPING,
                     self.STATUS_POSITION_CONTROL, self.STATUS_TIMED)
        for bit in busy_bits:
            if self.statusBit(bit):
                return True
        return False
    def waitUntilNotBusy(self, timeout=-1):
        """Wait until the motor has finished its move (runSecs, runDegs).
        timeout is in steps of 10 ms; -1 waits forever. Returns 0 when the
        motor is done, 1 on timeout."""
        steps_waited = 0
        while self.isBusy():
            if timeout >= 0 and steps_waited >= timeout:
                return 1
            time.sleep(0.01)
            steps_waited += 1
        return 0
    def isStalled(self):
        return self.statusBit(self.STATUS_STALLED)
    def isOverloaded(self):
        return self.statusBit(self.STATUS_OVERLOADED)
    ## Turn this motor by `degs` degrees (negative = backward) at `speed`.
    #  holdOnCompletion: keep holding the reached position against external
    #  force (control bit ON, "encoder active feedback" in the guide).
    #  Returns at once: see waitUntilNotBusy().
    def runDegs(self, degs, speed=100, brakeOnCompletion=False, holdOnCompletion=False):
        # smbus only accepts ints — a float here used to raise inside the
        # (formerly silent) writeArray, so the command was dropped.
        degs  = int(round(degs))
        speed = self._checkSpeed(speed)
        self._cancel(self._own)
        ctrl = registers.PS_CONTROL_SPEED | \
               registers.PS_CONTROL_TACHO | \
               registers.PS_CONTROL_RELATIVE | \
               registers.PS_CONTROL_GO
        if(brakeOnCompletion):
            ctrl |= registers.PS_CONTROL_BRK
        if(holdOnCompletion):
            ctrl |= registers.PS_CONTROL_ON
        # Encoder target: 4 bytes, least significant first, negative values
        # in two's complement (the old manual split gave b4 = -1 for -10°)
        degs_bytes = list(degs.to_bytes(4, 'little', signed=True))
        array = degs_bytes + [speed, 0, 0, ctrl]
        with self.bank.lock:
            self.bank.writeArray(self._reg('SetPoint'), array)
            # The motor stops on its own at the target: record 0 (see runSecs)
            self.bank.motor_speeds[self.index] = 0
    ## Reads the values of the PID control registers
    def ReadPerformanceParameters(self):
        try:
            b0 = self.bank.readInteger(registers.PS_P_Kp)
            b1 = self.bank.readInteger(registers.PS_P_Ki)
            b2 = self.bank.readInteger(registers.PS_P_Kd)
            b3 = self.bank.readInteger(registers.PS_S_Kp)
            b4 = self.bank.readInteger(registers.PS_S_Ki)
            b5 = self.bank.readInteger(registers.PS_S_Kd)
            b6 = self.bank.readByte(registers.PS_PassCount)
            b7 = self.bank.readByte(registers.PS_PassTolerance)
            return [b0, b1, b2, b3, b4, b5, b6, b7]
        except OSError:
            print ("Error: Could not read PID values")
            return []
    def SetPerformanceParameters(self, Kp_tacho, Ki_tacho, Kd_tacho, Kp_speed, Ki_speed, Kd_speed, passcount, tolerance): # untested
        Kp_t1 = int(Kp_tacho%0x100)
        Kp_t2 = int(Kp_tacho/0x100)
        Ki_t1 = int(Ki_tacho%0x100)
        Ki_t2 = int(Ki_tacho/0x100)
        Kd_t1 = int(Kd_tacho%0x100)
        Kd_t2 = int(Kd_tacho/0x100)
        Kp_s1 = int(Kp_speed%0x100)
        Kp_s2 = int(Kp_speed/0x100)
        Ki_s1 = int(Ki_speed%0x100)
        Ki_s2 = int(Ki_speed/0x100)
        Kd_s1 = int(Kd_speed%0x100)
        Kd_s2 = int(Kd_speed/0x100)
        passcount = passcount
        tolerance = tolerance
        array = [Kp_t1, Kp_t2, Ki_t1, Ki_t2, \
                 Kd_t1, Kd_t2, Kp_s1, Kp_s2, \
                 Ki_s1, Ki_s2, Kd_s1, Kd_s2, \
                 passcount, tolerance]
        self.bank.writeArray(registers.PS_P_Kp, array)
