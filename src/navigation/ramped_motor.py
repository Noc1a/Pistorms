import math


## RampedMotor: wraps a PiStormsMotor with rate-limited setSpeed so callers
#  cannot apply a sudden voltage step that would brown out the Pi.
#  Maintains an internal "current speed" and clamps each setSpeed delta to
#  max_step_acc (when growing magnitude) or max_step_dec (when shrinking).
#  brake() and reset() zero the internal state. pos() passes through.
#  For a synchronised pair, ramp each wheel with advance() and send both
#  results through the wrapped motor: left.motor.setSpeedSyncPair(l, r).
class RampedMotor():
    # ACC limits per-tick growth (voltage-safe soft start). DEC is loose because
    # deceleration is regenerative — feeds current back into the battery, no spike.
    # A loose DEC lets controller-level ramps (Manual base/diff) reach the wheels
    # without the wrapper double-clamping, which would cause asymmetric wheel
    # arrivals and continued curving after the user releases a turn key.
    MAX_STEP_ACC = 8
    MAX_STEP_DEC = 100
    # Motors have a static-friction deadband roughly 10-15% duty cycle. Below
    # that, the coil generates too little torque to overcome the gearbox and
    # the wheel sits still. Ramping from 0 sends useless low-power commands
    # for the first few ticks — and if the two motors have slightly different
    # deadbands, one wheel visibly starts before the other. Kicking past the
    # deadband on start makes both wheels engage together.
    MIN_START = 18

    def __init__(self, motor, max_step_acc=None, max_step_dec=None):
        self.motor = motor
        self._cur = 0.0
        self._acc = max_step_acc if max_step_acc is not None else self.MAX_STEP_ACC
        self._dec = max_step_dec if max_step_dec is not None else self.MAX_STEP_DEC

    def advance(self, target):
        """Update _cur toward target with ACC/DEC ramping. Returns int(cur).
        Does NOT write to the motor — caller is responsible for issuing the
        write (allows atomic dual-motor sync via bank-level 'S' command).
        Reversal is handled by DEC (loose) ramping through zero; no explicit
        brake is issued because that would fight the subsequent sync-write."""
        target = float(target)
        reversing = self._cur != 0.0 and target * self._cur < 0.0
        step = self._dec if (reversing or abs(target) < abs(self._cur)) else self._acc
        delta = target - self._cur
        if abs(delta) <= step:
            self._cur = target
        else:
            self._cur += math.copysign(step, delta)
        # Enforce deadband floor: any non-zero command below MIN_START is wasted
        # (motor coil can't overcome static friction). This makes both wheels
        # engage together on start AND on direction reversal — without it, a
        # snap from +50 to -4 sends -4 to a motor with forward momentum, and the
        # weak reverse torque takes ~1s to actually spin the wheel backwards.
        # Trade-off: below-MIN_START targets round up, so slow turning is
        # coarser — but below the deadband the wheel wouldn't move anyway.
        if 0 < abs(self._cur) < self.MIN_START:
            self._cur = math.copysign(self.MIN_START, self._cur)
        return int(self._cur)

    def setSpeed(self, target):
        # PiStormsMotor handles the tacho-setpoint reset internally on
        # direction change (via _needs_reset), so we don't need to do it here.
        self.motor.setSpeed(self.advance(target))

    def brake(self, *args, **kwargs):
        self._cur = 0.0
        self.motor.brake(*args, **kwargs)

    def brakeSync(self, *args, **kwargs):
        self._cur = 0.0
        self.motor.brakeSync(*args, **kwargs)

    def reset(self):
        self._cur = 0.0

    def current_speed(self):
        return self._cur

    def pos(self):
        return self.motor.pos()

    def runDegs(self, *args, **kwargs):
        return self.motor.runDegs(*args, **kwargs)
