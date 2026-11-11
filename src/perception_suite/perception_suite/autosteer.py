"""
The driver's engage and disengage logic for steer-only autosteer.

    OFF ----- CC up, in D, delta valid -----> AUTOSTEER
    AUTOSTEER -- CC up again -------------> CANCEL --(CANCEL_HOLD_S)--> OFF
    AUTOSTEER -- left D ------------------> CANCEL
    AUTOSTEER -- delta invalid -----------> OFF
    ABORT ------ not in D ----------------> OFF
"""

OFF = 0
LONG = 1
AUTOSTEER = 2
ABORT = 3
CANCEL = 4

MODE_NAMES = {OFF: 'OFF', LONG: 'LONG', AUTOSTEER: 'AUTOSTEER',
              ABORT: 'ABORT', CANCEL: 'CANCEL'}

# R is 7, P 0, N 6. It reads 5 in D on the car.
GEAR_DRIVE = 5


CC_BUTTON_UP = 1

# A press only counts once the switch has been released this long.
PRESS_RELEASE_S = 0.5

# How long CANCEL is held before OFF.
CANCEL_HOLD_S = 2.0


class AutosteerToggle:
    """CDCU control mode, driven by the cruise switch, the gear and the delta."""

    def __init__(self, enabled=True):

        self.enabled = bool(enabled)
        self.mode = OFF
        self._button_up = False
        self._released_s = None
        self._cancel_started_s = None

    def _pressed(self, cc_button, now_s):
        """A new press: switch up after at least PRESS_RELEASE_S released.

        Edge, not level, so holding the switch never toggles twice; and the
        release gap, so the burst of edges one press produces counts once.
        """
        up = cc_button == CC_BUTTON_UP
        pressed = False
        if up and not self._button_up:
            pressed = True
            if self._released_s is not None and now_s - self._released_s < PRESS_RELEASE_S:
                pressed = False
        if self._button_up and not up:
            self._released_s = now_s
        self._button_up = up
        return pressed

    def update(self, gear, cc_button, delta_valid, now_s):
        """
        Advance the mode by one CAN sample.
        """
        errors = []
        in_drive = int(gear) == GEAR_DRIVE
        pressed = self._pressed(cc_button, now_s)

        if self.mode == ABORT:
            if not in_drive:
                self.mode = OFF
            return errors

        if self.mode == CANCEL:
            if now_s - self._cancel_started_s > CANCEL_HOLD_S:
                self.mode = OFF
            return errors

        # A press while engaged always cancels
        if pressed and self.mode != OFF:
            self._cancel(now_s)
            return errors

        if pressed:
            if not self.enabled:
                errors.append('autosteer disabled (autosteer:=false); press ignored')
            elif not in_drive:
                errors.append(f'not in D (gear {int(gear)}); press ignored')
            elif not delta_valid:
                errors.append('delta invalid; staying OFF')
            else:
                self.mode = AUTOSTEER
            return errors

        if self.mode == AUTOSTEER:
            if not in_drive:
                errors.append(f'left D (gear {int(gear)}); cancelling')
                self._cancel(now_s)
            elif not delta_valid:
                errors.append('delta invalid; autosteer OFF')
                self.mode = OFF
        return errors

    def invalidate(self, reason):
        """
        Drop AUTOSTEER at once when the steering command stops being valid.
        """
        if self.mode == AUTOSTEER:
            self.mode = OFF
            return [f'{reason}; autosteer OFF']
        return []

    def _cancel(self, now_s):
        self.mode = CANCEL
        self._cancel_started_s = now_s
