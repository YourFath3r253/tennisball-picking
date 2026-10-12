"""One EDGE event per encounter, with cooldown and continuous clear rearm."""
class EdgeUartGate(object):
    def __init__(self, cooldown_sec=3.0, clear_sec=0.5):
        if cooldown_sec <= 0 or clear_sec <= 0:
            raise ValueError('Gate durations must be positive')
        self.cooldown_sec = cooldown_sec
        self.clear_sec = clear_sec
        self.armed = True
        self.last_edge = None
        self.clear_since = None

    def allow(self, state, now):
        if state == 'SAFE':
            if self.clear_since is None:
                self.clear_since = now
            if (not self.armed and now - self.last_edge >= self.cooldown_sec
                    and now - self.clear_since >= self.clear_sec):
                self.armed = True
            return True
        if state != 'EDGE':
            raise ValueError('Unknown court state')
        self.clear_since = None
        if not self.armed:
            return False
        self.armed = False
        self.last_edge = now
        return True
