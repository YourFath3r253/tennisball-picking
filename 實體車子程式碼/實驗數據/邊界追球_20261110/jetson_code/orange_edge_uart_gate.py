"""Orange EDGE UART gate: time-based cooldown without SAFE rearm."""

class EdgeUartGate(object):
    def __init__(self, cooldown_sec=0.5, clear_sec=0.5):
        if cooldown_sec <= 0:
            raise ValueError("Cooldown must be positive")
        self.cooldown_sec = cooldown_sec
        self.clear_sec = clear_sec
        self.armed = True
        self.last_edge = None
        self.clear_since = None

    def allow(self, state, now):
        if state == "SAFE":
            self.clear_since = now
            self.armed = True
            return True

        if state != "EDGE":
            raise ValueError("Unknown court state")

        self.clear_since = None

        if (self.last_edge is not None and
                now - self.last_edge < self.cooldown_sec):
            self.armed = False
            return False

        self.last_edge = now
        self.armed = False
        return True
