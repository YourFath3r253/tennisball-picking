import unittest
from edge_uart_gate import EdgeUartGate

class GateTests(unittest.TestCase):
    def test_continuous_edge_never_retriggers(self):
        g = EdgeUartGate()
        self.assertTrue(g.allow('EDGE', 0))
        for t in (0.1, 1, 3, 10, 100):
            self.assertFalse(g.allow('EDGE', t))

    def test_second_line_during_cooldown_is_blocked(self):
        g = EdgeUartGate()
        g.allow('EDGE', 0)
        g.allow('SAFE', 0.1)
        g.allow('SAFE', 0.8)
        self.assertFalse(g.allow('EDGE', 1))

    def test_rearm_requires_continuous_clear(self):
        g = EdgeUartGate()
        g.allow('EDGE', 0)
        g.allow('SAFE', 3)
        self.assertFalse(g.allow('EDGE', 3.4))
        g.allow('SAFE', 3.5)
        g.allow('SAFE', 3.8)
        self.assertFalse(g.allow('EDGE', 3.9))
        g.allow('SAFE', 4)
        g.allow('SAFE', 4.6)
        self.assertTrue(g.allow('EDGE', 4.7))
        self.assertFalse(g.allow('EDGE', 4.8))

    def test_safe_remains_real_heartbeat_state(self):
        g = EdgeUartGate()
        g.allow('EDGE', 0)
        self.assertTrue(g.allow('SAFE', 0.1))

if __name__ == '__main__':
    unittest.main()
