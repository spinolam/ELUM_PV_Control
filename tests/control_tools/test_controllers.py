# tests/controllers.py
import unittest
from control_tools.controllers import PIController

class TestIntegralActionFunction(unittest.TestCase):
    def test_integral_action(self):
        """ Test the integral accumulutation of the PI controller"""
        pi_controller = PIController(1, 1)
        self.assertEqual(pi_controller.integral_action(1000, 1000), 0)
        self.assertEqual(pi_controller.integral_action(1000, 900), 200)
        self.assertEqual(pi_controller.integral_action(1000, 900), 300)

if __name__ == "__main__":
    unittest.main()