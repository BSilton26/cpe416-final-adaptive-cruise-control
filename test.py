import unittest
from controllers.follow_controller.follow_controller import Vehicle

class TestCruiseControlPID(unittest.TestCase):
    def test_00(self):
        """when no obstacle in front of vehicle, test that velocity output is target"""
        v = Vehicle()
        v.front_right_distance_sensor = 255
        v.front_left_distance_sensor = 255
        v.current_velocity = v.current_velocity
        self.assertEqual(v.pid_velocity_correction(), v.target_velocity)
    def test_01(self):

class TestAngularPID(unittest.TestCase):
    def test_P_01(self):
        controller = Vehicle()
        self.assertEqual(controller.p_angular_control())


if __name__ == '__main__':
    unittest.main()
