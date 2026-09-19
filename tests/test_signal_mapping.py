import unittest

from app import BoatSimulatorApp
from boat_sim.signal_mapping import map_motor_commands, map_yaw_angle


class SignalMappingTests(unittest.TestCase):
    def test_swap_happens_before_channel_inversion(self):
        self.assertEqual(
            map_motor_commands(1000, -2000, swap=True, invert_left=True),
            (2000, 1000),
        )

    def test_each_motor_can_be_inverted(self):
        self.assertEqual(
            map_motor_commands(1000, -2000, invert_left=True, invert_right=True),
            (-1000, 2000),
        )

    def test_motor_values_are_clamped(self):
        self.assertEqual(map_motor_commands(20_000, -20_000), (10_000, -10_000))

    def test_yaw_angle_can_be_inverted(self):
        self.assertEqual(map_yaw_angle(0.25, invert=True), -0.25)

    def test_inverted_negative_pi_stays_in_protocol_range(self):
        self.assertEqual(map_yaw_angle(-3.141592653589793, invert=True), -3.141592653589793)

    def test_target_command_format(self):
        self.assertEqual(
            BoatSimulatorApp._format_target_command(30.0000449, 120.0),
            "sim target 30.0000449 120.0000000",
        )


if __name__ == "__main__":
    unittest.main()
