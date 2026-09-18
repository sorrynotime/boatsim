import math
import unittest

from boat_sim.model import DifferentialBoat, gps_to_local, local_to_gps


class ModelTests(unittest.TestCase):
    def test_equal_forward_thrust_moves_north(self):
        boat = DifferentialBoat()
        for _ in range(500):
            boat.step(10000, 10000, 0.01)
        self.assertGreater(boat.state.north_m, 5.0)
        self.assertAlmostEqual(boat.state.east_m, 0.0, places=5)
        self.assertAlmostEqual(boat.state.yaw_rad, 0.0, places=5)

    def test_right_motor_turns_clockwise(self):
        boat = DifferentialBoat()
        for _ in range(100):
            boat.step(0, 10000, 0.01)
        self.assertGreater(boat.state.yaw_rad, 0.0)

    def test_local_north_increases_latitude(self):
        latitude, longitude = local_to_gps(0.0, 100.0, 30.0, 120.0)
        self.assertGreater(latitude, 30.0)
        self.assertAlmostEqual(longitude, 120.0)

    def test_gps_local_round_trip(self):
        latitude, longitude = local_to_gps(12.5, -7.25, 30.0, 120.0)
        east, north = gps_to_local(latitude, longitude, 30.0, 120.0)
        self.assertTrue(math.isclose(east, 12.5, abs_tol=1e-6))
        self.assertTrue(math.isclose(north, -7.25, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
