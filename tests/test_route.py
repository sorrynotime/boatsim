import unittest

from boat_sim.route import MAX_WAYPOINTS, RoutePlan


class RoutePlanTests(unittest.TestCase):
    def test_route_advances_on_first_inside_sample(self):
        route = RoutePlan()
        route.add(10.0, 0.0)
        route.add(20.0, 0.0)
        route.start()
        self.assertEqual(route.observe(9.5, 0.0, 1.0), "advanced")
        self.assertEqual(route.active_index, 1)

    def test_outside_sample_does_not_advance(self):
        route = RoutePlan()
        route.add(10.0, 0.0)
        route.start()
        self.assertIsNone(route.observe(12.0, 0.0, 1.0))
        self.assertFalse(route.completed)

    def test_last_waypoint_completes_route(self):
        route = RoutePlan()
        route.add(1.0, 2.0)
        route.start()
        self.assertEqual(route.observe(1.0, 2.0, 0.5), "completed")
        self.assertTrue(route.completed)
        self.assertFalse(route.running)

    def test_route_rejects_more_than_twenty_points(self):
        route = RoutePlan()
        for index in range(MAX_WAYPOINTS):
            route.add(float(index), 0.0)
        with self.assertRaises(ValueError):
            route.add(21.0, 0.0)


if __name__ == "__main__":
    unittest.main()
