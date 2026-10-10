import unittest

from boat_sim.collision import CircleObstacle, PolygonObstacle, resolve_motion


class CollisionTests(unittest.TestCase):
    def test_swept_circle_collision_does_not_tunnel(self):
        collision = resolve_motion((-2.0, 0.0), (2.0, 0.0), 0.5, [CircleObstacle(0.0, 0.0, 0.5)])
        self.assertIsNotNone(collision)
        self.assertAlmostEqual(collision.position[0], -1.0001, places=3)
        self.assertLess(collision.normal[0], 0.0)

    def test_polygon_collision(self):
        obstacle = PolygonObstacle(((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)))
        collision = resolve_motion((0.0, -3.0), (0.0, 0.0), 0.25, [obstacle])
        self.assertIsNotNone(collision)
        self.assertLess(collision.position[1], -1.0)

    def test_clear_motion_has_no_collision(self):
        self.assertIsNone(resolve_motion((-2.0, 3.0), (2.0, 3.0), 0.5, [CircleObstacle(0.0, 0.0, 0.5)]))


if __name__ == "__main__":
    unittest.main()
