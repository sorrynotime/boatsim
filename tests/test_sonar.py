import math
import unittest

from boat_sim.collision import CircleObstacle, PolygonObstacle
from boat_sim.model import BoatState
from boat_sim.sonar import (
    MIN_HIT_DISTANCE_M,
    SonarConfig,
    ray_circle_first_hit,
    ray_polygon_first_hit,
    synthesize_sonar,
)


class TestRayCircle(unittest.TestCase):
    def test_hit_ahead(self):
        t = ray_circle_first_hit((0.0, 0.0), (0.0, 1.0), (0.0, 5.0), 1.0)
        self.assertAlmostEqual(t, 4.0, places=6)

    def test_miss(self):
        self.assertIsNone(ray_circle_first_hit((0.0, 0.0), (0.0, 1.0), (5.0, 5.0), 1.0))

    def test_behind(self):
        self.assertIsNone(ray_circle_first_hit((0.0, 0.0), (0.0, 1.0), (0.0, -5.0), 1.0))

    def test_origin_inside(self):
        t = ray_circle_first_hit((0.0, 0.0), (0.0, 1.0), (0.0, 0.0), 2.0)
        self.assertAlmostEqual(t, MIN_HIT_DISTANCE_M, places=6)

    def test_oblique(self):
        t = ray_circle_first_hit((0.0, 0.0), (0.0, 1.0), (1.0, 5.0), 1.0)
        self.assertAlmostEqual(t, 5.0 - math.sqrt(1.0 - 1.0), places=6)  # center on axis


class TestRayPolygon(unittest.TestCase):
    square = ((-2.0, 4.0), (2.0, 4.0), (2.0, 6.0), (-2.0, 6.0))

    def test_hit_edge(self):
        t = ray_polygon_first_hit((0.0, 0.0), (0.0, 1.0), self.square)
        self.assertAlmostEqual(t, 4.0, places=6)

    def test_miss_side(self):
        self.assertIsNone(ray_polygon_first_hit((5.0, 0.0), (0.0, 1.0), self.square))


class TestSynthesize(unittest.TestCase):
    def config(self) -> SonarConfig:
        return SonarConfig(sensor_range=5.0, beam_width=math.radians(60.0),
                           axis_gap=math.radians(40.0))

    def test_open_water(self):
        sample = synthesize_sonar(BoatState(), self.config(), [])
        self.assertEqual(sample.valid, 3)
        self.assertEqual(sample.hit, 0)
        self.assertEqual(sample.left_m, 5.0)
        self.assertEqual(sample.right_m, 5.0)

    def test_island_dead_ahead(self):
        state = BoatState()
        obstacles = [CircleObstacle(0.0, 3.0, 0.5)]
        sample = synthesize_sonar(state, self.config(), obstacles)
        self.assertEqual(sample.valid, 3)
        self.assertEqual(sample.hit, 3)
        # 波束扇(±30°)取最近回波, 距离在 [岛近缘, 量程) 内且左右对称
        self.assertGreater(sample.left_m, 3.0 - 0.5 - 0.15)
        self.assertLess(sample.left_m, 3.2)
        self.assertAlmostEqual(sample.left_m, sample.right_m, places=3)

    def test_island_left_only(self):
        # 岛在左波束扇内(-20°±30°)、右波束扇(+20°±30°)之外
        state = BoatState()
        obstacles = [CircleObstacle(-2.5, 2.0, 0.3)]
        sample = synthesize_sonar(state, self.config(), obstacles)
        self.assertEqual(sample.hit, 1)
        self.assertLess(sample.left_m, 5.0)
        self.assertEqual(sample.right_m, 5.0)

    def test_range_clamp(self):
        state = BoatState()
        obstacles = [CircleObstacle(0.0, 40.0, 2.0)]  # 远超量程
        sample = synthesize_sonar(state, self.config(), obstacles)
        self.assertEqual(sample.hit, 0)
        self.assertEqual(sample.left_m, 5.0)


if __name__ == "__main__":
    unittest.main()
