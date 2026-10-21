"""Check the compiled control library against the established Python implementation."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from icv import controller
from icv.model import State, Vehicle, advance as python_advance
from icv.native import NativeBackend
from icv.planning import Route

LIBRARY = None
for index, argument in enumerate(sys.argv[1:], 1):
    if not argument.startswith("-") and argument.lower().endswith(".dll"):
        LIBRARY = sys.argv.pop(index)
        break


class NativeParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.backend = NativeBackend(LIBRARY)
        except FileNotFoundError as error:
            if LIBRARY:
                raise
            raise unittest.SkipTest(str(error))

    @classmethod
    def tearDownClass(cls):
        cls.backend.close()

    def test_forward_reverse_control_and_dynamics(self):
        rng = np.random.default_rng(20261022)
        vehicle = Vehicle()
        for direction in (1, -1):
            x = np.linspace(0, 100, 701) * direction
            route = Route(np.c_[x, 2 * np.sin(x / 15)], direction)
            for _ in range(100):
                progress = float(rng.uniform(0, route.length))
                point = route.at(progress)
                state = State(point[0], point[1] + rng.uniform(-.5, .5),
                              rng.uniform(-math.pi, math.pi), direction * rng.uniform(0, 15),
                              rng.uniform(-.3, .3))
                steering = controller.steering_control(state, route, progress, vehicle)
                actual = self.backend.steering_control(state, route, progress, vehicle)
                self.assertAlmostEqual(actual, steering, places=9)
                for emergency in (False, True):
                    desired = rng.uniform(0, 12)
                    a = controller.speed_control(state, desired, direction, vehicle, emergency)
                    self.assertAlmostEqual(self.backend.speed_control(state, desired, direction, vehicle, emergency), a, places=10)
                    for dt in (.025, .05, .1):
                        expected = python_advance(state, a, steering, dt, vehicle)
                        native = self.backend.advance(state, a, steering, dt, vehicle)
                        np.testing.assert_allclose(list(vars(native).values()), list(vars(expected).values()), rtol=0, atol=2e-7)

    def test_obstacle_projection_and_braking(self):
        vehicle = Vehicle()
        x = np.linspace(0, 150, 1001)
        route = Route(np.c_[x, 3 * np.sin(x / 25)])
        for progress in (0, 20, 65, 120, route.length - .05):
            point = route.at(progress)
            state = State(*point, 0, 10)
            obstacles = [[float(p[0]), float(p[1]), 0, 4.6, 1.8, 2]
                         for p in (route.at(progress + 8), route.at(progress + 25), route.at(progress + 90))]
            expected = controller.obstacle_speed(state, route, progress, obstacles, vehicle)
            actual = self.backend.obstacle_speed(state, route, progress, obstacles, vehicle)
            np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-8)
        self.assertEqual(self.backend.obstacle_speed(State(0, 0, 0), route, 0, [], vehicle), (math.inf, False, math.inf))

    def test_cpp_closed_loop_scenarios_pass_independent_python_checks(self):
        from icv.scenarios import KEYS, make_scenario
        from icv.simulation import run, score

        vehicle = Vehicle()
        for key in KEYS:
            with self.subTest(scenario=key):
                scenario = make_scenario(key, vehicle)
                result, _ = run(scenario, vehicle, .05, backend=self.backend)
                _, checks = score(scenario, result)
                self.assertTrue(result["completed"], key + " did not finish")
                self.assertTrue(all(checks.values()), f"{key}: {checks}")

    def test_cpp_integrator_remains_stable_for_city_timestep(self):
        from icv.scenarios import make_scenario
        from icv.simulation import run, score

        vehicle = Vehicle()
        for dt in (.025, .05):
            scenario = make_scenario("city", vehicle)
            result, _ = run(scenario, vehicle, dt, backend=self.backend)
            _, checks = score(scenario, result)
            self.assertTrue(result["completed"])
            self.assertTrue(all(checks.values()), str(checks))


if __name__ == "__main__":
    unittest.main(verbosity=2)
