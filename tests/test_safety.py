import copy
import unittest
from icv.model import Vehicle
from icv.scenarios import make_scenario
from icv.simulation import run, score


class SafetyTests(unittest.TestCase):
    def test_collision_is_detected(self):
        vehicle = Vehicle()
        scenario = make_scenario("traffic_green", vehicle)
        scenario.obstacles = [[0, 1.75, 0, 4.6, 1.8, 0]]
        result, _ = run(scenario, vehicle)
        self.assertTrue(result["collision"])
        self.assertFalse(result["passed"])
        self.assertEqual(result["local_score_estimate"], 0)

    def test_red_stop_and_green_response(self):
        vehicle = Vehicle()
        scenario = make_scenario("traffic_red", vehicle)
        result, _ = run(scenario, vehicle)
        self.assertTrue(result["passed"])
        self.assertTrue(0 <= result["red_stop_gaps_m"][0] <= .5)
        self.assertLessEqual(result["green_start_delays_s"][0], 5)
        bad = copy.deepcopy(result)
        bad["red_violation"] = True
        self.assertEqual(score(scenario, bad)[0], 0)
        bad = copy.deepcopy(result)
        bad["red_stop_gaps_m"] = [.75]
        self.assertEqual(score(scenario, bad)[0], 50)

    def test_boundary_detects_body_not_only_axle(self):
        vehicle = Vehicle()
        scenario = make_scenario("traffic_green", vehicle)
        scenario.initial = (0, .5, 0)
        result, _ = run(scenario, vehicle)
        self.assertTrue(result["boundary_contact"])
        self.assertFalse(result["passed"])

    def test_parked_short_hold_zeroes_score(self):
        scenario = make_scenario("parking", Vehicle())
        result = {"completed": True, "collision": False, "boundary_contact": False,
                  "time_s": 110, "parked_in_slot": True, "reverse_entry": True,
                  "parked_seconds": 9.95}
        estimate, checks = score(scenario, result)
        self.assertEqual(estimate, 0)
        self.assertFalse(checks["stationary_at_least_10s"])


if __name__ == "__main__":
    unittest.main()
