import math
import unittest
import numpy as np

import gtsam

class TestScenarioRunner(unittest.TestCase):
    def setUp(self):
        self.g = 10  # simple gravity constant

    def test_loop_runner(self):
        # Forward velocity 2m/s
        # Pitch up with angular velocity 6 degree/sec (negative in FLU)
        v = 2
        w = math.radians(6)
        W = np.array([0, -w, 0])
        V = np.array([v, 0, 0])
        scenario = gtsam.ConstantTwistScenario(W, V)

        dt = 0.1
        params = gtsam.PreintegrationParams.MakeSharedU(self.g)
        runner = gtsam.ScenarioRunner(gtsam.ScenarioPointer(scenario), params, dt)

        # Test specific force at time 0: a is pointing up 
        t = 0.0
        a = w * v
        np.testing.assert_almost_equal(np.array([0, 0, a + self.g]), runner.actualSpecificForce(t))

if __name__ == '__main__':
    unittest.main()
