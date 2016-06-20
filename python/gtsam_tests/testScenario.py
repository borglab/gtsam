import math
import unittest
import numpy as np

import gtsam

class TestScenario(unittest.TestCase):
    def setUp(self):
        pass

    def test_loop(self):
        # Forward velocity 2m/s
        # Pitch up with angular velocity 6 degree/sec (negative in FLU)
        v = 2
        w = math.radians(6)
        W = np.array([0, -w, 0])
        V = np.array([v, 0, 0])
        scenario = gtsam.ConstantTwistScenario(W, V)
        
        T = 30
        np.testing.assert_almost_equal(W, scenario.omega_b(T))
        np.testing.assert_almost_equal(V, scenario.velocity_b(T))
        np.testing.assert_almost_equal(np.cross(W, V), scenario.acceleration_b(T))
        
        # R = v/w, so test if loop crests at 2*R
        R = v / w
        T30 = scenario.pose(T)
        np.testing.assert_almost_equal(np.array([-math.pi, 0, -math.pi]), T30.rotation().xyz())
        self.assert_(gtsam.Point3(0, 0, 2 * R).equals(T30.translation()))

if __name__ == '__main__':
    unittest.main()
