"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Scenario unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
from __future__ import print_function

import math
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase

# pylint: disable=invalid-name, E1101


class TestScenario(GtsamTestCase):
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
        np.testing.assert_almost_equal(
            np.cross(W, V), scenario.acceleration_b(T))

        # R = v/w, so test if loop crests at 2*R
        R = v / w
        T30 = scenario.pose(T)
        xyz = T30.rotation().xyz()
        if xyz[0] < 0:
            xyz = -xyz
        np.testing.assert_almost_equal(
            np.array([math.pi, 0, math.pi]), xyz)
        self.gtsamAssertEquals(gtsam.Point3(
            0, 0, 2.0 * R), T30.translation(), 1e-9)


if __name__ == '__main__':
    unittest.main()
