"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

ScenarioRunner unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import math
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestScenarioRunner(GtsamTestCase):
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
        bias = gtsam.imuBias.ConstantBias()
        runner = gtsam.ScenarioRunner(
            scenario, params, dt, bias)

        # Test specific force at time 0: a is pointing up
        t = 0.0
        a = w * v
        np.testing.assert_almost_equal(
            np.array([0, 0, a + self.g]), runner.actualSpecificForce(t))

    def test_sensor_frame_and_lever_arm(self):
        omega_b = np.array([0.0, 0.0, 2.0])
        scenario = gtsam.ConstantTwistScenario(omega_b, np.zeros(3))
        params = gtsam.PreintegrationParams.MakeSharedU(self.g)
        body_R_sensor = gtsam.Rot3.Yaw(math.pi / 2.0)
        params.setBodyPSensor(
            gtsam.Pose3(body_R_sensor, np.array([1.0, 0.0, 0.0])))
        runner = gtsam.ScenarioRunner(
            scenario, params, 0.01, gtsam.imuBias.ConstantBias())

        np.testing.assert_allclose(
            body_R_sensor.unrotate(omega_b),
            runner.actualAngularVelocity(0.0))
        expected_force_b = np.array([-4.0, 0.0, self.g])
        np.testing.assert_allclose(
            body_R_sensor.unrotate(expected_force_b),
            runner.actualSpecificForce(0.0))


if __name__ == '__main__':
    unittest.main()
