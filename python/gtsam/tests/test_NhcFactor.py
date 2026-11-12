"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for the non-holonomic constraint (NHC) factors.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import M, V, X
from gtsam.utils.test_case import GtsamTestCase


class TestNhcFactor(GtsamTestCase):
    """Calls the wrapped NHC factors through Python."""

    def setUp(self):
        self.angular_velocity = np.array([0.01, -0.02, 0.05])
        self.lever_arm = np.array([-1.3, 0.1, 0.2])
        self.model = gtsam.noiseModel.Isotropic.Sigma(2, 0.1)
        self.mount_rotation = gtsam.Rot3.RzRyRx(0.02, -0.01, 0.03)
        self.pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.1, -0.2, 0.3),
                                gtsam.Point3(1, 2, 3))

    def test_nhc_factor_accessors_and_error(self):
        """Accessors round-trip; the error is lateral and vertical velocity."""
        factor = gtsam.NhcFactor(X(0), V(0), self.angular_velocity,
                                 self.lever_arm, self.model,
                                 self.mount_rotation)
        np.testing.assert_allclose(factor.angularVelocity(),
                                   self.angular_velocity)
        np.testing.assert_allclose(factor.leverArm(), self.lever_arm)
        self.gtsamAssertEquals(factor.mountRotation(), self.mount_rotation)

        aligned = gtsam.NhcFactor(X(0), V(0), np.zeros(3), np.zeros(3),
                                  self.model)
        velocity = self.pose.rotation().rotate(np.array([5.0, 0.2, -0.3]))
        np.testing.assert_allclose(aligned.evaluateError(self.pose, velocity),
                                   [0.2, -0.3], atol=1e-9)

    def test_calib_matches_fixed_factor(self):
        """NhcFactorCalib matches NhcFactor at the same mounting rotation."""
        velocity = np.array([2.0, 0.3, -0.1])
        fixed = gtsam.NhcFactor(X(0), V(0), self.angular_velocity,
                                self.lever_arm, self.model,
                                self.mount_rotation)
        calib = gtsam.NhcFactorCalib(X(0), V(0), M(0), self.angular_velocity,
                                     self.lever_arm, self.model)
        np.testing.assert_allclose(
            fixed.evaluateError(self.pose, velocity),
            calib.evaluateError(self.pose, velocity, self.mount_rotation),
            atol=1e-12)

        values = gtsam.Values()
        values.insert(X(0), self.pose)
        values.insert(V(0), velocity)
        values.insert(M(0), self.mount_rotation)
        self.assertAlmostEqual(fixed.error(values), calib.error(values))


if __name__ == "__main__":
    unittest.main()
