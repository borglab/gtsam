"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for BiasedGPSFactor.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import B, X
from gtsam.utils.test_case import GtsamTestCase


class TestBiasedGPSFactor(GtsamTestCase):
    """The error is pose.translation() + bias - measured."""

    def setUp(self):
        self.pose = gtsam.Pose3(gtsam.Rot3.Rodrigues(0.1, 0.2, 0.3),
                                gtsam.Point3(1.0, 0.5, 0.2))
        self.bias = np.array([0.3, -0.2, 0.1])
        self.measured = gtsam.Point3(1.5, 0.0, 0.4)
        self.factor = gtsam_unstable.BiasedGPSFactor(
            X(1), B(1), self.measured, gtsam.noiseModel.Isotropic.Sigma(3, 0.5))
        self.values = gtsam.Values()
        self.values.insert(X(1), self.pose)
        self.values.insert(B(1), self.bias)

    def test_error(self):
        """The error is the biased position minus the measurement."""
        self.assertIsInstance(self.factor, gtsam.NoiseModelFactor)
        np.testing.assert_allclose(self.factor.measured(), self.measured)
        np.testing.assert_allclose(
            self.factor.unwhitenedError(self.values),
            self.pose.translation() + self.bias - self.measured)

    def test_jacobians(self):
        """The whitened Jacobians are [0 R] and I, scaled by 1/sigma."""
        jacobian, _ = self.factor.linearize(self.values).jacobian()
        expected = np.hstack([np.zeros((3, 3)), self.pose.rotation().matrix(),
                              np.eye(3)]) / 0.5
        np.testing.assert_allclose(jacobian, expected, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
