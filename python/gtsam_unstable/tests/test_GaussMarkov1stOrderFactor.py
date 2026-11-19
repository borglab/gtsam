"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for GaussMarkov1stOrderFactor.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase


class TestGaussMarkov1stOrderFactor(GtsamTestCase):
    """x2 = exp(-delta_t / tau) x1 + w, with w the exactly discretized noise."""

    def test_error_and_discrete_noise(self):
        """The error decays x1 per axis and the noise is the exact discrete covariance."""
        tau, delta_t, sigma = np.array([100.0, 150.0, 10.0]), 2.0, 0.5
        factor = gtsam_unstable.GaussMarkov1stOrderFactorVector(
            1, 2, delta_t, tau, gtsam.noiseModel.Isotropic.Sigma(3, sigma))
        x1, x2 = np.array([10.0, 12.0, 13.0]), np.array([10.0, 15.0, 14.0])
        values = gtsam.Values()
        values.insert(1, x1)
        values.insert(2, x2)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        np.testing.assert_allclose(factor.unwhitenedError(values),
                                   x2 - np.exp(-delta_t / tau) * x1)
        np.testing.assert_allclose(
            factor.noiseModel().sigmas()**2,
            sigma**2 * tau / 2 * -np.expm1(-2 * delta_t / tau))

    def test_point3(self):
        """Point3 states give the same error as dynamic vectors."""
        tau = np.array([5.0, 5.0, 5.0])
        factor = gtsam_unstable.GaussMarkov1stOrderFactorPoint3(
            1, 2, 1.0, tau, gtsam.noiseModel.Isotropic.Sigma(3, 1.0))
        values = gtsam.Values()
        values.insert(1, gtsam.Point3(1.0, 2.0, 3.0))
        values.insert(2, gtsam.Point3(0.0, 0.0, 0.0))

        np.testing.assert_allclose(factor.unwhitenedError(values),
                                   -np.exp(-1.0 / 5.0) * np.array([1.0, 2.0, 3.0]))


if __name__ == "__main__":
    unittest.main()
