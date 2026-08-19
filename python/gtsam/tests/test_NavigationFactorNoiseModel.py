"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests confirming navigation factors expose the NoiseModelFactor
interface (noiseModel, whitenedError) in Python.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import B, X
from gtsam.utils.test_case import GtsamTestCase


class TestNavigationFactorNoiseModel(GtsamTestCase):
    """Navigation factors derive from NoiseModelFactor in C++."""

    def test_gps_factor_exposes_noise_model(self):
        sigmas = np.array([1.0, 2.0, 3.0])
        model = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        factor = gtsam.GPSFactor(X(1), gtsam.Point3(1.0, 2.0, 3.0), model)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        np.testing.assert_allclose(factor.noiseModel().sigmas(), sigmas)

    def test_gps_factor_whitened_error(self):
        """Unit noise, so the whitened residual is the raw residual."""
        model = gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
        measured = gtsam.Point3(1.0, 2.0, 3.0)
        factor = gtsam.GPSFactor(X(1), measured, model)

        values = gtsam.Values()
        values.insert(X(1), gtsam.Pose3(gtsam.Rot3(),
                                        gtsam.Point3(1.0, 2.0, 4.0)))

        np.testing.assert_allclose(factor.whitenedError(values),
                                   np.array([0.0, 0.0, 1.0]), atol=1e-9)

    def test_barometric_factor_exposes_noise_model(self):
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.5)
        factor = gtsam.BarometricFactor(X(1), B(1), 1013.25, model)

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertAlmostEqual(factor.noiseModel().sigma(), 0.5)

    def test_whitened_error_scales_by_sigma(self):
        """A 4 m residual under a 2 m sigma whitens to 2."""
        model = gtsam.noiseModel.Isotropic.Sigma(3, 2.0)
        factor = gtsam.GPSFactor(X(1), gtsam.Point3(0.0, 0.0, 0.0), model)

        values = gtsam.Values()
        values.insert(X(1), gtsam.Pose3(gtsam.Rot3(),
                                        gtsam.Point3(0.0, 0.0, 4.0)))

        np.testing.assert_allclose(factor.whitenedError(values),
                                   np.array([0.0, 0.0, 2.0]), atol=1e-9)

    def test_robust_wrapped_factor_reaches_its_noise_model(self):
        """A robust-wrapped factor still exposes the NoiseModelFactor API.

        The residual is down-weighted by the m-estimator, which is what
        makes reaching the noise model necessary in the first place.
        """
        base = gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.345), base)
        factor = gtsam.GPSFactor(X(1), gtsam.Point3(0.0, 0.0, 0.0), robust)

        values = gtsam.Values()
        values.insert(X(1), gtsam.Pose3(gtsam.Rot3(),
                                        gtsam.Point3(0.0, 0.0, 50.0)))

        self.assertIsInstance(factor, gtsam.NoiseModelFactor)
        self.assertIsInstance(factor.noiseModel(), gtsam.noiseModel.Robust)

        # 50 sigma, but Huber pulls the whitened residual well below that.
        whitened = np.linalg.norm(factor.whitenedError(values))
        self.assertGreater(whitened, 0.0)
        self.assertLess(whitened, 50.0)


if __name__ == "__main__":
    unittest.main()
