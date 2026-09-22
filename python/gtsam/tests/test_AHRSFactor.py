"""Tests for the concrete AHRS preintegration and factor wrappers."""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestAHRSFactor(GtsamTestCase):
    """Exercise the supported, non-deprecated AHRS Python API."""

    def test_default_bias_covariance_and_factor(self):
        params = gtsam.PreintegratedRotationParams()
        covariance = np.diag([0.01, 0.02, 0.03])
        params.setGyroscopeCovariance(covariance)
        pim = gtsam.PreintegratedAhrsMeasurements(params)
        pim.integrateMeasurement(np.array([0.1, -0.2, 0.3]), 0.5)

        np.testing.assert_allclose(pim.biasHat(), np.zeros(3))
        np.testing.assert_allclose(pim.preintMeasCov(), covariance * 0.5)

        factor = gtsam.AHRSFactor(1, 2, 3, pim)
        rotation_i = gtsam.Rot3.RzRyRx(0.2, -0.1, 0.4)
        rotation_j = rotation_i.compose(pim.deltaRij())
        np.testing.assert_allclose(
            factor.evaluateError(rotation_i, rotation_j, np.zeros(3)),
            np.zeros(3),
            atol=1e-9,
        )


if __name__ == "__main__":
    unittest.main()
