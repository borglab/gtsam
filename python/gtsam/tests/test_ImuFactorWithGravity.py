"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Wrapper unit tests for the IMU factors with an optimized gravity variable.
"""

import unittest

import numpy as np

import gtsam
from gtsam import ImuFactorWithGravityDirection, ImuFactorWithGravityVector
from gtsam.symbol_shorthand import B, G, V, X
from gtsam.utils.test_case import GtsamTestCase


def stationary_pim(gravity=np.array([0.0, 0.0, -9.81])):
    """Integrate a stationary IMU: the accelerometer measures -gravity.

    Note: the params' own gravity (MakeSharedD: z-DOWN, (0, 0, +9.81)) points
    opposite to the gravity used to generate the data. Only the params' NORM is
    used by the factors (as the default magnitude), so this deliberately
    catches any leakage of the params' gravity direction into the error.
    """
    params = gtsam.PreintegrationParams.MakeSharedD(9.81)
    params.setAccelerometerCovariance(1e-4 * np.eye(3))
    params.setGyroscopeCovariance(1e-4 * np.eye(3))
    params.setIntegrationCovariance(1e-7 * np.eye(3))
    pim = gtsam.PreintegratedImuMeasurements(params)
    for _ in range(10):
        pim.integrateMeasurement(-gravity, np.zeros(3), 0.1)
    return pim


class TestImuFactorWithGravity(GtsamTestCase):
    """Smoke tests: construction and error evaluation through the wrapper."""

    def test_direction_factor(self):
        pim = stationary_pim()
        factor = ImuFactorWithGravityDirection(
            X(1), V(1), X(2), V(2), B(1), G(0), pim)
        self.assertAlmostEqual(factor.gravityMagnitude(), 9.81)

        error = factor.evaluateError(
            gtsam.Pose3(), np.zeros(3), gtsam.Pose3(), np.zeros(3),
            gtsam.imuBias.ConstantBias(), gtsam.Unit3(np.array([0.0, 0.0, -1.0])))
        np.testing.assert_allclose(error, np.zeros(9), atol=1e-9)

    def test_direction_factor_explicit_magnitude(self):
        pim = stationary_pim()
        factor = ImuFactorWithGravityDirection(
            X(1), V(1), X(2), V(2), B(1), G(0), pim, 1.62)
        self.assertAlmostEqual(factor.gravityMagnitude(), 1.62)

    def test_vector_factor(self):
        pim = stationary_pim()
        factor = ImuFactorWithGravityVector(
            X(1), V(1), X(2), V(2), B(1), G(0), pim)
        error = factor.evaluateError(
            gtsam.Pose3(), np.zeros(3), gtsam.Pose3(), np.zeros(3),
            gtsam.imuBias.ConstantBias(), np.array([0.0, 0.0, -9.81]))
        np.testing.assert_allclose(error, np.zeros(9), atol=1e-9)

    def test_vector_factor_rejects_magnitude(self):
        """The Point3 parametrization optimizes the magnitude with the
        variable; a magnitude argument is rejected (use VectorNormFactor3)."""
        pim = stationary_pim()
        with self.assertRaises(ValueError):
            ImuFactorWithGravityVector(
                X(1), V(1), X(2), V(2), B(1), G(0), pim, 9.81)

    def test_combined_direction_factor(self):
        params = gtsam.PreintegrationCombinedParams.MakeSharedD(9.81)
        params.setAccelerometerCovariance(1e-4 * np.eye(3))
        params.setGyroscopeCovariance(1e-6 * np.eye(3))
        params.setIntegrationCovariance(1e-8 * np.eye(3))
        params.setBiasAccCovariance(1e-6 * np.eye(3))
        params.setBiasOmegaCovariance(1e-8 * np.eye(3))
        pim = gtsam.PreintegratedCombinedMeasurements(params)
        for _ in range(10):
            pim.integrateMeasurement(np.array([0.0, 0.0, 9.81]), np.zeros(3), 0.1)

        zb = gtsam.imuBias.ConstantBias()
        factor = gtsam.CombinedImuFactorWithGravityDirection(
            X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim)
        self.assertAlmostEqual(factor.gravityMagnitude(), 9.81)
        error = factor.evaluateError(
            gtsam.Pose3(), np.zeros(3), gtsam.Pose3(), np.zeros(3), zb, zb,
            gtsam.Unit3(np.array([0.0, 0.0, -1.0])))
        np.testing.assert_allclose(error, np.zeros(15), atol=1e-9)

        explicit = gtsam.CombinedImuFactorWithGravityDirection(
            X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim, 1.62)
        self.assertAlmostEqual(explicit.gravityMagnitude(), 1.62)

        vector_factor = gtsam.CombinedImuFactorWithGravityVector(
            X(1), V(1), X(2), V(2), B(1), B(2), G(0), pim)
        error = vector_factor.evaluateError(
            gtsam.Pose3(), np.zeros(3), gtsam.Pose3(), np.zeros(3), zb, zb,
            np.array([0.0, 0.0, -9.81]))
        np.testing.assert_allclose(error, np.zeros(15), atol=1e-9)

    def test_vector_norm_factor(self):
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.03)
        factor = gtsam.VectorNormFactor3(G(0), 9.81, model)
        self.assertAlmostEqual(factor.norm(), 9.81)
        error = factor.evaluateError(np.array([0.0, 0.0, -9.81]))
        np.testing.assert_allclose(error, np.zeros(1), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
