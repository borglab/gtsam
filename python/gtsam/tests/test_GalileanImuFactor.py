"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Tests for the Python wrapper of the Galilean IMU factor.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestGalileanImuFactor(GtsamTestCase):
    """Exercise Galilean preintegration and factor construction from Python."""

    def test_named_preintegration_backends(self):
        """All named PIM backends are independently constructible."""
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        bias = gtsam.imuBias.ConstantBias()
        named_backend_types = (
            gtsam.PreintegratedImuMeasurementsManifold,
            gtsam.PreintegratedImuMeasurementsTangent,
            gtsam.PreintegratedImuMeasurementsLieGroup,
            gtsam.PreintegratedImuMeasurementsG,
        )

        aliases_of_default = [
            backend_type is gtsam.PreintegratedImuMeasurements
            for backend_type in named_backend_types
        ]
        self.assertEqual(1, sum(aliases_of_default))

        for backend_type in named_backend_types:
            pim = backend_type(params, bias)
            pim.integrateMeasurement(np.zeros(3), np.zeros(3), 0.01)
            self.assertTrue(pim.equals(pim, 1e-9))
            self.assertAlmostEqual(0.01, pim.deltaTij())
            self.assertEqual((9,), pim.preintegrated().shape)
            self.assertEqual((9, 9), pim.preintMeasCov().shape)
            self.assertEqual((9, 9), pim.residualCovariance().shape)
            self.assertEqual((6,), pim.biasHatVector().shape)

    @unittest.skipUnless(
        hasattr(gtsam.PreintegratedImuMeasurementsG, "serialize"),
        "Serialization not enabled")
    def test_serialization(self):
        """The Galilean PIM and factor survive Python pickle round trips."""
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        params.setAccelerometerCovariance(1e-4 * np.eye(3))
        params.setGyroscopeCovariance(1e-6 * np.eye(3))
        params.setIntegrationCovariance(1e-8 * np.eye(3))
        pim = gtsam.PreintegratedImuMeasurementsG(params)
        pim.integrateMeasurement(np.array([0.2, -0.1, 9.7]),
                                 np.array([0.03, -0.02, 0.01]), 0.01)
        self.assertEqualityOnPickleRoundtrip(pim)

        factor = gtsam.GalileanImuFactor(0, 1, 2, 3, 4, pim)
        self.assertEqualityOnPickleRoundtrip(factor)

    def test_preintegrate_predict_and_factor(self):
        """A predicted endpoint has zero Galilean IMU factor error."""
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        params.setAccelerometerCovariance(1e-4 * np.eye(3))
        params.setGyroscopeCovariance(1e-6 * np.eye(3))
        params.setIntegrationCovariance(1e-8 * np.eye(3))

        bias_hat = gtsam.imuBias.ConstantBias(
            np.array([0.01, -0.02, 0.03]),
            np.array([-0.01, 0.02, 0.01]),
        )
        pim = gtsam.PreintegratedImuMeasurementsG(params, bias_hat)
        measurements = (
            (np.array([0.2, -0.1, 9.7]),
             np.array([0.03, -0.02, 0.01]), 0.01),
            (np.array([0.1, 0.2, 9.8]),
             np.array([-0.01, 0.04, 0.02]), 0.02),
        )
        for measured_acc, measured_omega, delta_t in measurements:
            pim.integrateMeasurement(measured_acc, measured_omega, delta_t)

        self.assertAlmostEqual(0.03, pim.deltaTij())
        self.assertEqual((9, 9), pim.preintMeasCov().shape)
        np.testing.assert_allclose(
            pim.preintMeasCov(), pim.residualCovariance(), atol=1e-12)

        state_i = gtsam.NavState(
            gtsam.Rot3.RzRyRx(0.1, -0.2, 0.3),
            np.array([1.0, -2.0, 0.5]),
            np.array([0.4, -0.1, 0.2]),
        )
        state_j = pim.predict(state_i, bias_hat)
        factor = gtsam.GalileanImuFactor(
            gtsam.symbol("x", 0), gtsam.symbol("v", 0),
            gtsam.symbol("x", 1), gtsam.symbol("v", 1),
            gtsam.symbol("b", 0), pim)

        error = factor.evaluateError(
            state_i.pose(), state_i.velocity(),
            state_j.pose(), state_j.velocity(), bias_hat)
        np.testing.assert_allclose(error, np.zeros(9), atol=1e-9)
        self.assertTrue(
            factor.preintegratedMeasurements().equals(pim, 1e-12))


if __name__ == "__main__":
    unittest.main()
