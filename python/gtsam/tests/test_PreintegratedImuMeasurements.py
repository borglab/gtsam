"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Tests for preintegrated IMU measurements.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestPreintegratedImuMeasurements(GtsamTestCase):
    """Tests the common API of both preintegration backends."""

    def test_error_mode_defaults_and_explicit_compatibility(self):
        """Fresh standard and Combined parameters default to Logmap."""
        params = (
            gtsam.PreintegrationParams(np.zeros(3)),
            gtsam.PreintegrationParams.MakeSharedD(9.81),
            gtsam.PreintegrationParams.MakeSharedU(9.81),
            gtsam.PreintegrationCombinedParams(np.zeros(3)),
            gtsam.PreintegrationCombinedParams.MakeSharedD(9.81),
            gtsam.PreintegrationCombinedParams.MakeSharedU(9.81),
        )

        for value in params:
            self.assertEqual(
                gtsam.ImuFactorErrorMode.Logmap,
                value.getImuFactorErrorMode(),
            )
            value.setImuFactorErrorMode(
                gtsam.ImuFactorErrorMode.ComponentWise)
            self.assertEqual(
                gtsam.ImuFactorErrorMode.ComponentWise,
                value.getImuFactorErrorMode(),
            )
            value.setImuFactorErrorMode(gtsam.ImuFactorErrorMode.Legacy)
            self.assertEqual(
                gtsam.ImuFactorErrorMode.Legacy,
                value.getImuFactorErrorMode(),
            )

    def test_preintegrated_coordinates(self):
        """The raw measurements agree with the component accessors."""
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        pim = gtsam.PreintegratedImuMeasurements(params)
        pim.integrateMeasurement(
            np.array([0.1, -0.2, 9.7]),
            np.array([0.02, -0.03, 0.01]),
            0.1,
        )

        expected = np.concatenate(
            (
                gtsam.Rot3.Logmap(pim.deltaRij()),
                pim.deltaPij(),
                pim.deltaVij(),
            )
        )
        np.testing.assert_allclose(pim.preintegrated(), expected, atol=1e-12)

    def test_residual_covariance_accessors(self):
        """Standard and combined PIMs expose residual-chart covariance."""
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        combined_params = gtsam.PreintegrationCombinedParams.MakeSharedD(9.81)
        pims = (
            (gtsam.PreintegratedImuMeasurements(params), (9, 9)),
            (gtsam.PreintegratedCombinedMeasurements(combined_params),
             (15, 15)),
        )

        for pim, expected_shape in pims:
            for _ in range(10):
                pim.integrateMeasurement(
                    np.array([0.3, -0.2, 9.7]),
                    np.array([0.15, -0.1, 1.0]),
                    0.02,
                )

            covariance = pim.residualCovariance()
            self.assertEqual(expected_shape, covariance.shape)
            np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)

    def _deskew_pims(self):
        """Create standard and combined PIMs with the same yaw trajectory."""
        yaw_rate, duration = 0.4, 2.0
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        standard = gtsam.PreintegratedImuMeasurements(params)
        standard.integrateMeasurement(
            np.zeros(3), np.array([0.0, 0.0, yaw_rate]), duration)

        combined_params = gtsam.PreintegrationCombinedParams.MakeSharedD(9.81)
        combined = gtsam.PreintegratedCombinedMeasurements(combined_params)
        combined.integrateMeasurement(
            np.zeros(3), np.array([0.0, 0.0, yaw_rate]), duration)
        return yaw_rate, duration, (standard, combined)

    def test_deskew_numpy_layouts_and_combined_smoke(self):
        """Deskew accepts C/F NumPy matrices for standard and combined PIMs."""
        yaw_rate, duration, pims = self._deskew_pims()
        c_points = np.arange(24, dtype=float).reshape(6, 4) / 10.0
        f_points = np.asfortranarray(c_points)
        times = np.array([duration, 0.0, duration / 2.0, duration / 4.0])
        velocity = np.array([0.5, -0.2, 0.1])

        for pim in pims:
            implicit = pim.deskewPoints(c_points)
            self.assertEqual(c_points.shape, implicit.shape)
            implicit_with_velocity = pim.deskewPoints(
                c_points, velocity_i=velocity)
            for column in range(c_points.shape[1]):
                time = duration * column / c_points.shape[1]
                rotation = gtsam.Rot3.Expmap(
                    np.array([0.0, 0.0, yaw_rate * time]))
                for row in (0, 3):
                    expected = rotation.rotate(c_points[row:row + 3, column])
                    expected += velocity * time
                    np.testing.assert_allclose(
                        implicit_with_velocity[row:row + 3, column], expected,
                        atol=1e-8)
            np.testing.assert_allclose(
                pim.deskewPointsAtTimes(c_points, times),
                pim.deskewPointsAtTimes(f_points, times),
                atol=1e-12,
                rtol=0.0,
            )
            translated = pim.deskewPointsAtTimes(
                c_points, times, velocity)
            for column, time in enumerate(times):
                rotation = gtsam.Rot3.Expmap(
                    np.array([0.0, 0.0, yaw_rate * time]))
                for row in (0, 3):
                    expected = rotation.rotate(c_points[row:row + 3, column])
                    expected += velocity * time
                    np.testing.assert_allclose(
                        translated[row:row + 3, column], expected, atol=1e-8)
            with self.assertRaises(TypeError):
                pim.deskewPointsAtTimes(c_points.tolist(), times)

    def test_deskew_validation(self):
        """Deskew rejects invalid shapes, time counts, and interval times."""
        _, duration, (pim, _) = self._deskew_pims()
        self.assertEqual((0, 3), pim.deskewPoints(np.empty((0, 3))).shape)
        with self.assertRaises(ValueError):
            pim.deskewPoints(np.zeros((4, 2)))
        with self.assertRaises(ValueError):
            pim.deskewPointsAtTimes(np.zeros((3, 2)), np.zeros(1))
        with self.assertRaises(IndexError):
            pim.deskewPointsAtTimes(
                np.zeros((3, 1)), np.array([-1e-6]))
        with self.assertRaises(IndexError):
            pim.deskewPointsAtTimes(
                np.zeros((3, 1)), np.array([duration + 1e-6]))


if __name__ == "__main__":
    unittest.main()
