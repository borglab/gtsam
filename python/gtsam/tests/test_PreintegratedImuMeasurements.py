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


if __name__ == "__main__":
    unittest.main()
