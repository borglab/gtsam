"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Basis unit tests.
Author: Frank Dellaert & Varun Agrawal (Python)
"""
import unittest

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestSam(GtsamTestCase):
    """
    Tests python binding for classes/functions in `sam.i`.
    """
    def test_RangeFactor2D(self):
        """
        Test that `measured` works as expected for RangeFactor2D.
        """
        measurement = 10.0
        factor = gtsam.RangeFactor2D(1, 2, measurement,
                                     gtsam.noiseModel.Isotropic.Sigma(1, 1))
        self.assertEqual(measurement, factor.measured())

    def test_BearingFactor2D(self):
        """
        Test that `measured` works as expected for BearingFactor2D.
        """
        measurement = gtsam.Rot2(.3)
        factor = gtsam.BearingFactor2D(1, 2, measurement,
                                       gtsam.noiseModel.Isotropic.Sigma(1, 1))
        self.gtsamAssertEquals(measurement, factor.measured())

    def test_BearingRangeFactor2D(self):
        """
        Test that `measured` works as expected for BearingRangeFactor2D.
        """
        range_measurement = 10.0
        bearing_measurement = gtsam.Rot2(0.3)
        factor = gtsam.BearingRangeFactor2D(
            1, 2, bearing_measurement, range_measurement,
            gtsam.noiseModel.Isotropic.Sigma(2, 1))
        measurement = factor.measured()

        self.assertEqual(range_measurement, measurement.range())
        self.gtsamAssertEquals(bearing_measurement, measurement.bearing())

    def test_BearingRangeFactor3D(self):
        """
        Test that `measured` works as expected for BearingRangeFactor3D.
        """
        bearing_measurement = gtsam.Unit3()
        range_measurement = 10.0
        factor = gtsam.BearingRangeFactor3D(
            1, 2, bearing_measurement, range_measurement,
            gtsam.noiseModel.Isotropic.Sigma(3, 1))
        measurement = factor.measured()

        self.assertEqual(range_measurement, measurement.range())
        self.gtsamAssertEquals(bearing_measurement, measurement.bearing())

    def test_BearingRangeFactorPose3(self):
        """
        Test that `measured` works as expected for BearingRangeFactorPose3.
        """
        range_measurement = 10.0
        bearing_measurement = gtsam.Unit3()
        factor = gtsam.BearingRangeFactorPose3(
            1, 2, bearing_measurement, range_measurement,
            gtsam.noiseModel.Isotropic.Sigma(3, 1))
        measurement = factor.measured()

        self.assertEqual(range_measurement, measurement.range())
        self.gtsamAssertEquals(bearing_measurement, measurement.bearing())


if __name__ == "__main__":
    unittest.main()
