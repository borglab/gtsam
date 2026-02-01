"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Serialization and deep copy tests.

Author: Varun Agrawal
"""
import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam


@unittest.skipUnless(hasattr(gtsam.Pose3, "serialize"), "Serialization not enabled")
class TestDeepCopy(GtsamTestCase):
    """Tests for deep copy of various GTSAM objects."""

    def test_PreintegratedImuMeasurements(self):
        """
        Test the deep copy of `PreintegratedImuMeasurements`.
        """
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        pim = gtsam.PreintegratedImuMeasurements(params)

        self.assertDeepCopyEquality(pim)

    def test_ImuFactor(self):
        """
        Test the deep copy of `ImuFactor`.
        """
        params = gtsam.PreintegrationParams.MakeSharedD(9.81)
        params.setAccelerometerCovariance(1e-7 * np.eye(3))
        params.setGyroscopeCovariance(1e-8 * np.eye(3))
        params.setIntegrationCovariance(1e-9 * np.eye(3))
        priorBias = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))
        pim = gtsam.PreintegratedImuMeasurements(params, priorBias)

        # Preintegrate a single measurement for serialization to work.
        pim.integrateMeasurement(measuredAcc=np.zeros(3),
                                 measuredOmega=np.zeros(3),
                                 deltaT=0.005)

        factor = gtsam.ImuFactor(0, 1, 2, 3, 4, pim)

        self.assertDeepCopyEquality(factor)

    def test_PreintegratedCombinedMeasurements(self):
        """
        Test the deep copy of `PreintegratedCombinedMeasurements`.
        """
        params = gtsam.PreintegrationCombinedParams(np.asarray([0, 0, -9.81]))
        pim = gtsam.PreintegratedCombinedMeasurements(params)

        self.assertDeepCopyEquality(pim)


if __name__ == "__main__":
    unittest.main()
