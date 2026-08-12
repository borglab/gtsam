"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Wrapper unit tests for attitude factors on rotation-bearing state types.
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestAttitudeFactor(GtsamTestCase):
    """Test wrapped attitude factor variants."""

    @staticmethod
    def model():
        return gtsam.noiseModel.Diagonal.Sigmas(np.array([0.25, 0.25]))

    @staticmethod
    def n_down():
        return gtsam.Unit3(np.array([0.0, 0.0, -1.0]))

    def check_factor(self, factor_cls, state):
        factor0 = factor_cls(0, self.n_down(), self.model())
        factor = factor_cls(
            0, self.n_down(), self.model(), gtsam.Unit3(np.array([0.0, 0.0, 1.0]))
        )
        self.gtsamAssertEquals(factor0.nRef(), factor.nRef(), 1e-9)
        self.gtsamAssertEquals(factor0.bMeasured(), factor.bMeasured(), 1e-9)
        np.testing.assert_allclose(factor.evaluateError(state), np.zeros(2), atol=1e-9)

        values = gtsam.Values()
        values.insert(0, state)
        self.assertAlmostEqual(factor.error(values), 0.0, places=9)

    def test_navstate_attitude_factor(self):
        state = gtsam.NavState(
            gtsam.Rot3(),
            np.array([-5.0, 8.0, -11.0]),
            np.array([0.2, -0.4, 0.6]),
        )
        self.check_factor(gtsam.AttitudeFactorNavState, state)

    def test_gal3_attitude_factor(self):
        state = gtsam.Gal3(
            gtsam.Rot3(),
            np.array([-5.0, 8.0, -11.0]),
            np.array([0.2, -0.4, 0.6]),
            1.25,
        )
        self.check_factor(gtsam.AttitudeFactorGal3, state)

    def test_se23_attitude_factor(self):
        x = np.array(
            [
                [-5.0, 0.2],
                [8.0, -0.4],
                [-11.0, 0.6],
            ]
        )
        state = gtsam.Se23(gtsam.Rot3(), x)
        self.check_factor(gtsam.AttitudeFactorSe23, state)

    def test_extended_pose3d_attitude_factor(self):
        x = np.array(
            [
                [-5.0, 0.2, 1.3],
                [8.0, -0.4, 2.1],
                [-11.0, 0.6, -0.7],
            ]
        )
        state = gtsam.ExtendedPose3d(gtsam.Rot3(), x)
        self.check_factor(gtsam.AttitudeFactorExtendedPose3d, state)


if __name__ == "__main__":
    unittest.main()
