"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for the two known-landmark pose conventions.
"""

import unittest

import gtsam
import numpy as np


class TestKnownLandmarkFactor(unittest.TestCase):
    """Verify conventional wTk and certifiable kTw wrapper semantics."""

    def test_pose2_conventions(self):
        """Both Pose2 factors predict the same kP from inverse pose states."""
        wTk = gtsam.Pose2(0.4, -0.2, 0.3)
        kTw = wTk.inverse()
        wL = np.array([1.2, -0.7])
        measured_kP = wTk.transformTo(wL)
        model = gtsam.noiseModel.Unit.Create(2)

        conventional = gtsam.KnownLandmarkFactorPose2(
            0, wL, measured_kP, model
        )
        certifiable = gtsam.KnownLandmarkFactor2Pose2(
            0, wL, measured_kP, model
        )

        np.testing.assert_allclose(
            conventional.evaluateError(wTk), np.zeros(2), atol=1e-12
        )
        np.testing.assert_allclose(
            certifiable.evaluateError(kTw), np.zeros(2), atol=1e-12
        )

    def test_pose3_conventions(self):
        """Both factors predict the same kP from inverse pose states."""
        wTk = gtsam.Pose3(
            gtsam.Rot3.RzRyRx(0.2, -0.1, 0.3),
            np.array([0.4, -0.2, 0.6]),
        )
        kTw = wTk.inverse()
        wL = np.array([1.2, -0.7, 0.5])
        measured_kP = wTk.transformTo(wL)
        model = gtsam.noiseModel.Unit.Create(3)

        conventional = gtsam.KnownLandmarkFactorPose3(0, wL, measured_kP, model)
        certifiable = gtsam.KnownLandmarkFactor2Pose3(0, wL, measured_kP, model)

        np.testing.assert_allclose(
            conventional.evaluateError(wTk), np.zeros(3), atol=1e-12
        )
        np.testing.assert_allclose(
            certifiable.evaluateError(kTw), np.zeros(3), atol=1e-12
        )


if __name__ == "__main__":
    unittest.main()
