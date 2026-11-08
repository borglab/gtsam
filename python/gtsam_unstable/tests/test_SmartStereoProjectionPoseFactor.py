"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Fixed cameras of SmartStereoProjectionPoseFactor through the wrapper.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
import gtsam_unstable
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


class TestSmartStereoProjectionPoseFactor(GtsamTestCase):
    """Conditioning keeps the stereo measurement and its calibration."""

    def test_condition_on_fixes_a_camera(self):
        """conditionOn fixes a pose, keeping the error and calibration."""
        calibration = gtsam.Cal3_S2Stereo(500.0, 500.0, 0.0, 320.0, 240.0, 0.2)
        point = gtsam.Point3(0.5, -0.2, 5.0)
        poses = [
            gtsam.Pose3(),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.0, 1.0, 0.0)),
        ]
        factor = gtsam_unstable.SmartStereoProjectionPoseFactor(
            gtsam.noiseModel.Isotropic.Sigma(3, 1.0)
        )
        values = gtsam.Values()
        for index, pose in enumerate(poses):
            measured = gtsam.StereoCamera(pose, calibration).project(point)
            factor.add(
                gtsam.StereoPoint2(
                    measured.uL() + 0.3 * index,
                    measured.uR(),
                    measured.v() - 0.1,
                ),
                X(index),
                calibration,
            )
            values.insert(X(index), pose)

        fixed_values = gtsam.Values()
        fixed_values.insert(X(0), poses[0])
        conditioned = factor.conditionOn(fixed_values)
        self.assertIsInstance(
            conditioned, gtsam_unstable.SmartStereoProjectionPoseFactor
        )
        self.assertEqual(list(conditioned.keys()), [X(1), X(2)])
        self.assertEqual(list(conditioned.fixedMeasurements()), [0])
        self.assertEqual(list(conditioned.activeMeasurements()), [1, 2])
        self.assertTrue(conditioned.isFixedMeasurement(0))
        self.gtsamAssertEquals(
            conditioned.fixedCameras().at(0),
            gtsam.StereoCamera(poses[0], calibration),
        )
        live_values = gtsam.Values()
        live_values.insert(X(1), poses[1])
        live_values.insert(X(2), poses[2])
        self.assertAlmostEqual(
            factor.error(values), conditioned.error(live_values), places=9
        )
        self.gtsamAssertEquals(
            factor.cameraForMeasurement(2, poses[2]),
            gtsam.StereoCamera(poses[2], calibration),
        )


if __name__ == "__main__":
    unittest.main()
