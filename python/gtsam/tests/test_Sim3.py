"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Sim3 unit tests.
Author: John Lambert
"""
# pylint: disable=no-name-in-module
import math
import unittest

import numpy as np

import gtsam
from gtsam import Point3, Pose3, Pose3Pairs, Rot3, Similarity3
from gtsam.utils.test_case import GtsamTestCase


class TestSim3(GtsamTestCase):
    """Test selected Sim3 methods."""

    def test_align_poses_along_straight_line(self):
        """Test Align Pose3Pairs method."""

        Rx90 = Rot3.Rx(np.deg2rad(90))

        w1Ti0 = Pose3(Rot3(), np.array([5, 0, 0]))
        w1Ti1 = Pose3(Rot3(), np.array([10, 0, 0]))
        w1Ti2 = Pose3(Rot3(), np.array([15, 0, 0]))

        w1Ti_list = [w1Ti0, w1Ti1, w1Ti2]

        w2Ti0 = Pose3(Rx90, np.array([-10, 0, 0]))
        w2Ti1 = Pose3(Rx90, np.array([-5, 0, 0]))
        w2Ti2 = Pose3(Rx90, np.array([0, 0, 0]))

        w2Ti_list = [w2Ti0, w2Ti1, w2Ti2]

        pairs = list(zip(w2Ti_list, w1Ti_list))
        pose_pairs = Pose3Pairs(pairs)

        w2Sw1 = Similarity3.Align(pose_pairs)

        for w1Ti, w2Ti in zip(w1Ti_list, w2Ti_list):
            self.gtsamAssertEquals(w2Ti, w2Sw1.transformFrom(w1Ti))


if __name__ == "__main__":
    unittest.main()
