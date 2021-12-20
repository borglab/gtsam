"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Sim3 unit tests.
Author: John Lambert
"""
# pylint: disable=no-name-in-module
import unittest

import numpy as np

import gtsam
from gtsam import Point3, Pose3, Pose3Pairs, Rot3, Similarity3
from gtsam.utils.test_case import GtsamTestCase


class TestSim3(GtsamTestCase):
    """Test selected Sim3 methods."""

    def test_align_poses_along_straight_line(self):
        """Test Align Pose3Pairs method.

        Scenario:
           3 object poses
           same scale (no gauge ambiguity)
           world frame has poses rotated about x-axis (90 degree roll)
           world and egovehicle frame translated by 15 meters w.r.t. each other
        """
        Rx90 = Rot3.Rx(np.deg2rad(90))

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose3(Rot3(), np.array([5, 0, 0]))
        eTo1 = Pose3(Rot3(), np.array([10, 0, 0]))
        eTo2 = Pose3(Rot3(), np.array([15, 0, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose3(Rx90, np.array([-10, 0, 0]))
        wTo1 = Pose3(Rx90, np.array([-5, 0, 0]))
        wTo2 = Pose3(Rx90, np.array([0, 0, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = Pose3Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity3.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses_along_straight_line_gauge(self):
        """Test if Align Pose3Pairs method can account for gauge ambiguity.

        Scenario:
           3 object poses
           with gauge ambiguity (2x scale)
           world frame has poses rotated about z-axis (90 degree yaw)
           world and egovehicle frame translated by 11 meters w.r.t. each other
        """
        Rz90 = Rot3.Rz(np.deg2rad(90))

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose3(Rot3(), np.array([1, 0, 0]))
        eTo1 = Pose3(Rot3(), np.array([2, 0, 0]))
        eTo2 = Pose3(Rot3(), np.array([4, 0, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose3(Rz90, np.array([0, 12, 0]))
        wTo1 = Pose3(Rz90, np.array([0, 14, 0]))
        wTo2 = Pose3(Rz90, np.array([0, 18, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = Pose3Pairs(list(zip(wToi_list, eToi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity3.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses_scaled_squares(self):
        """Test if Align Pose3Pairs method can account for gauge ambiguity.

        Make sure a big and small square can be aligned.
        The u's represent a big square (10x10), and v's represents a small square (4x4).

        Scenario:
           4 object poses
           with gauge ambiguity (2.5x scale)
        """
        # 0, 90, 180, and 270 degrees yaw
        R0 = Rot3.Rz(np.deg2rad(0))
        R90 = Rot3.Rz(np.deg2rad(90))
        R180 = Rot3.Rz(np.deg2rad(180))
        R270 = Rot3.Rz(np.deg2rad(270))

        aTi0 = Pose3(R0, np.array([2, 3, 0]))
        aTi1 = Pose3(R90, np.array([12, 3, 0]))
        aTi2 = Pose3(R180, np.array([12, 13, 0]))
        aTi3 = Pose3(R270, np.array([2, 13, 0]))

        aTi_list = [aTi0, aTi1, aTi2, aTi3]

        bTi0 = Pose3(R0, np.array([4, 3, 0]))
        bTi1 = Pose3(R90, np.array([8, 3, 0]))
        bTi2 = Pose3(R180, np.array([8, 7, 0]))
        bTi3 = Pose3(R270, np.array([4, 7, 0]))

        bTi_list = [bTi0, bTi1, bTi2, bTi3]

        ab_pairs = Pose3Pairs(list(zip(aTi_list, bTi_list)))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        aSb = Similarity3.Align(ab_pairs)

        for aTi, bTi in zip(aTi_list, bTi_list):
            self.gtsamAssertEquals(aTi, aSb.transformFrom(bTi))


if __name__ == "__main__":
    unittest.main()
