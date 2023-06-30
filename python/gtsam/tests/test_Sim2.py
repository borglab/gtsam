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
from gtsam.utils.test_case import GtsamTestCase

from gtsam import Pose2, Rot2, Similarity2


class TestSim2(GtsamTestCase):
    """Test selected Sim2 methods."""

    def test_align_poses_along_straight_line(self) -> None:
        """Test Align of list of Pose2Pair.

        Scenario:
           3 object poses
           same scale (no gauge ambiguity)
           world frame has poses rotated about 180 degrees.
           world and egovehicle frame translated by 15 meters w.r.t. each other
        """
        R180 = Rot2.fromDegrees(180)

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose2(Rot2(), np.array([5, 0]))
        eTo1 = Pose2(Rot2(), np.array([10, 0]))
        eTo2 = Pose2(Rot2(), np.array([15, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world "w" frame)
        wTo0 = Pose2(R180, np.array([-10, 0]))
        wTo1 = Pose2(R180, np.array([-5, 0]))
        wTo2 = Pose2(R180, np.array([0, 0]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = list(zip(wToi_list, eToi_list))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity2.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses_along_straight_line_gauge(self):
        """Test if Pose2 Align method can account for gauge ambiguity.

        Scenario:
           3 object poses
           with gauge ambiguity (2x scale)
           world frame has poses rotated by 90 degrees.
           world and egovehicle frame translated by 11 meters w.r.t. each other
        """
        R90 = Rot2.fromDegrees(90)

        # Create source poses (three objects o1, o2, o3 living in the egovehicle "e" frame)
        # Suppose they are 3d cuboids detected by an onboard sensor in the egovehicle frame
        eTo0 = Pose2(Rot2(), np.array([1, 0]))
        eTo1 = Pose2(Rot2(), np.array([2, 0]))
        eTo2 = Pose2(Rot2(), np.array([4, 0]))

        eToi_list = [eTo0, eTo1, eTo2]

        # Create destination poses
        # (same three objects, but instead living in the world/city "w" frame)
        wTo0 = Pose2(R90, np.array([0, 12]))
        wTo1 = Pose2(R90, np.array([0, 14]))
        wTo2 = Pose2(R90, np.array([0, 18]))

        wToi_list = [wTo0, wTo1, wTo2]

        we_pairs = list(zip(wToi_list, eToi_list))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        wSe = Similarity2.Align(we_pairs)

        for wToi, eToi in zip(wToi_list, eToi_list):
            self.gtsamAssertEquals(wToi, wSe.transformFrom(eToi))

    def test_align_poses_scaled_squares(self):
        """Test if Align method can account for gauge ambiguity.

        Make sure a big and small square can be aligned.
        The u's represent a big square (10x10), and v's represents a small square (4x4).

        Scenario:
           4 object poses
           with gauge ambiguity (2.5x scale)
        """
        # 0, 90, 180, and 270 degrees yaw
        R0 = Rot2.fromDegrees(0)
        R90 = Rot2.fromDegrees(90)
        R180 = Rot2.fromDegrees(180)
        R270 = Rot2.fromDegrees(270)

        aTi0 = Pose2(R0, np.array([2, 3]))
        aTi1 = Pose2(R90, np.array([12, 3]))
        aTi2 = Pose2(R180, np.array([12, 13]))
        aTi3 = Pose2(R270, np.array([2, 13]))

        aTi_list = [aTi0, aTi1, aTi2, aTi3]

        bTi0 = Pose2(R0, np.array([4, 3]))
        bTi1 = Pose2(R90, np.array([8, 3]))
        bTi2 = Pose2(R180, np.array([8, 7]))
        bTi3 = Pose2(R270, np.array([4, 7]))

        bTi_list = [bTi0, bTi1, bTi2, bTi3]

        ab_pairs = list(zip(aTi_list, bTi_list))

        # Recover the transformation wSe (i.e. world_S_egovehicle)
        aSb = Similarity2.Align(ab_pairs)

        for aTi, bTi in zip(aTi_list, bTi_list):
            self.gtsamAssertEquals(aTi, aSb.transformFrom(bTi))

    def test_constructor(self) -> None:
        """Sim(2) to perform p_b = bSa * p_a"""
        bRa = Rot2()
        bta = np.array([1, 2])
        bsa = 3.0
        bSa = Similarity2(R=bRa, t=bta, s=bsa)
        self.assertIsInstance(bSa, Similarity2)
        np.testing.assert_allclose(bSa.rotation().matrix(), bRa.matrix())
        np.testing.assert_allclose(bSa.translation(), bta)
        np.testing.assert_allclose(bSa.scale(), bsa)

    def test_is_eq(self) -> None:
        """Ensure object equality works properly (are equal)."""
        bSa = Similarity2(R=Rot2(), t=np.array([1, 2]), s=3.0)
        bSa_ = Similarity2(R=Rot2(), t=np.array([1.0, 2.0]), s=3)
        self.gtsamAssertEquals(bSa, bSa_)

    def test_not_eq_translation(self) -> None:
        """Ensure object equality works properly (not equal translation)."""
        bSa = Similarity2(R=Rot2(), t=np.array([2, 1]), s=3.0)
        bSa_ = Similarity2(R=Rot2(), t=np.array([1.0, 2.0]), s=3)
        self.assertNotEqual(bSa, bSa_)

    def test_not_eq_rotation(self) -> None:
        """Ensure object equality works properly (not equal rotation)."""
        bSa = Similarity2(R=Rot2(), t=np.array([2, 1]), s=3.0)
        bSa_ = Similarity2(R=Rot2.fromDegrees(180), t=np.array([2.0, 1.0]), s=3)
        self.assertNotEqual(bSa, bSa_)

    def test_not_eq_scale(self) -> None:
        """Ensure object equality works properly (not equal scale)."""
        bSa = Similarity2(R=Rot2(), t=np.array([2, 1]), s=3.0)
        bSa_ = Similarity2(R=Rot2(), t=np.array([2.0, 1.0]), s=1.0)
        self.assertNotEqual(bSa, bSa_)

    def test_rotation(self) -> None:
        """Ensure rotation component is returned properly."""
        R = Rot2.fromDegrees(90)
        t = np.array([1, 2])
        bSa = Similarity2(R=R, t=t, s=3.0)

        # evaluates to [[0, -1], [1, 0]]
        expected_R = Rot2.fromDegrees(90)
        np.testing.assert_allclose(expected_R.matrix(), bSa.rotation().matrix())

    def test_translation(self) -> None:
        """Ensure translation component is returned properly."""
        R = Rot2.fromDegrees(90)
        t = np.array([1, 2])
        bSa = Similarity2(R=R, t=t, s=3.0)

        expected_t = np.array([1, 2])
        np.testing.assert_allclose(expected_t, bSa.translation())

    def test_scale(self) -> None:
        """Ensure the scale factor is returned properly."""
        bRa = Rot2()
        bta = np.array([1, 2])
        bsa = 3.0
        bSa = Similarity2(R=bRa, t=bta, s=bsa)
        self.assertEqual(bSa.scale(), 3.0)


if __name__ == "__main__":
    unittest.main()
