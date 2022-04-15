"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Pose2 unit tests.
Author: Frank Dellaert & Duy Nguyen Ta & John Lambert
"""
import math
import unittest

import gtsam
import numpy as np
from gtsam import Point2, Point2Pairs, Pose2
from gtsam.utils.test_case import GtsamTestCase


class TestPose2(GtsamTestCase):
    """Test selected Pose2 methods."""
    def test_adjoint(self) -> None:
        """Test adjoint method."""
        xi = np.array([1, 2, 3])
        expected = np.dot(Pose2.adjointMap_(xi), xi)
        actual = Pose2.adjoint_(xi, xi)
        np.testing.assert_array_equal(actual, expected)

    def test_transformTo(self):
        """Test transformTo method."""
        pose = Pose2(2, 4, -math.pi/2)
        actual = pose.transformTo(Point2(3, 2))
        expected = Point2(2, 1)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        # multi-point version
        points = np.stack([Point2(3, 2), Point2(3, 2)]).T
        actual_array = pose.transformTo(points)
        self.assertEqual(actual_array.shape, (2, 2))
        expected_array = np.stack([expected, expected]).T
        np.testing.assert_allclose(actual_array, expected_array, atol=1e-6)

    def test_transformFrom(self):
        """Test transformFrom method."""
        pose = Pose2(2, 4, -math.pi/2)
        actual = pose.transformFrom(Point2(2, 1))
        expected = Point2(3, 2)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        # multi-point version
        points = np.stack([Point2(2, 1), Point2(2, 1)]).T
        actual_array = pose.transformFrom(points)
        self.assertEqual(actual_array.shape, (2, 2))
        expected_array = np.stack([expected, expected]).T
        np.testing.assert_allclose(actual_array, expected_array, atol=1e-6)

    def test_align(self) -> None:
        """Ensure estimation of the Pose2 element to align two 2d point clouds succeeds.

        Two point clouds represent horseshoe-shapes of the same size, just rotated and translated:

                |  X---X
                |  |
                |  X---X
        ------------------
                |
                |
              O | O
              | | |
              O---O
        """
        pts_a = [
            Point2(1, -3),
            Point2(1, -5),
            Point2(-1, -5),
            Point2(-1, -3),
        ]
        pts_b = [
            Point2(3, 1),
            Point2(1, 1),
            Point2(1, 3),
            Point2(3, 3),
        ]

        # fmt: on
        ab_pairs = Point2Pairs(list(zip(pts_a, pts_b)))
        aTb = Pose2.Align(ab_pairs)
        self.assertIsNotNone(aTb)

        for pt_a, pt_b in zip(pts_a, pts_b):
            pt_a_ = aTb.transformFrom(pt_b)
            np.testing.assert_allclose(pt_a, pt_a_)

        # Matrix version
        A = np.array(pts_a).T
        B = np.array(pts_b).T
        aTb = Pose2.Align(A, B)
        self.assertIsNotNone(aTb)

        for pt_a, pt_b in zip(pts_a, pts_b):
            pt_a_ = aTb.transformFrom(pt_b)
            np.testing.assert_allclose(pt_a, pt_a_)


if __name__ == "__main__":
    unittest.main()
