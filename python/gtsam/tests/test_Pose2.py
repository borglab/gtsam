"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Pose2 unit tests.
Author: Frank Dellaert & Duy Nguyen Ta & John Lambert
"""
import unittest

import numpy as np

import gtsam
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
            Point2(3, 1),
            Point2(1, 1),
            Point2(1, 3),
            Point2(3, 3),
        ]
        pts_b = [
            Point2(1, -3),
            Point2(1, -5),
            Point2(-1, -5),
            Point2(-1, -3),
        ]

        # fmt: on
        ab_pairs = Point2Pairs(list(zip(pts_a, pts_b)))
        bTa = gtsam.align(ab_pairs)
        aTb = bTa.inverse()
        assert aTb is not None

        for pt_a, pt_b in zip(pts_a, pts_b):
            pt_a_ = aTb.transformFrom(pt_b)
            assert np.allclose(pt_a, pt_a_)


if __name__ == "__main__":
    unittest.main()
