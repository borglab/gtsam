"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Pose2 unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam import Pose2
from gtsam.utils.test_case import GtsamTestCase


class TestPose2(GtsamTestCase):
    """Test selected Pose2 methods."""

    def test_adjoint(self):
        """Test adjoint method."""
        xi = np.array([1, 2, 3])
        expected = np.dot(Pose2.adjointMap_(xi), xi)
        actual = Pose2.adjoint_(xi, xi)
        np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
