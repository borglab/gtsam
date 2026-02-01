"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Point2 unit tests.
Author: Frank Dellaert & Fan Jiang
"""
import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestPoint2(GtsamTestCase):
    """Test selected Point2 methods."""

    def test_constructors(self):
        """Test constructors from doubles and vectors."""
        expected = gtsam.Point2(1, 2)
        actual = gtsam.Point2(np.array([1, 2]))
        np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
