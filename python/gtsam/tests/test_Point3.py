"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Point3 unit tests.
Author: Frank Dellaert & Fan Jiang
"""
import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestPoint3(GtsamTestCase):
    """Test selected Point3 methods."""

    def test_constructors(self):
        """Test constructors from doubles and vectors."""
        expected = gtsam.Point3(1, 2, 3)
        actual = gtsam.Point3(np.array([1, 2, 3]))
        np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
