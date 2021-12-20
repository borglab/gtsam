"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for various factors.

Author: Varun Agrawal
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestNonlinearEquality2Factor(GtsamTestCase):
    """
    Test various instantiations of NonlinearEquality2.
    """
    def test_point3(self):
        """Test for Point3 version."""
        factor = gtsam.NonlinearEquality2Point3(0, 1)
        error = factor.evaluateError(gtsam.Point3(0, 0, 0),
                                     gtsam.Point3(0, 0, 0))

        np.testing.assert_allclose(error, np.zeros(3))


if __name__ == "__main__":
    unittest.main()
