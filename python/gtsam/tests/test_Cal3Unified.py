"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Cal3Unified unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestCal3Unified(GtsamTestCase):

    def test_Cal3Unified(self):
        K = gtsam.Cal3Unified()
        self.assertEqual(K.fx(), 1.)
        self.assertEqual(K.fx(), 1.)

    def test_retract(self):
        expected = gtsam.Cal3Unified(100 + 2, 105 + 3, 0.0 + 4, 320 + 5, 240 + 6,
                                     1e-3 + 7, 2.0*1e-3 + 8, 3.0*1e-3 + 9, 4.0*1e-3 + 10, 0.1 + 1)
        K = gtsam.Cal3Unified(100, 105, 0.0, 320, 240,
                              1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3, 0.1)
        d = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10, 1], order='F')
        actual = K.retract(d)
        self.gtsamAssertEquals(actual, expected)
        np.testing.assert_allclose(d, K.localCoordinates(actual))


if __name__ == "__main__":
    unittest.main()
