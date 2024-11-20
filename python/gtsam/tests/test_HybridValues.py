"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Values.
Author: Shangjie Xue
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import numpy as np
from gtsam.symbol_shorthand import C, X
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestHybridValues(GtsamTestCase):
    """Unit tests for HybridValues."""

    def test_basic(self):
        """Test construction and basic methods of hybrid values."""

        hv1 = gtsam.HybridValues()
        hv1.insert(X(0), np.ones((3, 1)))
        hv1.insert(C(0), 2)

        hv2 = gtsam.HybridValues()
        hv2.insert(C(0), 2)
        hv2.insert(X(0), np.ones((3, 1)))

        self.assertEqual(hv1.atDiscrete(C(0)), 2)
        self.assertEqual(hv1.at(X(0))[0], np.ones((3, 1))[0])


if __name__ == "__main__":
    unittest.main()
