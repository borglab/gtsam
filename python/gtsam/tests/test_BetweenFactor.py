"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

BetweenFactor unit tests.

Author: Varun Agrawal
"""
import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


class TestBetweenFactor(GtsamTestCase):
    def test_double(self):
        """Test BetweenFactorDouble."""
        noise_model = gtsam.noiseModel.Isotropic.Sigma(1, 1.0)
        factor = gtsam.BetweenFactorDouble(X(0), X(1), 1.0, noise_model)
        values = gtsam.Values()
        for i in range(2):
            values.insert(X(i), i)
        self.assertEqual(factor.error(values), 0)
        # self.assertEqual(K.fy(), 1.)


if __name__ == "__main__":
    unittest.main()
