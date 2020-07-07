"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

FrobeniusFactor unit tests.
Author: Frank Dellaert
"""
# pylint: disable=no-name-in-module, import-error, invalid-name
import unittest

import numpy as np
from gtsam import (Rot3, SO3, SO4, FrobeniusBetweenFactorSO4, FrobeniusFactorSO4,
                   FrobeniusWormholeFactor, SOn)

id = SO4()
v1 = np.array([0, 0, 0, 0.1, 0, 0])
Q1 = SO4.Expmap(v1)
v2 = np.array([0, 0, 0, 0.01, 0.02, 0.03])
Q2 = SO4.Expmap(v2)


class TestFrobeniusFactorSO4(unittest.TestCase):
    """Test FrobeniusFactor factors."""

    def test_frobenius_factor(self):
        """Test creation of a factor that calculates the Frobenius norm."""
        factor = FrobeniusFactorSO4(1, 2)
        actual = factor.evaluateError(Q1, Q2)
        expected = (Q2.matrix()-Q1.matrix()).transpose().reshape((16,))
        np.testing.assert_array_equal(actual, expected)

    def test_frobenius_between_factor(self):
        """Test creation of a Frobenius BetweenFactor."""
        factor = FrobeniusBetweenFactorSO4(1, 2, Q1.between(Q2))
        actual = factor.evaluateError(Q1, Q2)
        expected = np.zeros((16,))
        np.testing.assert_array_almost_equal(actual, expected)

    def test_frobenius_wormhole_factor(self):
        """Test creation of a factor that calculates Shonan error."""
        R1 = SO3.Expmap(v1[3:])
        R2 = SO3.Expmap(v2[3:])
        factor = FrobeniusWormholeFactor(1, 2, Rot3(R1.between(R2).matrix()), p=4)
        I4 = SOn(4)
        Q1 = I4.retract(v1)
        Q2 = I4.retract(v2)
        actual = factor.evaluateError(Q1, Q2)
        expected = np.zeros((12,))
        np.testing.assert_array_almost_equal(actual, expected, decimal=4)


if __name__ == "__main__":
    unittest.main()
