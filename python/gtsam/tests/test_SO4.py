"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SO4 unit tests.
Author: Frank Dellaert
"""
# pylint: disable=no-name-in-module, import-error
import unittest

import numpy as np
from gtsam import SO4

I4 = SO4()
v1 = np.array([0, 0, 0, .1, 0, 0])
v2 = np.array([0, 0, 0, 0.01, 0.02, 0.03])
Q1 = SO4.Expmap(v1)
Q2 = SO4.Expmap(v2)


class TestSO4(unittest.TestCase):
    """Test selected SO4 methods."""

    def test_constructor(self):
        """Construct from matrix."""
        matrix = np.eye(4)
        so4 = SO4(matrix)
        self.assertIsInstance(so4, SO4)

    def test_retract(self):
        """Test retraction to manifold."""
        v = np.zeros((6,), float)
        actual = I4.retract(v)
        self.assertTrue(actual.equals(I4, 1e-9))

    def test_local(self):
        """Check localCoordinates for trivial case."""
        v0 = np.zeros((6,), float)
        actual = I4.localCoordinates(I4)
        np.testing.assert_array_almost_equal(actual, v0)

    def test_compose(self):
        """Check compose works in subgroup."""
        expected = SO4.Expmap(2*v1)
        actual = Q1.compose(Q1)
        self.assertTrue(actual.equals(expected, 1e-9))

    def test_vec(self):
        """Check wrapping of vec."""
        expected = np.array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        actual = I4.vec()
        np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
