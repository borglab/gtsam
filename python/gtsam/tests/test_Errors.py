"""Tests for dot products of factor-associated vectors.

GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
"""

import unittest

import numpy as np

import gtsam


class TestErrors(unittest.TestCase):
    """Errors uses Python lists with one NumPy vector per factor."""

    def test_dot_with_empty_blocks(self):
        """Empty factor slots contribute zero in a valid dot product."""
        a = [np.array([1., 2.]), np.empty(0), np.array([3.])]
        b = [np.array([4., 5.]), np.empty(0), np.array([6.])]
        self.assertEqual(gtsam.dot(a, b), 32.)
        self.assertEqual(gtsam.dot([], []), 0.)

    def test_dot_rejects_block_counts(self):
        """Missing and extra blocks raise ValueError in both argument orders."""
        a = [np.ones(1)]
        for b in ([], [np.ones(1), np.empty(0)]):
            for left, right in ((a, b), (b, a)):
                with self.subTest(left=len(left), right=len(right)):
                    with self.assertRaisesRegex(ValueError, "Errors::dot"):
                        gtsam.dot(left, right)

    def test_dot_rejects_block_dimensions(self):
        """Every corresponding vector pair must have the same dimension."""
        cases = (
            ([np.ones(2)], [np.ones(1)]),
            ([np.ones(1), np.ones(2)], [np.ones(1), np.ones(1)]),
            ([np.empty(0)], [np.ones(1)]),
        )
        for a, b in cases:
            for left, right in ((a, b), (b, a)):
                with self.subTest(dimensions=[v.size for v in left]):
                    with self.assertRaisesRegex(ValueError, "Errors::dot"):
                        gtsam.dot(left, right)


if __name__ == "__main__":
    unittest.main()
