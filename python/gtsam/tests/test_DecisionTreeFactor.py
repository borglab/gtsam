"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for DecisionTreeFactors.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

from gtsam import (DecisionTreeFactor, DiscreteDistribution, DiscreteValues,
                   Ordering)
from gtsam.utils.test_case import GtsamTestCase


class TestDecisionTreeFactor(GtsamTestCase):
    """Tests for DecisionTreeFactors."""

    def setUp(self):
        self.A = (12, 3)
        self.B = (5, 2)
        self.factor = DecisionTreeFactor([self.A, self.B], "1 2  3 4  5 6")

    def test_from_floats(self):
        """Test whether we can construct a factor from floats."""
        actual = DecisionTreeFactor([self.A, self.B], [1., 2.,  3., 4.,  5., 6.])
        self.gtsamAssertEquals(actual, self.factor)

    def test_enumerate(self):
        """Test whether we can enumerate the factor."""
        actual = self.factor.enumerate()
        _, values = zip(*actual)
        self.assertEqual(list(values), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_multiplication(self):
        """Test whether multiplication works with overloading."""
        v0 = (0, 2)
        v1 = (1, 2)
        v2 = (2, 2)

        # Multiply with a DiscreteDistribution, i.e., Bayes Law!
        prior = DiscreteDistribution(v1, [1, 3])
        f1 = DecisionTreeFactor([v0, v1], "1 2 3 4")
        expected = DecisionTreeFactor([v0, v1], "0.25 1.5 0.75 3")
        self.gtsamAssertEquals(DecisionTreeFactor(prior) * f1, expected)
        self.gtsamAssertEquals(f1 * prior, expected)

        # Multiply two factors
        f2 = DecisionTreeFactor([v1, v2], "5 6 7 8")
        actual = f1 * f2
        expected2 = DecisionTreeFactor([v0, v1, v2], "5 6 14 16 15 18 28 32")
        self.gtsamAssertEquals(actual, expected2)

    def test_methods(self):
        """Test whether we can call methods in python."""
        # double operator()(const DiscreteValues& values) const;
        values = DiscreteValues()
        values[self.A[0]] = 0
        values[self.B[0]] = 0
        self.assertIsInstance(self.factor(values), float)

        # size_t cardinality(Key j) const;
        self.assertIsInstance(self.factor.cardinality(self.A[0]), int)

        # DecisionTreeFactor operator/(const DecisionTreeFactor& f) const;
        self.assertIsInstance(self.factor / self.factor, DecisionTreeFactor)

        # DecisionTreeFactor* sum(size_t nrFrontals) const;
        self.assertIsInstance(self.factor.sum(1), DecisionTreeFactor)

        # DecisionTreeFactor* sum(const Ordering& keys) const;
        ordering = Ordering()
        ordering.push_back(self.A[0])
        self.assertIsInstance(self.factor.sum(ordering), DecisionTreeFactor)

        # DecisionTreeFactor* max(size_t nrFrontals) const;
        self.assertIsInstance(self.factor.max(1), DecisionTreeFactor)

    def test_markdown(self):
        """Test whether the _repr_markdown_ method."""

        expected = \
            "|A|B|value|\n" \
            "|:-:|:-:|:-:|\n" \
            "|0|0|1|\n" \
            "|0|1|2|\n" \
            "|1|0|3|\n" \
            "|1|1|4|\n" \
            "|2|0|5|\n" \
            "|2|1|6|\n"

        def formatter(x: int):
            return "A" if x == 12 else "B"

        actual = self.factor._repr_markdown_(formatter)
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
