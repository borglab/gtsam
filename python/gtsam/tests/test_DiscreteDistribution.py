"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Priors.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

import numpy as np
from gtsam import DecisionTreeFactor, DiscreteKeys, DiscreteDistribution
from gtsam.utils.test_case import GtsamTestCase

X = 0, 2


class TestDiscreteDistribution(GtsamTestCase):
    """Tests for Discrete Priors."""

    def test_constructor(self):
        """Test various constructors."""
        keys = DiscreteKeys()
        keys.push_back(X)
        f = DecisionTreeFactor(keys, "0.4 0.6")
        expected = DiscreteDistribution(f)

        actual = DiscreteDistribution(X, "2/3")
        self.gtsamAssertEquals(actual, expected)

        actual2 = DiscreteDistribution(X, [0.4, 0.6])
        self.gtsamAssertEquals(actual2, expected)

    def test_operator(self):
        prior = DiscreteDistribution(X, "2/3")
        self.assertAlmostEqual(prior(0), 0.4)
        self.assertAlmostEqual(prior(1), 0.6)

    def test_pmf(self):
        prior = DiscreteDistribution(X, "2/3")
        expected = np.array([0.4, 0.6])
        np.testing.assert_allclose(expected, prior.pmf())

    def test_sample(self):
        prior = DiscreteDistribution(X, "2/3")
        actual = prior.sample()
        self.assertIsInstance(actual, int)

    def test_markdown(self):
        """Test the _repr_markdown_ method."""

        prior = DiscreteDistribution(X, "2/3")
        expected = " *P(0):*\n\n" \
            "|0|value|\n" \
            "|:-:|:-:|\n" \
            "|0|0.4|\n" \
            "|1|0.6|\n" \

        actual = prior._repr_markdown_()
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
