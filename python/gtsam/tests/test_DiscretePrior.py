"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Priors.
Author: Varun Agrawal
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

import numpy as np
from gtsam import DecisionTreeFactor, DiscreteKeys, DiscretePrior
from gtsam.utils.test_case import GtsamTestCase

X = 0, 2


class TestDiscretePrior(GtsamTestCase):
    """Tests for Discrete Priors."""

    def test_constructor(self):
        """Test various constructors."""
        actual = DiscretePrior(X, "2/3")
        keys = DiscreteKeys()
        keys.push_back(X)
        f = DecisionTreeFactor(keys, "0.4 0.6")
        expected = DiscretePrior(f)
        self.gtsamAssertEquals(actual, expected)

    def test_operator(self):
        prior = DiscretePrior(X, "2/3")
        self.assertAlmostEqual(prior(0), 0.4)
        self.assertAlmostEqual(prior(1), 0.6)

    def test_pmf(self):
        prior = DiscretePrior(X, "2/3")
        expected = np.array([0.4, 0.6])
        np.testing.assert_allclose(expected, prior.pmf())

    def test_markdown(self):
        """Test the _repr_markdown_ method."""

        prior = DiscretePrior(X, "2/3")
        expected = " *P(0)*:\n\n" \
            "|0|value|\n" \
            "|:-:|:-:|\n" \
            "|0|0.4|\n" \
            "|1|0.6|\n" \

        actual = prior._repr_markdown_()
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
