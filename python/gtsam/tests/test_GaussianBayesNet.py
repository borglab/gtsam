"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Gaussian Bayes Nets.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import GaussianBayesNet, GaussianConditional

# some keys
_x_ = 11
_y_ = 22
_z_ = 33

I_1x1 = np.eye(1, dtype=float)


def smallBayesNet():
    """Create a small Bayes Net for testing"""
    bayesNet = GaussianBayesNet()
    bayesNet.push_back(GaussianConditional(_x_, [9.0], I_1x1, _y_, I_1x1))
    bayesNet.push_back(GaussianConditional(_y_, [5.0], I_1x1))
    return bayesNet


class TestGaussianBayesNet(GtsamTestCase):
    """Tests for Gaussian Bayes nets."""

    def test_matrix(self):
        """Test matrix method"""
        R, d = smallBayesNet().matrix()  # get matrix and RHS
        R1 = np.array([[1.0, 1.0], [0.0, 1.0]])
        d1 = np.array([9.0, 5.0])
        np.testing.assert_equal(R, R1)
        np.testing.assert_equal(d, d1)

    def test_evaluate(self):
        """Test evaluate method"""
        bayesNet = smallBayesNet()
        values = gtsam.VectorValues()
        values.insert(_x_, np.array([9.0]))
        values.insert(_y_, np.array([5.0]))
        for i in [0, 1]:
            self.assertAlmostEqual(
                bayesNet.at(i).logProbability(values),
                np.log(bayesNet.at(i).evaluate(values)))
        self.assertAlmostEqual(bayesNet.logProbability(values),
                               np.log(bayesNet.evaluate(values)))

    def test_sample(self):
        """Test sample method"""
        bayesNet = smallBayesNet()
        sample = bayesNet.sample()
        self.assertIsInstance(sample, gtsam.VectorValues)

        # standard deviation is 1.0 for both, so we set tolerance to 3*sigma
        mean = bayesNet.optimize()
        self.gtsamAssertEquals(sample, mean, tol=3.0)

        # Sample with rng
        rng = gtsam.MT19937(42)
        conditional = GaussianConditional(_x_, [9.0], I_1x1)
        # Sample multiple times and average to get mean
        val = 0
        niters = 10000
        for _ in range(niters):
            val += conditional.sample(rng).at(_x_).item()
        self.assertAlmostEqual(val / niters, 9.0, 1)


if __name__ == "__main__":
    unittest.main()
