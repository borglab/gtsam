"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Gaussian Bayes Nets.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import gtsam
import numpy as np
from gtsam import GaussianBayesNet, GaussianConditional
from gtsam.utils.test_case import GtsamTestCase

# some keys
_x_ = 11
_y_ = 22
_z_ = 33


def smallBayesNet():
    """Create a small Bayes Net for testing"""
    bayesNet = GaussianBayesNet()
    I_1x1 = np.eye(1, dtype=float)
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

    def test_sample(self):
        """Test sample method"""
        bayesNet = smallBayesNet()
        sample = bayesNet.sample()
        self.assertIsInstance(sample, gtsam.VectorValues)

        # standard deviation is 1.0 for both, so we set tolerance to 3*sigma
        mean = bayesNet.optimize()
        self.gtsamAssertEquals(sample, mean, tol=3.0)


if __name__ == "__main__":
    unittest.main()
