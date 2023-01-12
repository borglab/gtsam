"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Values.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import math
import unittest

import numpy as np
from gtsam.symbol_shorthand import A, X
from gtsam.utils.test_case import GtsamTestCase

from gtsam import (DiscreteConditional, DiscreteKeys, GaussianConditional,
                   GaussianMixture, HybridBayesNet, HybridValues, noiseModel)


class TestHybridBayesNet(GtsamTestCase):
    """Unit tests for HybridValues."""

    def test_evaluate(self):
        """Test evaluate for a hybrid Bayes net P(X0|X1) P(X1|Asia) P(Asia)."""
        asiaKey = A(0)
        Asia = (asiaKey, 2)

        # Create the continuous conditional
        I_1x1 = np.eye(1)
        conditional = GaussianConditional.FromMeanAndStddev(X(0), 2 * I_1x1, X(1), [-4],
                                                            5.0)

        # Create the noise models
        model0 = noiseModel.Diagonal.Sigmas([2.0])
        model1 = noiseModel.Diagonal.Sigmas([3.0])

        # Create the conditionals
        conditional0 = GaussianConditional(X(1), [5], I_1x1, model0)
        conditional1 = GaussianConditional(X(1), [2], I_1x1, model1)
        discrete_keys = DiscreteKeys()
        discrete_keys.push_back(Asia)

        # Create hybrid Bayes net.
        bayesNet = HybridBayesNet()
        bayesNet.push_back(conditional)
        bayesNet.push_back(GaussianMixture(
            [X(1)], [], discrete_keys, [conditional0, conditional1]))
        bayesNet.push_back(DiscreteConditional(Asia, "99/1"))

        # Create values at which to evaluate.
        values = HybridValues()
        values.insert(asiaKey, 0)
        values.insert(X(0), [-6])
        values.insert(X(1), [1])

        conditionalProbability = conditional.evaluate(values.continuous())
        mixtureProbability = conditional0.evaluate(values.continuous())
        self.assertAlmostEqual(conditionalProbability * mixtureProbability *
                               0.99,
                               bayesNet.evaluate(values),
                               places=5)

        # Check logProbability
        self.assertAlmostEqual(bayesNet.logProbability(values),
                               math.log(bayesNet.evaluate(values)))


if __name__ == "__main__":
    unittest.main()
