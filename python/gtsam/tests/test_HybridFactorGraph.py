"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Factor Graphs.
Author: Fan Jiang
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.symbol_shorthand import C, M, X, Z
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (DiscreteConditional, DiscreteKeys, GaussianConditional,
                   GaussianMixture, GaussianMixtureFactor,
                   HybridGaussianFactorGraph, JacobianFactor, Ordering,
                   noiseModel)


class TestHybridGaussianFactorGraph(GtsamTestCase):
    """Unit tests for HybridGaussianFactorGraph."""
    def test_create(self):
        """Test construction of hybrid factor graph."""
        model = noiseModel.Unit.Create(3)
        dk = DiscreteKeys()
        dk.push_back((C(0), 2))

        jf1 = JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)), model)
        jf2 = JacobianFactor(X(0), np.eye(3), np.ones((3, 1)), model)

        gmf = GaussianMixtureFactor([X(0)], dk, [jf1, jf2])

        hfg = HybridGaussianFactorGraph()
        hfg.push_back(jf1)
        hfg.push_back(jf2)
        hfg.push_back(gmf)

        hbn = hfg.eliminateSequential(
            Ordering.ColamdConstrainedLastHybridGaussianFactorGraph(
                hfg, [C(0)]))

        self.assertEqual(hbn.size(), 2)

        mixture = hbn.at(0).inner()
        self.assertIsInstance(mixture, GaussianMixture)
        self.assertEqual(len(mixture.keys()), 2)

        discrete_conditional = hbn.at(hbn.size() - 1).inner()
        self.assertIsInstance(discrete_conditional, DiscreteConditional)

    def test_optimize(self):
        """Test construction of hybrid factor graph."""
        model = noiseModel.Unit.Create(3)
        dk = DiscreteKeys()
        dk.push_back((C(0), 2))

        jf1 = JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)), model)
        jf2 = JacobianFactor(X(0), np.eye(3), np.ones((3, 1)), model)

        gmf = GaussianMixtureFactor([X(0)], dk, [jf1, jf2])

        hfg = HybridGaussianFactorGraph()
        hfg.push_back(jf1)
        hfg.push_back(jf2)
        hfg.push_back(gmf)

        dtf = gtsam.DecisionTreeFactor([(C(0), 2)], "0 1")
        hfg.push_back(dtf)

        hbn = hfg.eliminateSequential(
            Ordering.ColamdConstrainedLastHybridGaussianFactorGraph(
                hfg, [C(0)]))

        hv = hbn.optimize()
        self.assertEqual(hv.atDiscrete(C(0)), 1)

    @staticmethod
    def tiny(num_measurements: int = 1) -> gtsam.HybridBayesNet:
        """
        Create a tiny two variable hybrid model which represents
        the generative probability P(z, x, n) = P(z | x, n)P(x)P(n).
        """
        # Create hybrid Bayes net.
        bayesNet = gtsam.HybridBayesNet()

        # Create mode key: 0 is low-noise, 1 is high-noise.
        mode = (M(0), 2)

        # Create Gaussian mixture Z(0) = X(0) + noise for each measurement.
        I = np.eye(1)
        keys = DiscreteKeys()
        keys.push_back(mode)
        for i in range(num_measurements):
            conditional0 = GaussianConditional.FromMeanAndStddev(Z(i),
                                                                 I,
                                                                 X(0), [0],
                                                                 sigma=0.5)
            conditional1 = GaussianConditional.FromMeanAndStddev(Z(i),
                                                                 I,
                                                                 X(0), [0],
                                                                 sigma=3)
            bayesNet.emplaceMixture([Z(i)], [X(0)], keys,
                                    [conditional0, conditional1])

        # Create prior on X(0).
        prior_on_x0 = GaussianConditional.FromMeanAndStddev(X(0), [5.0], 5.0)
        bayesNet.addGaussian(prior_on_x0)

        # Add prior on mode.
        bayesNet.emplaceDiscrete(mode, "1/1")

        return bayesNet

    def test_tiny(self):
        """Test a tiny two variable hybrid model."""
        bayesNet = self.tiny()
        sample = bayesNet.sample()
        # print(sample)

        # Create a factor graph from the Bayes net with sampled measurements.
        fg = HybridGaussianFactorGraph()
        conditional = bayesNet.atMixture(0)
        measurement = gtsam.VectorValues()
        measurement.insert(Z(0), sample.at(Z(0)))
        factor = conditional.likelihood(measurement)
        fg.push_back(factor)
        fg.push_back(bayesNet.atGaussian(1))
        fg.push_back(bayesNet.atDiscrete(2))

        self.assertEqual(fg.size(), 3)

    def test_ratio(self):
        """
        Given a tiny two variable hybrid model, with 2 measurements,
        test the ratio of the bayes net model representing P(z, x, n)=P(z|x, n)P(x)P(n)
        and the factor graph P(x, n | z)=P(x | n, z)P(n|z),
        both of which represent the same posterior.
        """
        # Create the Bayes net representing the generative model P(z, x, n)=P(z|x, n)P(x)P(n)
        bayesNet = self.tiny(num_measurements=2)
        # Sample from the Bayes net.
        sample: gtsam.HybridValues = bayesNet.sample()
        # print(sample)

        # Create a factor graph from the Bayes net with sampled measurements.
        # The factor graph is `P(x)P(n) ϕ(x, n; z1) ϕ(x, n; z2)`
        # and thus represents the same joint probability as the Bayes net.
        fg = HybridGaussianFactorGraph()
        for i in range(2):
            conditional = bayesNet.atMixture(i)
            measurement = gtsam.VectorValues()
            measurement.insert(Z(i), sample.at(Z(i)))
            factor = conditional.likelihood(measurement)
            fg.push_back(factor)
        fg.push_back(bayesNet.atGaussian(2))
        fg.push_back(bayesNet.atDiscrete(3))

        self.assertEqual(fg.size(), 4)

        # Calculate ratio between Bayes net probability and the factor graph:
        continuousValues = gtsam.VectorValues()
        continuousValues.insert(X(0), sample.at(X(0)))
        discreteValues = sample.discrete()
        expected_ratio = bayesNet.evaluate(sample) / fg.probPrime(
            continuousValues, discreteValues)
        #TODO(Varun) This should be 1. Adding the normalizing factor should fix fg.probPrime
        print(expected_ratio)

        # TODO(dellaert): Change the mode to 0 and calculate the ratio again.


if __name__ == "__main__":
    unittest.main()
