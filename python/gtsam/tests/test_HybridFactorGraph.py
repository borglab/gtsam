"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Factor Graphs.
Author: Fan Jiang, Varun Agrawal, Frank Dellaert
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.symbol_shorthand import C, M, X, Z
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (DiscreteConditional, DiscreteKeys, GaussianConditional,
                   GaussianMixture, GaussianMixtureFactor, HybridBayesNet, HybridValues,
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
    def tiny(num_measurements: int = 1) -> HybridBayesNet:
        """
        Create a tiny two variable hybrid model which represents
        the generative probability P(z, x, n) = P(z | x, n)P(x)P(n).
        """
        # Create hybrid Bayes net.
        bayesNet = HybridBayesNet()

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
        bayesNet.emplaceDiscrete(mode, "4/6")

        return bayesNet

    @staticmethod
    def measurements(sample: HybridValues, indices) -> gtsam.VectorValues:
        """Create measurements from a sample, grabbing Z(i) where i in indices."""
        measurements = gtsam.VectorValues()
        for i in indices:
            measurements.insert(Z(i), sample.at(Z(i)))
        return measurements

    @classmethod
    def factor_graph_from_bayes_net(cls, bayesNet: HybridBayesNet, sample: HybridValues):
        """Create a factor graph from the Bayes net with sampled measurements.
            The factor graph is `P(x)P(n) ϕ(x, n; z0) ϕ(x, n; z1) ...`
            and thus represents the same joint probability as the Bayes net.
        """
        fg = HybridGaussianFactorGraph()
        num_measurements = bayesNet.size() - 2
        for i in range(num_measurements):
            conditional = bayesNet.atMixture(i)
            factor = conditional.likelihood(cls.measurements(sample, [i]))
            fg.push_back(factor)
        fg.push_back(bayesNet.atGaussian(num_measurements))
        fg.push_back(bayesNet.atDiscrete(num_measurements+1))
        return fg

    @classmethod
    def estimate_marginals(cls, bayesNet: HybridBayesNet, sample: HybridValues, N=10000):
        """Do importance sampling to get an estimate of the discrete marginal P(mode)."""
        # Use prior on x0, mode as proposal density.
        prior = cls.tiny(num_measurements=0)  # just P(x0)P(mode)

        # Allocate space for marginals.
        marginals = np.zeros((2,))

        # Do importance sampling.
        num_measurements = bayesNet.size() - 2
        measurements = cls.measurements(sample, range(num_measurements))
        for s in range(N):
            proposed = prior.sample()
            proposed.insert(measurements)
            weight = bayesNet.evaluate(proposed) / prior.evaluate(proposed)
            marginals[proposed.atDiscrete(M(0))] += weight

        # print marginals:
        marginals /= marginals.sum()
        return marginals

    def test_tiny(self):
        """Test a tiny two variable hybrid model."""
        bayesNet = self.tiny()
        sample = bayesNet.sample()
        # print(sample)

        # Estimate marginals using importance sampling.
        marginals = self.estimate_marginals(bayesNet, sample)
        # print(f"True mode: {sample.atDiscrete(M(0))}")
        # print(f"P(mode=0; z0) = {marginals[0]}")
        # print(f"P(mode=1; z0) = {marginals[1]}")

        # Check that the estimate is close to the true value.
        self.assertAlmostEqual(marginals[0], 0.4, delta=0.1)
        self.assertAlmostEqual(marginals[1], 0.6, delta=0.1)

        fg = self.factor_graph_from_bayes_net(bayesNet, sample)
        self.assertEqual(fg.size(), 3)

    @staticmethod
    def calculate_ratio(bayesNet: HybridBayesNet,
                        fg: HybridGaussianFactorGraph,
                        sample: HybridValues):
        """Calculate ratio  between Bayes net probability and the factor graph."""
        return bayesNet.evaluate(sample) / fg.probPrime(sample) if fg.probPrime(sample) > 0 else 0

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
        sample: HybridValues = bayesNet.sample()
        # print(sample)

        # Estimate marginals using importance sampling.
        marginals = self.estimate_marginals(bayesNet, sample)
        # print(f"True mode: {sample.atDiscrete(M(0))}")
        # print(f"P(mode=0; z0, z1) = {marginals[0]}")
        # print(f"P(mode=1; z0, z1) = {marginals[1]}")

        # Check marginals based on sampled mode.
        if sample.atDiscrete(M(0)) == 0:
            self.assertGreater(marginals[0], marginals[1])
        else:
            self.assertGreater(marginals[1], marginals[0])

        fg = self.factor_graph_from_bayes_net(bayesNet, sample)
        self.assertEqual(fg.size(), 4)

        # Create measurements from the sample.
        measurements = self.measurements(sample, [0, 1])

        # Calculate ratio between Bayes net probability and the factor graph:
        expected_ratio = self.calculate_ratio(bayesNet, fg, sample)
        # print(f"expected_ratio: {expected_ratio}\n")

        # Check with a number of other samples.
        for i in range(10):
            other = bayesNet.sample()
            other.update(measurements)
            ratio = self.calculate_ratio(bayesNet, fg, other)
            # print(f"Ratio: {ratio}\n")
            if (ratio > 0):
                self.assertAlmostEqual(ratio, expected_ratio)

        # Test elimination.
        ordering = gtsam.Ordering()
        ordering.push_back(X(0))
        ordering.push_back(M(0))
        posterior = fg.eliminateSequential(ordering)
        print(posterior)

        # Calculate ratio between Bayes net probability and the factor graph:
        expected_ratio = self.calculate_ratio(posterior, fg, sample)
        print(f"expected_ratio: {expected_ratio}\n")

        # Check with a number of other samples.
        for i in range(10):
            other = posterior.sample()
            other.insert(measurements)
            ratio = self.calculate_ratio(posterior, fg, other)
            print(f"Ratio: {ratio}\n")
            # if (ratio > 0):
            #     self.assertAlmostEqual(ratio, expected_ratio)

if __name__ == "__main__":
    unittest.main()
