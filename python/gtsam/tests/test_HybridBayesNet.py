"""
GTSAM Copyright 2010-2022, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Values.
Author: Frank Dellaert
"""

# pylint: disable=invalid-name, no-name-in-module, no-member, E1101

import math
import unittest

import numpy as np
from gtsam.symbol_shorthand import A, X
from gtsam.utils.test_case import GtsamTestCase

from gtsam import (
    AlgebraicDecisionTreeKey,
    AssignmentKey,
    DecisionTreeFactor,
    DiscreteBayesNet,
    DiscreteConditional,
    DiscreteValues,
    GaussianBayesNet,
    GaussianConditional,
    HybridBayesNet,
    HybridGaussianConditional,
    HybridGaussianFactorGraph,
    HybridValues,
    KeySet,
    VectorValues,
    noiseModel,
)


class TestHybridBayesNet(GtsamTestCase):
    """Unit tests for HybridBayesNet."""

    def setUp(self):
        """Set up a common HybridBayesNet for testing P(X0|X1) P(X1|Asia) P(Asia)."""
        self.Asia = (A(0), 2)

        self.I_1x1 = np.eye(1)
        self.conditional_X0_X1 = GaussianConditional.FromMeanAndStddev(
            X(0), 2 * self.I_1x1, X(1), [-4], 5.0
        )

        self.model0 = noiseModel.Diagonal.Sigmas([2.0])
        self.model1 = noiseModel.Diagonal.Sigmas([3.0])

        self.conditional0_X1_A = GaussianConditional(X(1), [5], self.I_1x1, self.model0)
        self.conditional1_X1_A1 = GaussianConditional(
            X(1), [2], self.I_1x1, self.model1
        )

        self.hybrid_conditional_X1_Asia = HybridGaussianConditional(
            self.Asia, [self.conditional0_X1_A, self.conditional1_X1_A1]
        )

        self.conditional_Asia = DiscreteConditional(self.Asia, "99/1")

        self.bayesNet = HybridBayesNet()
        self.bayesNet.push_back(self.conditional_X0_X1)
        self.bayesNet.push_back(self.hybrid_conditional_X1_Asia)
        self.bayesNet.push_back(self.conditional_Asia)

        self.values = HybridValues()
        continuous = VectorValues()
        continuous.insert(X(0), [-6])
        continuous.insert(X(1), [1])
        self.values.insert(continuous)
        discrete = DiscreteValues()
        discrete[A(0)] = 0
        self.values.insert(discrete)

    def test_constructor_and_basic_props(self):
        """Test constructor, empty(), size(), keys(), print(), dot()."""
        bn_empty = HybridBayesNet()
        self.assertTrue(bn_empty.empty())
        self.assertEqual(bn_empty.size(), 0)

        self.assertFalse(self.bayesNet.empty())
        self.assertEqual(self.bayesNet.size(), 3)

        keys = self.bayesNet.keys()
        self.assertIsInstance(keys, KeySet)
        self.assertTrue(X(0) in keys)
        self.assertTrue(X(1) in keys)
        self.assertTrue(A(0) in keys)
        self.assertEqual(keys.size(), 3)

        # Test dot (returns a string)
        self.assertIsInstance(self.bayesNet.dot(), str)

    def test_equals_method(self):
        """Test the equals(HybridBayesNet) method."""
        bn_copy = HybridBayesNet()
        bn_copy.push_back(self.conditional_X0_X1)
        bn_copy.push_back(self.hybrid_conditional_X1_Asia)
        bn_copy.push_back(self.conditional_Asia)
        self.assertTrue(self.bayesNet.equals(bn_copy, 1e-9))

        bn_different_order = HybridBayesNet()  # Order matters for BayesNets
        bn_different_order.push_back(self.conditional_Asia)  # Different order
        bn_different_order.push_back(self.conditional_X0_X1)
        bn_different_order.push_back(self.hybrid_conditional_X1_Asia)
        self.assertFalse(self.bayesNet.equals(bn_different_order, 1e-9))

        bn_different_cond = HybridBayesNet()
        bn_different_cond.push_back(self.conditional_X0_X1)
        bn_different_cond.push_back(self.hybrid_conditional_X1_Asia)
        # Different P(Asia)
        bn_different_cond.push_back(DiscreteConditional(self.Asia, "5/5"))
        self.assertFalse(self.bayesNet.equals(bn_different_cond, 1e-9))

    def test_error_method(self):
        """Test the error(HybridValues) method of HybridBayesNet."""
        # logProbability(x) = -(K + error(x)) with K = -log(k)
        self.assertAlmostEqual(
            self.bayesNet.negLogConstant() + self.bayesNet.error(self.values),
            -self.bayesNet.logProbability(self.values),
            places=5,
        )

    def test_evaluate(self):
        """Test evaluate for a hybrid Bayes net P(X0|X1) P(X1|Asia) P(Asia)."""
        # Original test_evaluate content
        conditionalProbability = self.conditional_X0_X1.evaluate(
            self.values.continuous()
        )
        # For HybridGaussianConditional, we need to evaluate its chosen component
        # given the discrete assignment in self.values
        chosen_gaussian_conditional_X1 = self.hybrid_conditional_X1_Asia.choose(
            self.values.discrete()
        )
        mixtureProbability = chosen_gaussian_conditional_X1.evaluate(
            self.values.continuous()
        )
        discreteProbability = self.conditional_Asia.evaluate(self.values.discrete())

        self.assertAlmostEqual(
            conditionalProbability * mixtureProbability * discreteProbability,
            self.bayesNet.evaluate(self.values),
            places=5,
        )

        # Check logProbability
        self.assertAlmostEqual(
            self.bayesNet.logProbability(self.values),
            math.log(self.bayesNet.evaluate(self.values)),
            places=5,
        )

        # Check invariance for all conditionals:
        self.check_invariance(
            self.bayesNet.at(0).asGaussian(), self.values.continuous()
        )
        self.check_invariance(self.bayesNet.at(0).asGaussian(), self.values)
        self.check_invariance(self.bayesNet.at(0), self.values)

        self.check_invariance(self.bayesNet.at(1), self.values)

        self.check_invariance(self.bayesNet.at(2).asDiscrete(), self.values.discrete())
        self.check_invariance(self.bayesNet.at(2).asDiscrete(), self.values)
        self.check_invariance(self.bayesNet.at(2), self.values)

    def check_invariance(self, conditional, values_obj):
        """Check invariance for given conditional."""
        probability = conditional.evaluate(values_obj)
        self.assertTrue(probability >= 0.0)
        logProb = conditional.logProbability(values_obj)
        if probability > 1e-9:  # Avoid issues with log(0)
            self.assertAlmostEqual(
                probability, np.exp(logProb), delta=probability * 1e-6
            )  # Relative delta

        expected = -(conditional.negLogConstant() + conditional.error(values_obj))
        self.assertAlmostEqual(logProb, expected, places=5)

    def test_mpe_and_optimize_methods(self):
        """Test mpe(), optimize(), and optimize(DiscreteValues)."""
        # MPE: Most Probable Explanation for discrete variables
        mpe_assignment = self.bayesNet.mpe()
        self.assertIsInstance(mpe_assignment, DiscreteValues)
        # Given P(Asia) = "99/1", Asia=0 is more probable
        self.assertEqual(mpe_assignment[A(0)], 0)

        # Optimize(): MAP for discrete, then optimize continuous given MAP discrete
        map_solution = self.bayesNet.optimize()
        self.assertIsInstance(map_solution, HybridValues)
        self.assertEqual(map_solution.atDiscrete(A(0)), 0)  # Asia=0
        # If Asia=0, X1 from conditional0_X1_A: N(mean=[5], model0_sigmas=[2.0]) -> optimal X1=5
        self.assertAlmostEqual(map_solution.at(X(1))[0], 5.0, places=5)
        # If X1=5, X0 from conditional_X0_X1: N(mean=2*5-4=6, stddev=5.0) -> optimal X0=6
        self.assertAlmostEqual(map_solution.at(X(0))[0], 6.0, places=5)

        # Optimize(DiscreteValues): optimize continuous given fixed discrete
        discrete_choice = DiscreteValues()
        discrete_choice[A(0)] = 1  # Fix Asia=1 (less likely choice)
        optimized_continuous = self.bayesNet.optimize(discrete_choice)
        self.assertIsInstance(optimized_continuous, VectorValues)
        # If Asia=1, X1 from conditional1_X1_A1: N(mean=[2], model1_sigmas=[3.0]) -> optimal X1=2
        self.assertAlmostEqual(optimized_continuous.at(X(1))[0], 2.0, places=5)
        # If X1=2, X0 from conditional_X0_X1: N(mean=2*2-4=0, stddev=5.0) -> optimal X0=0
        self.assertAlmostEqual(optimized_continuous.at(X(0))[0], 0.0, places=5)

    def test_sampling_methods(self):
        """Test sample() and sample(HybridValues)."""
        # sample()
        full_sample = self.bayesNet.sample()
        self.assertIsInstance(full_sample, HybridValues)
        self.assertTrue(full_sample.existsDiscrete(A(0)))
        self.assertTrue(full_sample.existsVector(X(0)))
        self.assertTrue(full_sample.existsVector(X(1)))
        self.assertIn(full_sample.atDiscrete(A(0)), [0, 1])

        # sample(HybridValues) - conditional sampling
        given_values = HybridValues()
        discrete_given = DiscreteValues()
        discrete_given[A(0)] = 0  # Condition on Asia=0
        given_values.insert(discrete_given)

        conditional_sample = self.bayesNet.sample(given_values)
        self.assertIsInstance(conditional_sample, HybridValues)
        self.assertEqual(
            conditional_sample.atDiscrete(A(0)), 0
        )  # Should respect condition
        self.assertTrue(conditional_sample.existsVector(X(0)))
        self.assertTrue(conditional_sample.existsVector(X(1)))

    def test_marginals_and_choice_methods(self):
        """Test discreteMarginal() and choose(DiscreteValues)."""
        # discreteMarginal() -> DiscreteBayesNet P(M)
        discrete_marginal_bn = self.bayesNet.discreteMarginal()
        self.assertIsInstance(discrete_marginal_bn, DiscreteBayesNet)
        # Our BN has only one discrete var Asia, so marginal is just P(Asia)
        self.assertEqual(discrete_marginal_bn.size(), 1)
        d_cond = discrete_marginal_bn.at(0)  # Should be P(Asia)
        self.gtsamAssertEquals(d_cond, self.conditional_Asia, 1e-9)

        # choose(DiscreteValues) -> GaussianBayesNet P(X | M=m)
        fixed_discrete = DiscreteValues()
        fixed_discrete[A(0)] = 0  # Fix Asia = 0
        gaussian_bn_given_asia0 = self.bayesNet.choose(fixed_discrete)
        self.assertIsInstance(gaussian_bn_given_asia0, GaussianBayesNet)
        # Should contain P(X0|X1) and P(X1|Asia=0)
        # The order is important for BayesNets, parents are always *last*
        self.assertEqual(gaussian_bn_given_asia0.size(), 2)
        self.gtsamAssertEquals(
            gaussian_bn_given_asia0.at(1), self.conditional0_X1_A, 1e-9
        )  # P(X1|A=0)
        self.gtsamAssertEquals(
            gaussian_bn_given_asia0.at(0), self.conditional_X0_X1, 1e-9
        )  # P(X0|X1)

    def test_errorTree(self):
        """Test errorTree(VectorValues)."""
        vector_values = self.values.continuous()

        # errorTree(VectorValues) -> unnormalized log P(M | X=continuous_values)
        error_tree = self.bayesNet.errorTree(vector_values)
        self.assertIsInstance(error_tree, AlgebraicDecisionTreeKey)

        # Get the errorTree for X1 given Asia=0 (key A(0)):
        error_tree_X1_A = self.hybrid_conditional_X1_Asia.errorTree(vector_values)

        # For Asia=0 (key A(0)) from self.values:
        assignment_Asia0 = AssignmentKey()
        assignment_Asia0[A(0)] = 0
        # self.values has Asia=0
        error_Asia0 = self.conditional_Asia.error(self.values.discrete())
        expected_error_Asia0 = (
            self.conditional_X0_X1.error(vector_values)
            + error_tree_X1_A(assignment_Asia0)
            + error_Asia0
        )
        self.assertAlmostEqual(
            error_tree(assignment_Asia0), expected_error_Asia0, places=4
        )

        # For Asia=1:
        # Need error(A=1) from P(Asia)="99/1" -> P(A=1)=0.01
        dv = DiscreteValues()
        dv[A(0)] = 1
        error_Asia1 = self.conditional_Asia.error(dv)
        assignment_Asia1 = AssignmentKey()
        assignment_Asia1[A(0)] = 1
        expected_error_Asia1 = (
            self.conditional_X0_X1.error(vector_values)
            + error_tree_X1_A(assignment_Asia1)
            + error_Asia1
        )
        self.assertAlmostEqual(
            error_tree(assignment_Asia1), expected_error_Asia1, places=4
        )

    def test_errorTree_discretePosterior(self):
        """Test discretePosterior(VectorValues)."""
        vector_values = self.values.continuous()
        posterior_tree = self.bayesNet.discretePosterior(vector_values)
        self.assertIsInstance(posterior_tree, AlgebraicDecisionTreeKey)

    def test_pruning_methods(self):
        """Test prune(maxNrLeaves) and prune(maxNrLeaves, marginalThreshold)."""
        # Prune to max 1 leaf (most likely path)
        pruned_bn_1leaf = self.bayesNet.prune(1)
        self.assertIsInstance(pruned_bn_1leaf, HybridBayesNet)
        # The discrete part should now be deterministic for Asia=0
        # TODO(frank): why does it not become a conditional?
        mpe_pruned = pruned_bn_1leaf.mpe()
        self.assertEqual(mpe_pruned[A(0)], 0)
        # The discrete conditional for Asia should reflect P(Asia=0)=1.0
        # TODO(frank): why is there no dead-mode removal here?
        actual = pruned_bn_1leaf.discreteMarginal().at(0).evaluate(mpe_pruned)
        self.assertAlmostEqual(actual, 1.0, places=5)

        # Prune with marginalThreshold
        # P(Asia=0)=0.99, P(Asia=1)=0.01
        # Threshold 0.5 should keep only Asia=0 branch
        pruned_bn_thresh = self.bayesNet.prune(
            2, marginalThreshold=0.5
        )  # maxNrLeaves=2 allows both if above thresh
        # TODO(Frank): I don't understand *how* discrete got cut here.
        self.assertEqual(pruned_bn_thresh.size(), 2)  # Should keep both conditionals

    def test_toFactorGraph_method(self):
        """Test toFactorGraph(VectorValues) method."""
        # Create measurements for conditioning
        measurements = VectorValues()
        measurements.insert(X(0), [-5.0])  # Example measurement for X(0)

        hfg = self.bayesNet.toFactorGraph(measurements)
        self.assertIsInstance(hfg, HybridGaussianFactorGraph)
        self.assertEqual(hfg.size(), self.bayesNet.size())


if __name__ == "__main__":
    unittest.main()
