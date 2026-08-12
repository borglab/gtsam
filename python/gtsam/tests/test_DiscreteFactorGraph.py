"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Factor Graphs.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from dfg_utils import make_key, generate_transition_cpt, generate_observation_cpt

from gtsam import (
    DecisionTreeFactor,
    DiscreteConditional,
    DiscreteFactorGraph,
    DiscreteKeys,
    DiscreteValues,
    Ordering,
)

OrderingType = Ordering.OrderingType


class TestDiscreteFactorGraph(GtsamTestCase):
    """Tests for Discrete Factor Graphs."""

    def test_evaluation(self):
        """Test constructing and evaluating a discrete factor graph."""

        # Three keys
        P1 = (0, 2)
        P2 = (1, 2)
        P3 = (2, 3)

        # Create the DiscreteFactorGraph
        graph = DiscreteFactorGraph()

        # Add two unary factors (priors)
        graph.add(P1, [0.9, 0.3])
        graph.add(P2, "0.9 0.6")

        # Add a binary factor
        graph.add([P1, P2], "4 1 10 4")

        # Instantiate Values
        assignment = DiscreteValues()
        assignment[0] = 1
        assignment[1] = 1

        # Check if graph evaluation works ( 0.3*0.6*4 )
        self.assertAlmostEqual(0.72, graph(assignment))

        # Create a new test with third node and adding unary and ternary factor
        graph.add(P3, "0.9 0.2 0.5")
        keys = DiscreteKeys()
        keys.push_back(P1)
        keys.push_back(P2)
        keys.push_back(P3)
        graph.add(keys, "1 2 3 4 5 6 7 8 9 10 11 12")

        # Below assignment selects the 8th index in the ternary factor table
        assignment[0] = 1
        assignment[1] = 0
        assignment[2] = 1

        # Check if graph evaluation works (0.3*0.9*1*0.2*8)
        self.assertAlmostEqual(4.32, graph(assignment))

        # Below assignment selects the 3rd index in the ternary factor table
        assignment[0] = 0
        assignment[1] = 1
        assignment[2] = 0

        # Check if graph evaluation works (0.9*0.6*1*0.9*4)
        self.assertAlmostEqual(1.944, graph(assignment))

        # Check if graph product works
        product = graph.product()
        self.assertAlmostEqual(1.944, product(assignment))

    def test_optimize(self):
        """Test constructing and optizing a discrete factor graph."""

        # Three keys
        C = (0, 2)
        B = (1, 2)
        A = (2, 2)

        # A simple factor graph (A)-fAC-(C)-fBC-(B)
        # with smoothness priors
        graph = DiscreteFactorGraph()
        graph.add([A, C], "3 1 1 3")
        graph.add([C, B], "3 1 1 3")

        # Test optimization
        expectedValues = DiscreteValues()
        expectedValues[0] = 0
        expectedValues[1] = 0
        expectedValues[2] = 0
        actualValues = graph.optimize()
        self.assertEqual(list(actualValues.items()), list(expectedValues.items()))

    def test_MPE(self):
        """Test maximum probable explanation (MPE): same as optimize."""

        # Declare a bunch of keys
        C, A, B = (0, 2), (1, 2), (2, 2)

        # Create Factor graph
        graph = DiscreteFactorGraph()
        graph.add([C, A], "0.2 0.8 0.3 0.7")
        graph.add([C, B], "0.1 0.9 0.4 0.6")

        # We know MPE
        mpe = DiscreteValues()
        mpe[0] = 0
        mpe[1] = 1
        mpe[2] = 1

        # Use maxProduct
        dag = graph.maxProduct(OrderingType.COLAMD)
        actualMPE = dag.argmax()
        self.assertEqual(list(actualMPE.items()), list(mpe.items()))

        # All in one
        actualMPE2 = graph.optimize()
        self.assertEqual(list(actualMPE2.items()), list(mpe.items()))

    def test_sumProduct(self):
        """Test sumProduct."""

        # Declare a bunch of keys
        C, A, B = (0, 2), (1, 2), (2, 2)

        # Create Factor graph
        graph = DiscreteFactorGraph()
        graph.add([C, A], "0.2 0.8 0.3 0.7")
        graph.add([C, B], "0.1 0.9 0.4 0.6")

        # We know MPE
        mpe = DiscreteValues()
        mpe[0] = 0
        mpe[1] = 1
        mpe[2] = 1

        # Use default sumProduct
        bayesNet = graph.sumProduct()
        mpeProbability = bayesNet(mpe)
        self.assertAlmostEqual(mpeProbability, 0.36)  # regression

        # Use sumProduct
        for ordering_type in [
            OrderingType.COLAMD,
            OrderingType.METIS,
            OrderingType.NATURAL,
            OrderingType.CUSTOM,
        ]:
            bayesNet = graph.sumProduct(ordering_type)
            self.assertEqual(bayesNet(mpe), mpeProbability)


class TestChains(GtsamTestCase):
    def test_MPE_chain(self):
        """
        Test for numerical underflow in EliminateMPE on long chains.
        Adapted from the toy problem of @pcl15423
        Ref: https://github.com/borglab/gtsam/issues/1448
        """
        num_states = 3
        num_obs = 200
        desired_state = 1
        states = list(range(num_states))

        X = {index: make_key("X", index, len(states)) for index in range(num_obs)}
        Z = {index: make_key("Z", index, num_obs + 1) for index in range(num_obs)}
        graph = DiscreteFactorGraph()

        transition_cpt = generate_transition_cpt(num_states)
        for i in reversed(range(1, num_obs)):
            transition_conditional = DiscreteConditional(
                X[i], [X[i - 1]], transition_cpt
            )
            graph.push_back(transition_conditional)

        # Contrived example such that the desired state gives measurements [0, num_obs) with equal probability
        #   but all other states always give measurement num_obs
        obs_cpt = generate_observation_cpt(num_states, num_obs, desired_state)
        # Contrived example where each measurement is its own index
        for i in range(num_obs):
            obs_conditional = DiscreteConditional(Z[i], [X[i]], obs_cpt)
            factor = obs_conditional.likelihood(i)
            graph.push_back(factor)

        mpe = graph.optimize()
        vals = [mpe[X[i][0]] for i in range(num_obs)]

        self.assertEqual(vals, [desired_state] * num_obs)

    def test_sumProduct_chain(self):
        """
        Test for numerical underflow in EliminateDiscrete on long chains.
        Adapted from the toy problem of @pcl15423
        Ref: https://github.com/borglab/gtsam/issues/1448
        """
        num_states = 3
        chain_length = 400
        states = list(range(num_states))

        X = {index: make_key("X", index, len(states)) for index in range(chain_length)}
        graph = DiscreteFactorGraph()

        # Construct test transition matrix
        transitions = np.diag([1.0, 0.5, 0.1])
        transitions += 0.1/(num_states)

        # Ensure that the transition matrix is Markov (columns sum to 1)
        transitions /= np.sum(transitions, axis=0)

        # The stationary distribution is the eigenvector corresponding to eigenvalue 1
        eigvals, eigvecs = np.linalg.eig(transitions)
        stationary_idx = np.where(np.isclose(eigvals, 1.0))
        stationary_dist = eigvecs[:, stationary_idx]

        # Ensure that the stationary distribution is positive and normalized
        stationary_dist /= np.sum(stationary_dist)
        expected = DecisionTreeFactor(X[chain_length - 1], stationary_dist.ravel())

        # The transition matrix parsed by DiscreteConditional is a row-wise CPT
        transition_cpt = generate_transition_cpt(num_states, transitions.T)

        for i in reversed(range(1, chain_length)):
            transition_conditional = DiscreteConditional(
                X[i], [X[i - 1]], transition_cpt
            )
            graph.push_back(transition_conditional)

        # Run sum product using natural ordering so the resulting Bayes net has the form:
        # X_0 <- X_1 <- ... <- X_n
        sum_product = graph.sumProduct(OrderingType.NATURAL)

        # Get the DiscreteConditional representing the marginal on the last factor
        last_marginal = sum_product.at(chain_length - 1)

        # Ensure marginal probabilities are close to the stationary distribution
        self.gtsamAssertEquals(expected, last_marginal)


if __name__ == "__main__":
    unittest.main()
