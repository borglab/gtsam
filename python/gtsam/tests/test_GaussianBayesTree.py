"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for wrapped GaussianBayesTree methods added for covariance recovery.
Author: Codex, prompted by Frank Dellaert
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


def create_linear_chain():
    """Create a small anchored linear chain and its key tuple."""
    graph = gtsam.GaussianFactorGraph()

    x0 = X(0)
    x1 = X(1)
    x2 = X(2)
    x3 = X(3)

    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.ones(1))
    between_noise = gtsam.noiseModel.Diagonal.Sigmas(np.ones(1))

    graph.add(x0, np.eye(1), np.zeros(1), prior_noise)
    graph.add(x1, np.eye(1), x0, -np.eye(1), np.ones(1), between_noise)
    graph.add(x2, np.eye(1), x1, -np.eye(1), 2 * np.ones(1), between_noise)
    graph.add(x3, np.eye(1), x2, -np.eye(1), 3 * np.ones(1), between_noise)

    return graph, (x0, x1, x2, x3)


def subset_vector_values(values, keys):
    """Extract a VectorValues subset in the requested key order."""
    result = gtsam.VectorValues()
    for key in keys:
        result.insert(key, values.at(key))
    return result


class TestGaussianBayesTree(GtsamTestCase):
    """Tests for GaussianBayesTree wrapper extensions."""

    def test_marginal_information_and_covariance(self):
        """Single-key GaussianBayesTree marginals should match Marginals."""
        graph, keys = create_linear_chain()
        bayes_tree = graph.eliminateMultifrontal()
        solution = graph.optimize()
        marginals = gtsam.Marginals(graph, solution)

        expected_info = marginals.marginalInformation(keys[2])
        expected_covariance = marginals.marginalCovariance(keys[2])

        np.testing.assert_allclose(
            bayes_tree.marginalInformation(keys[2]), expected_info, atol=1e-9
        )
        np.testing.assert_allclose(
            bayes_tree.marginalCovariance(keys[2]), expected_covariance, atol=1e-9
        )

    def test_joint_key_vector(self):
        """joint(KeyVector) should preserve the queried joint covariance."""
        graph, keys = create_linear_chain()
        bayes_tree = graph.eliminateMultifrontal()
        solution = graph.optimize()
        query_keys = [keys[0], keys[2], keys[3]]

        expected = (
            gtsam.Marginals(graph, solution)
            .jointMarginalCovariance(query_keys)
            .fullMatrix()
        )

        joint_graph = bayes_tree.joint(query_keys)
        query_solution = subset_vector_values(solution, query_keys)
        actual = (
            gtsam.Marginals(joint_graph, query_solution)
            .jointMarginalCovariance(query_keys)
            .fullMatrix()
        )

        np.testing.assert_allclose(actual, expected, atol=1e-9)

    def test_joint_bayes_net_key_vector(self):
        """jointBayesNet(KeyVector) should optimize to the queried solution."""
        graph, keys = create_linear_chain()
        bayes_tree = graph.eliminateMultifrontal()
        solution = graph.optimize()
        query_keys = [keys[0], keys[2], keys[3]]

        joint_bayes_net = bayes_tree.jointBayesNet(query_keys)
        actual = joint_bayes_net.optimize()

        for key in query_keys:
            np.testing.assert_allclose(actual.at(key), solution.at(key), atol=1e-9)

    def test_joint_marginal_information_and_covariance(self):
        """Joint GaussianBayesTree marginals should match Marginals."""
        graph, keys = create_linear_chain()
        bayes_tree = graph.eliminateMultifrontal()
        solution = graph.optimize()
        query_keys = [keys[3], keys[0], keys[2]]

        marginals = gtsam.Marginals(graph, solution)
        expected_covariance = marginals.jointMarginalCovariance(query_keys).fullMatrix()
        expected_information = (
            marginals.jointMarginalInformation(query_keys).fullMatrix()
        )

        actual_covariance = bayes_tree.jointMarginalCovariance(query_keys).fullMatrix()
        actual_information = (
            bayes_tree.jointMarginalInformation(query_keys).fullMatrix()
        )

        np.testing.assert_allclose(actual_covariance, expected_covariance, atol=1e-9)
        np.testing.assert_allclose(actual_information, expected_information, atol=1e-9)

    def test_delete_cached_shortcuts(self):
        """deleteCachedShortcuts should clear caches created by query shortcuts."""
        graph, keys = create_linear_chain()
        bayes_tree = graph.eliminateMultifrontal()
        query_keys = [keys[0], keys[2], keys[3]]

        self.assertEqual(bayes_tree.numCachedSeparatorMarginals(), 0)
        bayes_tree.jointBayesNet(query_keys)
        self.assertGreater(bayes_tree.numCachedSeparatorMarginals(), 0)

        bayes_tree.deleteCachedShortcuts()
        self.assertEqual(bayes_tree.numCachedSeparatorMarginals(), 0)


if __name__ == "__main__":
    unittest.main()
