"""Tests for Bayes-tree Gauss-Newton differentiation through both wrappers' API.

GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
"""

import contextlib
import gc
import io
import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X


def affine_problem():
    """Return an explicit factor residual graph, including an empty factor slot."""
    first, second = X(1), X(2)
    prior_noise = gtsam.noiseModel.Gaussian.Covariance(
        np.array([[0.16, 0.03], [0.03, 0.09]])
    )
    between_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.5]))
    final_noise = gtsam.noiseModel.Isotropic.Sigma(2, 0.7)
    graph = gtsam.NonlinearFactorGraph()
    graph.add(gtsam.PriorFactorPoint2(first, np.zeros(2), prior_noise))
    graph.resize(2)
    graph.add(
        gtsam.BetweenFactorPoint2(first, second, np.array([1.0, 0.2]), between_noise)
    )
    graph.add(gtsam.PriorFactorPoint2(second, np.array([1.1, 0.4]), final_noise))
    solution = gtsam.Values()
    solution.insert(first, np.array([0.1, -0.2]))
    solution.insert(second, np.array([1.2, 0.5]))
    identity, zero = np.eye(2), np.zeros((2, 2))
    prior = prior_noise.Whiten(identity)
    between = between_noise.Whiten(identity)
    final = final_noise.Whiten(identity)
    jacobian = np.block([[prior, zero], [-between, between], [zero, final]])
    return graph, solution, (first, second), jacobian


def explicit_linearization(keys, jacobian):
    """Build ordinary JacobianFactors, preserving the null factor slot."""
    graph = gtsam.GaussianFactorGraph()
    graph.add(keys[0], jacobian[:2, :2], np.zeros(2))
    graph.push_back(None)
    graph.add(keys[0], jacobian[2:4, :2], keys[1], jacobian[2:4, 2:], np.zeros(2))
    graph.add(keys[1], jacobian[4:, 2:], np.zeros(2))
    return graph


class TestGradients(unittest.TestCase):
    """Exercise actual native calls, array conversions, and owned-input lifetimes."""

    def setUp(self):
        self.graph, self.solution, self.keys, self.jacobian = affine_problem()
        self.seed = gtsam.VectorValues()
        self.seed.insert(self.keys[1], np.array([1.5, -0.4]))
        self.dense_seed = np.array([0.0, 0.0, 1.5, -0.4])
        self.direction = [
            np.array([0.2, -0.1]),
            np.empty(0),
            np.array([0.4, 0.3]),
            np.array([-0.5, 0.6]),
        ]

    def check_queries(self, gradients):
        """Compare independent dense solves and the forward/reverse pairing."""
        jacobian = self.jacobian
        hessian = jacobian.T @ jacobian
        expected_adjoint = np.linalg.solve(hessian, self.dense_seed)
        expected_forward = -np.linalg.solve(
            hessian, jacobian.T @ np.concatenate(self.direction)
        )
        adjoint = gradients.adjoint(self.seed)
        forward = gradients.pushforward(self.direction)
        reverse = gradients.pullback(self.seed)
        np.testing.assert_allclose(
            adjoint.vector(list(self.keys)), expected_adjoint, atol=1e-12
        )
        np.testing.assert_allclose(
            forward.vector(list(self.keys)), expected_forward, atol=1e-12
        )
        self.assertIsInstance(reverse, list)
        self.assertEqual([block.size for block in reverse], [2, 0, 2, 2])
        np.testing.assert_allclose(
            np.concatenate(reverse), -jacobian @ expected_adjoint, atol=1e-12
        )
        self.assertAlmostEqual(
            self.dense_seed @ forward.vector(list(self.keys)),
            gtsam.dot(reverse, self.direction),
            places=12,
        )

    def test_all_constructor_overloads_and_factorizations(self):
        """Every constructor works with nontrivial ordering and 64-bit keys."""
        self.assertGreater(self.keys[0], 2**53)
        ordering = gtsam.Ordering()
        for key in reversed(self.keys):
            ordering.push_back(key)
        linearized = explicit_linearization(self.keys, self.jacobian)
        tree = linearized.eliminateMultifrontal(ordering)
        self.check_queries(gtsam.Gradients(self.graph, self.solution))
        self.check_queries(gtsam.Gradients(linearized))
        self.check_queries(gtsam.Gradients(linearized, tree))
        for factorization in (
            gtsam.Gradients.Factorization.CHOLESKY,
            gtsam.Gradients.Factorization.QR,
        ):
            with self.subTest(factorization=factorization):
                for gradients in (
                    gtsam.Gradients(self.graph, self.solution, factorization),
                    gtsam.Gradients(self.graph, self.solution, ordering, factorization),
                    gtsam.Gradients(linearized, factorization),
                    gtsam.Gradients(linearized, ordering, factorization),
                ):
                    self.check_queries(gradients)

    def test_owned_inputs_survive_sources_and_repeated_queries(self):
        """Imported conditionals remain alive after source containers are released."""
        linearized = explicit_linearization(self.keys, self.jacobian)
        tree = linearized.eliminateMultifrontal()
        gradients = gtsam.Gradients(linearized, tree)
        nonlinear = gtsam.Gradients(self.graph, self.solution)
        saved_adjoint = gradients.adjoint(self.seed)
        saved_factor_cotangents = gradients.pullback(self.seed)
        # Removing a factor does not mutate any shared numerical data.
        self.graph.remove(0)
        del self.graph, self.solution, linearized, tree
        gc.collect()
        for _ in range(3):
            self.check_queries(gradients)
            self.check_queries(nonlinear)
        np.testing.assert_allclose(
            saved_adjoint.vector(list(self.keys)),
            gradients.adjoint(self.seed).vector(list(self.keys)),
        )
        del gradients, nonlinear
        gc.collect()
        expected = -self.jacobian @ np.linalg.solve(
            self.jacobian.T @ self.jacobian, self.dense_seed
        )
        np.testing.assert_allclose(np.concatenate(saved_factor_cotangents), expected)

    def test_adjoint_matches_public_tree_passes(self):
        """The facade agrees with the public lower-then-upper tree solves."""
        linearized = explicit_linearization(self.keys, self.jacobian)
        tree = linearized.eliminateMultifrontal()
        gradients = gtsam.Gradients(linearized, tree)
        lower_solution = tree.backSubstituteTranspose(self.seed)
        tree_adjoint = tree.backSubstitute(lower_solution)
        np.testing.assert_allclose(
            tree_adjoint.vector(list(self.keys)),
            gradients.adjoint(self.seed).vector(list(self.keys)), atol=1e-12,
        )

    def test_print_and_representation(self):
        """The shared print method supports native Python/MATLAB display."""
        gradients = gtsam.Gradients(self.graph, self.solution)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            gradients.print("Wrapped gradients: ")
        self.assertIn("Wrapped gradients: ", output.getvalue())
        self.assertIn("6 factor residual", output.getvalue())
        self.assertIn("6 factor residual", repr(gradients))

    def test_zero_seeds_and_empty_graph(self):
        """Omitted state blocks are zero, including a completely empty seed."""
        gradients = gtsam.Gradients(self.graph, self.solution)
        zeros = gradients.pullback(gtsam.VectorValues())
        self.assertEqual([block.size for block in zeros], [2, 0, 2, 2])
        self.assertEqual(gtsam.dot(zeros, zeros), 0.0)
        np.testing.assert_array_equal(
            gradients.pushforward(zeros).vector(list(self.keys)), np.zeros(4)
        )
        empty = gtsam.Gradients(gtsam.GaussianFactorGraph())
        self.assertEqual(empty.adjoint(gtsam.VectorValues()).size(), 0)
        self.assertEqual(empty.pushforward([]).size(), 0)
        self.assertEqual(empty.pullback(gtsam.VectorValues()), [])

    def test_invalid_query_inputs(self):
        """Block count, dimensions, null slots, keys and nonfinite values are checked."""
        gradients = gtsam.Gradients(self.graph, self.solution)
        for factor_direction in (
            [],
            [np.zeros(1), np.empty(0), np.zeros(2), np.zeros(2)],
            [np.zeros(2), np.ones(1), np.zeros(2), np.zeros(2)],
            [np.full(2, np.nan), np.empty(0), np.zeros(2), np.zeros(2)],
            self.direction + [np.empty(0)],
        ):
            with self.assertRaises(ValueError):
                gradients.pushforward(factor_direction)
        for key, block in (
            (X(99), np.ones(2)),
            (self.keys[0], np.ones(1)),
            (self.keys[0], np.array([np.inf, 0.0])),
        ):
            seed = gtsam.VectorValues()
            seed.insert(key, block)
            with self.assertRaises(ValueError):
                gradients.adjoint(seed)

    def test_unsupported_noise_and_hessian_rows(self):
        """Robust, constrained, and synthetic Hessian factor residuals are rejected."""
        unit = gtsam.noiseModel.Unit.Create(2)
        robust = gtsam.noiseModel.Robust.Create(
            gtsam.noiseModel.mEstimator.Huber.Create(1.0), unit
        )
        for noise in (robust, gtsam.noiseModel.Constrained.All(2)):
            graph = gtsam.NonlinearFactorGraph()
            graph.add(gtsam.PriorFactorPoint2(self.keys[0], np.zeros(2), noise))
            with self.assertRaises(ValueError):
                gtsam.Gradients(graph, self.solution)
        hessian_graph = gtsam.GaussianFactorGraph()
        hessian_graph.add(gtsam.HessianFactor(self.keys[0], np.eye(2), np.zeros(2), 0.0))
        with self.assertRaises(ValueError):
            gtsam.Gradients(hessian_graph)


if __name__ == "__main__":
    unittest.main()
