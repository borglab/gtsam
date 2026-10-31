"""ParameterTag identity and parameter VJPs through real Python callbacks.

GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
"""

import copy
import gc
import unittest
import weakref

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X


KEY = X(42)
CALIBRATION = gtsam.ParameterTag("parameter")
SCALE = gtsam.ParameterTag("parameter")  # Same label, different identity.
STATE_COTANGENT = np.array([0.6, -0.7])
STATE_JACOBIAN = np.array([[1.3, -0.2], [0.4, 0.9]])
PARAMETER_JACOBIAN = np.array([[0.2, 0.7], [-0.5, 1.1]])
DIRECTIONS = (np.array([0.7, -0.2]), np.array([-0.3, 0.8]))
COVARIANCE = np.array([[0.7, 0.2], [0.2, 1.4]])


def vector_factor(callback=None):
    """A custom factor with an identity state Jacobian and optional VJP."""
    def error(this, values, jacobians):
        if jacobians is not None:
            jacobians[0] = np.eye(2)
        return values.atPoint2(KEY)

    return gtsam.CustomFactor(
        gtsam.noiseModel.Unit.Create(2), [KEY], error, callback
    )


def problem(calibration, scale):
    """Two parameterized factors share tags; ordinary factors also affect the solve."""
    graph = gtsam.NonlinearFactorGraph()
    noises = (
        gtsam.noiseModel.Diagonal.Sigmas(np.array([0.4, 1.3])),
        gtsam.noiseModel.Gaussian.Covariance(COVARIANCE),
    )

    def factor(state_jacobian, parameter_jacobian, direction, noise):
        def error(this, values, jacobians):
            if jacobians is not None:
                jacobians[0] = state_jacobian
            return (
                state_jacobian @ values.atPoint2(KEY)
                - parameter_jacobian @ calibration - scale * direction
            )

        def vjp(this, values, parameter, factor_residual_cotangent):
            if parameter == CALIBRATION:
                return -parameter_jacobian.T @ factor_residual_cotangent
            if parameter == SCALE:
                return np.array([-direction @ factor_residual_cotangent])
            return None

        return gtsam.CustomFactor(noise, [KEY], error, vjp)

    graph.add(factor(np.eye(2), np.eye(2), DIRECTIONS[0], noises[0]))
    graph.resize(2)  # A null slot does not require parameter bookkeeping.
    graph.add(factor(STATE_JACOBIAN, PARAMETER_JACOBIAN, DIRECTIONS[1], noises[1]))
    graph.add(gtsam.PriorFactorPoint2(
        KEY, np.array([0.1, -0.4]), gtsam.noiseModel.Unit.Create(2)
    ))
    graph.add(vector_factor())  # A custom factor without a parameter callback.
    initial = gtsam.Values()
    initial.insert(KEY, np.zeros(2))
    solution = gtsam.GaussNewtonOptimizer(graph, initial).optimize()
    assert graph.linearize(solution).gradientAtZero().norm() < 1e-10
    return graph, solution


class TestParameterGradients(unittest.TestCase):
    """Check identity, numerical derivatives, failure behavior, and ownership."""

    def test_tag_identity_copy_and_hash(self):
        tag = gtsam.ParameterTag("calibration")
        self.assertEqual(tag.name(), "calibration")
        self.assertIn("calibration", repr(tag))
        self.assertNotEqual(tag, gtsam.ParameterTag("calibration"))
        self.assertNotEqual(gtsam.ParameterTag(), gtsam.ParameterTag())
        self.assertNotEqual(tag, None)
        copies = [gtsam.ParameterTag(tag), copy.copy(tag), copy.deepcopy(tag)]
        mapping = {tag: "shared parameter"}
        for other in copies:
            self.assertEqual(tag, other)
            self.assertTrue(tag.equals(other))
            self.assertEqual(hash(tag), hash(other))
            self.assertEqual(mapping[other], "shared parameter")
        del tag, mapping
        gc.collect()
        self.assertEqual(copies[0], copies[1])
        self.assertEqual(copies[0].name(), "calibration")

    def test_shared_parameters_and_fixed_noise(self):
        calibration, scale = np.array([0.3, -0.2]), 0.8
        graph, solution = problem(calibration, scale)
        cotangents = gtsam.VectorValues()
        cotangents.insert(KEY, STATE_COTANGENT)
        self.assertGreater(KEY, 2**53)
        first_information = np.diag(1 / np.array([0.4, 1.3])**2)
        second_information = np.linalg.inv(COVARIANCE)
        hessian = (first_information + 2 * np.eye(2)
                   + STATE_JACOBIAN.T @ second_information @ STATE_JACOBIAN)
        adjoint = np.linalg.solve(hessian, STATE_COTANGENT)
        expected_calibration = (
            first_information
            + PARAMETER_JACOBIAN.T @ second_information @ STATE_JACOBIAN
        ) @ adjoint
        expected_scale = (
            DIRECTIONS[0] @ first_information @ adjoint
            + DIRECTIONS[1] @ second_information @ STATE_JACOBIAN @ adjoint
        )
        ordering = gtsam.Ordering()
        ordering.push_back(KEY)
        for method in (gtsam.Gradients.Factorization.CHOLESKY,
                       gtsam.Gradients.Factorization.QR):
            for gradients in (gtsam.Gradients(graph, solution, method),
                              gtsam.Gradients(graph, solution, ordering, method)):
                for _ in range(3):
                    np.testing.assert_allclose(
                        gradients.parameterGradient(cotangents, CALIBRATION),
                        expected_calibration, atol=1e-12,
                    )
                    np.testing.assert_allclose(
                        gradients.parameterGradient(cotangents, SCALE),
                        [expected_scale], atol=1e-12,
                    )
                self.assertIsNone(gradients.parameterGradient(
                    cotangents, gtsam.ParameterTag("parameter")
                ))
                np.testing.assert_array_equal(gradients.parameterGradient(
                    gtsam.VectorValues(), CALIBRATION
                ), np.zeros(2))
                # C++ tag copies preserve parameter identity.
                np.testing.assert_allclose(gradients.parameterGradient(
                    cotangents, gtsam.ParameterTag(CALIBRATION)
                ), expected_calibration, atol=1e-12)

        def loss(parameter, factor_scale):
            _, estimate = problem(parameter, factor_scale)
            return STATE_COTANGENT @ estimate.atPoint2(KEY)

        for step in (1e-4, 1e-5, 1e-6):
            for coordinate in range(2):
                direction = np.eye(2)[coordinate] * step
                finite = (loss(calibration + direction, scale)
                          - loss(calibration - direction, scale)) / (2 * step)
                self.assertAlmostEqual(expected_calibration[coordinate], finite, places=8)
            finite = (loss(calibration, scale + step)
                      - loss(calibration, scale - step)) / (2 * step)
            self.assertAlmostEqual(expected_scale, finite, places=8)

    def test_callback_and_tag_lifetimes(self):
        tag = gtsam.ParameterTag("temporary")
        retained_tags, observed_states = [], []
        error_calls = []

        def error(this, values, jacobians):
            error_calls.append(1)
            if jacobians is not None:
                jacobians[0] = np.eye(2)
            return values.atPoint2(KEY)

        def vjp(this, values, parameter, cotangent, recognized=gtsam.ParameterTag(tag)):
            retained_tags.append(parameter)
            observed_states.append(values.atPoint2(KEY).copy())
            return -cotangent if parameter == recognized else None

        callback_reference = weakref.ref(vjp)
        factor = gtsam.CustomFactor(gtsam.noiseModel.Unit.Create(2), [KEY], error, vjp)
        graph = gtsam.NonlinearFactorGraph()
        graph.add(factor)
        solution = gtsam.Values()
        solution.insert(KEY, np.zeros(2))
        gradients = gtsam.Gradients(graph, solution)
        calls_at_construction = len(error_calls)
        solution.update(KEY, np.array([100., 200.]))
        query_tag = gtsam.ParameterTag(tag)
        del tag, graph, factor, solution, error, vjp
        gc.collect()
        self.assertIsNotNone(callback_reference())
        cotangents = gtsam.VectorValues()
        cotangents.insert(KEY, STATE_COTANGENT)
        for _ in range(3):
            result = gradients.parameterGradient(cotangents, query_tag)
            np.testing.assert_allclose(result, STATE_COTANGENT, atol=1e-12)
        self.assertEqual(len(error_calls), calls_at_construction)
        for observed in observed_states:
            np.testing.assert_array_equal(observed, np.zeros(2))
        del gradients, query_tag
        gc.collect()
        self.assertIsNone(callback_reference())
        self.assertEqual(retained_tags[0], retained_tags[-1])
        self.assertEqual(retained_tags[0].name(), "temporary")
        np.testing.assert_allclose(result, STATE_COTANGENT, atol=1e-12)

    def test_absent_and_invalid_callbacks(self):
        values = gtsam.Values()
        values.insert(KEY, np.zeros(2))
        cotangents = gtsam.VectorValues()
        cotangents.insert(KEY, STATE_COTANGENT)
        for callback in (None, lambda *_: None):
            graph = gtsam.NonlinearFactorGraph()
            factor = vector_factor(callback)
            graph.add(factor)
            self.assertIsNone(factor.parameterVjp(values, SCALE, np.ones(2)))
            self.assertIsNone(gtsam.Gradients(graph, values).parameterGradient(cotangents, SCALE))
        empty = gtsam.Gradients(gtsam.NonlinearFactorGraph(), gtsam.Values())
        self.assertIsNone(empty.parameterGradient(gtsam.VectorValues(), SCALE))
        gaussian = gtsam.Gradients(gtsam.GaussianFactorGraph())
        with self.assertRaises(RuntimeError):
            gaussian.parameterGradient(gtsam.VectorValues(), SCALE)
        for invalid in (np.empty(0), np.array([np.nan]), np.array([np.inf])):
            graph = gtsam.NonlinearFactorGraph()
            graph.add(vector_factor(lambda *_, result=invalid: result))
            with self.assertRaises(ValueError):
                gtsam.Gradients(graph, values).parameterGradient(cotangents, SCALE)
        graph = gtsam.NonlinearFactorGraph()
        graph.add(vector_factor(lambda *_: np.zeros(1)))
        graph.add(vector_factor(lambda *_: np.zeros(2)))
        with self.assertRaises(ValueError):
            gtsam.Gradients(graph, values).parameterGradient(cotangents, SCALE)

        def failing(*_):
            raise ValueError("parameter callback failed")

        graph = gtsam.NonlinearFactorGraph()
        graph.add(vector_factor(failing))
        with self.assertRaisesRegex(ValueError, "parameter callback failed"):
            gtsam.Gradients(graph, values).parameterGradient(cotangents, SCALE)

    def test_diagonal_noise_model_tag(self):
        tag = gtsam.ParameterTag("visual sigma")
        sigmas = np.array([0.4, 1.3])
        noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas, tag)
        self.assertTrue(noise.hasParameterTag())
        self.assertEqual(noise.parameterTag(), tag)
        self.assertIsInstance(noise, gtsam.noiseModel.Diagonal)
        self.assertNotIsInstance(noise, gtsam.noiseModel.Isotropic)

        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.PriorFactorPoint2(KEY, np.zeros(2), noise))
        graph.add(gtsam.PriorFactorPoint2(
            KEY, np.array([1.0, -0.5]), gtsam.noiseModel.Isotropic.Sigma(2, 0.5)
        ))
        initial = gtsam.Values()
        initial.insert(KEY, np.zeros(2))
        solution = gtsam.GaussNewtonOptimizer(graph, initial).optimize()
        cotangents = gtsam.VectorValues()
        cotangents.insert(KEY, solution.atPoint2(KEY) - np.array([0.2, -0.3]))
        gradient = gtsam.Gradients(graph, solution).parameterGradient(cotangents, tag)
        self.assertIsNotNone(gradient)
        self.assertEqual(gradient.size, 2)
        self.assertTrue(np.all(np.isfinite(gradient)))
        self.assertIsNone(
            gtsam.Gradients(graph, solution).parameterGradient(
                cotangents, gtsam.ParameterTag("visual sigma")
            )
        )


if __name__ == "__main__":
    unittest.main()
