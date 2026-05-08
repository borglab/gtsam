"""Tests for TorchFactor — PyTorch autograd bridge for GTSAM."""

import unittest

import numpy as np

try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

import gtsam
from gtsam.utils.numerical_derivative import numericalDerivative11


@unittest.skipUnless(HAS_TORCH, "PyTorch not installed")
class TestTorchFactor(unittest.TestCase):
    """Test TorchFactor produces correct errors and Jacobians."""

    def setUp(self):
        from gtsam.utils.torch_factor import TorchFactor, extract_vector
        self.TorchFactor = TorchFactor
        self.extract_vector = extract_vector

    def test_simple_prior(self):
        """Test a simple prior factor: error = x - measurement."""
        measurement = np.array([1.0, 2.0, 3.0])

        def residual_fn(tensors):
            x = tensors[0]
            return x - torch.tensor(measurement, dtype=torch.float64)

        noise = gtsam.noiseModel.Unit.Create(3)
        key = 0
        tf = self.TorchFactor(
            noise_model=noise,
            keys=[key],
            residual_fn=residual_fn,
            value_extractors=[self.extract_vector],
        )

        factor = tf.as_custom_factor()
        graph = gtsam.NonlinearFactorGraph()
        graph.add(factor)

        # Test with initial values away from measurement
        values = gtsam.Values()
        values.insert(key, np.array([0.0, 0.0, 0.0]))

        # Error should be non-zero
        self.assertGreater(graph.error(values), 0)

        # Optimize should converge to measurement
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("SILENT")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
        result = optimizer.optimize()

        np.testing.assert_allclose(
            result.atVector(key), measurement, atol=1e-6
        )

    def test_between_factor(self):
        """Test a between factor: error = (x1 - x0) - measurement."""
        measurement = np.array([1.0, 0.5])

        def residual_fn(tensors):
            x0, x1 = tensors
            return (x1 - x0) - torch.tensor(measurement, dtype=torch.float64)

        noise = gtsam.noiseModel.Unit.Create(2)
        key0, key1 = 0, 1
        tf = self.TorchFactor(
            noise_model=noise,
            keys=[key0, key1],
            residual_fn=residual_fn,
            value_extractors=[self.extract_vector, self.extract_vector],
        )

        # Build graph with prior on key0 and between factor
        graph = gtsam.NonlinearFactorGraph()
        graph.addPrior(key0, np.array([0.0, 0.0]), noise)
        graph.add(tf.as_custom_factor())

        values = gtsam.Values()
        values.insert(key0, np.array([0.0, 0.0]))
        values.insert(key1, np.array([0.0, 0.0]))

        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("SILENT")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
        result = optimizer.optimize()

        np.testing.assert_allclose(result.atVector(key0), [0.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(result.atVector(key1), measurement, atol=1e-5)

    def test_jacobian_correctness(self):
        """Verify autograd Jacobians match numerical derivatives."""
        measurement = np.array([2.0, 3.0])

        def residual_fn(tensors):
            x = tensors[0]
            return x ** 2 - torch.tensor(measurement, dtype=torch.float64)

        noise = gtsam.noiseModel.Unit.Create(2)
        key = 0
        tf = self.TorchFactor(
            noise_model=noise,
            keys=[key],
            residual_fn=residual_fn,
            value_extractors=[self.extract_vector],
        )

        factor = tf.as_custom_factor()

        values = gtsam.Values()
        x0 = np.array([1.5, 2.0])
        values.insert(key, x0)

        # Get Jacobian from autograd via linearize
        gf = factor.linearize(values)
        # The linearized factor contains the whitened Jacobian
        # For unit noise model, whitened == unwhitened
        jf = gtsam.JacobianFactor(gf)
        A = jf.getA(jf.find(key))

        # Expected Jacobian: d(x^2)/dx = diag(2x)
        expected_J = np.diag(2 * x0)
        np.testing.assert_allclose(A, expected_J, atol=1e-5)

    def test_nonlinear_residual(self):
        """Test with a nonlinear residual (sin/cos)."""
        target = np.array([0.5, 0.866])  # sin(pi/6), cos(pi/6) approx

        def residual_fn(tensors):
            theta = tensors[0]
            predicted = torch.stack([torch.sin(theta[0]), torch.cos(theta[0])])
            return predicted - torch.tensor(target, dtype=torch.float64)

        noise = gtsam.noiseModel.Unit.Create(2)
        key = 0
        tf = self.TorchFactor(
            noise_model=noise,
            keys=[key],
            residual_fn=residual_fn,
            value_extractors=[self.extract_vector],
        )

        graph = gtsam.NonlinearFactorGraph()
        graph.add(tf.as_custom_factor())

        values = gtsam.Values()
        values.insert(key, np.array([0.3]))  # Initial guess

        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("SILENT")
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, values, params)
        result = optimizer.optimize()

        # Should converge to pi/6 ~ 0.5236
        np.testing.assert_allclose(
            result.atVector(key), [np.pi / 6], atol=1e-4
        )


if __name__ == "__main__":
    unittest.main()
