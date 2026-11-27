"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Batched block-tridiagonal GPU Levenberg-Marquardt and differentiable-
backward Python wrapper tests.
"""

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X

cuda = getattr(gtsam, "cuda", None)

_NUM_BLOCKS = 3
_BLOCK_DIM = 3  # Pose2's tangent dimension.


def _make_pose2_graph(odometry_y_offset):
    """3-pose Pose2 chain: a prior on pose 0 and BetweenFactors coupling
    only adjacent poses, matching the block-tridiagonal chain shape."""
    graph = gtsam.NonlinearFactorGraph()
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.15, 0.15, 0.1]))
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.2, 0.2, 0.15]))
    graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(), prior_noise))
    graph.add(
        gtsam.BetweenFactorPose2(X(0), X(1),
                                 gtsam.Pose2(2.0, odometry_y_offset, 0.1),
                                 odometry_noise))
    graph.add(
        gtsam.BetweenFactorPose2(X(1), X(2),
                                 gtsam.Pose2(1.5, -0.1, -0.08),
                                 odometry_noise))
    return graph


def _make_pose2_initial(pose_x_offset):
    initial = gtsam.Values()
    initial.insert(X(0), gtsam.Pose2(0.35 + pose_x_offset, -0.25, 0.18))
    initial.insert(X(1), gtsam.Pose2(2.45 + pose_x_offset, -0.35, -0.05))
    initial.insert(X(2), gtsam.Pose2(3.15 + pose_x_offset, 0.45, 0.2))
    return initial


def _make_batch():
    graphs = [_make_pose2_graph(0.2), _make_pose2_graph(-0.3)]
    initial = [_make_pose2_initial(0.0), _make_pose2_initial(0.15)]
    return graphs, initial


def _make_cotangent(scale):
    cotangent = gtsam.VectorValues()
    cotangent.insert(X(0), scale * np.array([1.0, -0.5, 0.2]))
    cotangent.insert(X(1), scale * np.array([-0.3, 0.8, -0.1]))
    cotangent.insert(X(2), scale * np.array([0.5, 0.1, 0.4]))
    return cotangent


@unittest.skipIf(cuda is None, "GTSAM was not built with CUDA")
class TestCudaBatchedBlockTridiagonalLevenbergMarquardt(unittest.TestCase):

    def _run_or_skip_unavailable_runtime(self, function):
        try:
            return function()
        except RuntimeError as exception:
            message = str(exception)
            unavailable_fragments = (
                "no CUDA-capable device",
                "CUDA driver version is insufficient",
                "initialization error",
                "not initialized",
                "CUDA toolkit",
            )
            if any(fragment in message for fragment in unavailable_fragments):
                self.skipTest(f"CUDA runtime is unavailable: {message}")
            raise

    def test_batched_optimize_matches_cpu_per_graph(self):
        graphs, initial = _make_batch()
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(20)

        optimizer = self._run_or_skip_unavailable_runtime(
            lambda: cuda.BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
                graphs, initial, _NUM_BLOCKS, _BLOCK_DIM, params))
        actual = self._run_or_skip_unavailable_runtime(optimizer.optimize)
        results = optimizer.results()

        for i, graph in enumerate(graphs):
            cpu_optimizer = gtsam.LevenbergMarquardtOptimizer(
                graph, initial[i], params)
            expected = cpu_optimizer.optimize()
            self.assertAlmostEqual(
                graph.error(expected), graph.error(actual[i]), places=6)
            self.assertTrue(actual[i].equals(expected, 1e-6))
            self.assertEqual(cpu_optimizer.iterations(),
                             results[i].acceptedIterations)

    def test_batched_adjoint_matches_cpu_gradients(self):
        graphs, initial = _make_batch()
        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(20)
        optimizer = cuda.BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
            graphs, initial, _NUM_BLOCKS, _BLOCK_DIM, params)
        converged = self._run_or_skip_unavailable_runtime(optimizer.optimize)

        cotangents = [_make_cotangent(1.0), _make_cotangent(-1.0)]
        gpu_gradients = cuda.BatchedBlockTridiagonalDeviceGradients(
            graphs, converged, _NUM_BLOCKS, _BLOCK_DIM)
        actual_adjoint = self._run_or_skip_unavailable_runtime(
            lambda: gpu_gradients.adjoint(cotangents))

        keys = [X(0), X(1), X(2)]
        for i, graph in enumerate(graphs):
            cpu_gradients = gtsam.Gradients(graph, converged[i])
            expected = cpu_gradients.adjoint(cotangents[i])
            diff = np.linalg.norm(
                actual_adjoint[i].vector(keys) - expected.vector(keys))
            self.assertLess(diff, 1e-6)

    def test_batched_optimizer_rejects_mismatched_batch_sizes(self):
        # The C++ constructor throws std::invalid_argument, which pybind11
        # translates to ValueError (confirmed empirically, not assumed).
        graphs, initial = _make_batch()
        with self.assertRaises(ValueError):
            cuda.BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
                graphs, [initial[0]], _NUM_BLOCKS, _BLOCK_DIM)

    def test_batched_optimizer_rejects_invalid_chain_shape(self):
        graphs, initial = _make_batch()
        with self.assertRaises(ValueError):
            cuda.BatchedBlockTridiagonalLevenbergMarquardtOptimizer(
                graphs, initial, 1, _BLOCK_DIM)


if __name__ == "__main__":
    unittest.main()
