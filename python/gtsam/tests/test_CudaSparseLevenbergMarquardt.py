"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

General CUDA Levenberg-Marquardt Python wrapper tests.
"""

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X

cuda = getattr(gtsam, "cuda", None)


def _make_pose2_problem():
    """Create the small Pose2 problem used by the C++ CUDA LM tests."""
    graph = gtsam.NonlinearFactorGraph()
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.15, 0.15, 0.1]))
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.2, 0.2, 0.15]))
    graph.add(
        gtsam.PriorFactorPose2(X(0), gtsam.Pose2(), prior_noise))
    graph.add(
        gtsam.BetweenFactorPose2(X(0), X(1),
                                 gtsam.Pose2(2.0, 0.2, 0.1),
                                 odometry_noise))
    graph.add(
        gtsam.BetweenFactorPose2(X(1), X(2),
                                 gtsam.Pose2(1.5, -0.1, -0.08),
                                 odometry_noise))

    initial = gtsam.Values()
    initial.insert(X(0), gtsam.Pose2(0.35, -0.25, 0.18))
    initial.insert(X(1), gtsam.Pose2(2.45, -0.35, -0.05))
    initial.insert(X(2), gtsam.Pose2(3.15, 0.45, 0.2))
    return graph, initial


@unittest.skipIf(cuda is None, "GTSAM was not built with CUDA")
class TestCudaSparseLevenbergMarquardt(unittest.TestCase):

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

    def test_options_and_status_are_wrapped(self):
        linear = cuda.LinearSolverOptions()
        linear.backend = cuda.LinearSolverType.Pcg
        pcg = cuda.PcgOptions()
        pcg.maxIterations = 80
        pcg.relativeTolerance = 1e-9
        pcg.warmStart = False
        pcg.convergenceCheckInterval = 4

        params = cuda.SparseLevenbergMarquardtParams()
        params.linear = linear
        params.pcg = pcg
        params.fallbackOnUnsupported = False
        params.collectTiming = True
        params.collectAttemptTrace = True
        params.validateStructureEveryIteration = True

        self.assertEqual(cuda.LinearSolverType.Pcg, params.linear.backend)
        self.assertEqual(80, params.pcg.maxIterations)
        self.assertAlmostEqual(1e-9, params.pcg.relativeTolerance)
        self.assertFalse(params.pcg.warmStart)
        self.assertEqual(4, params.pcg.convergenceCheckInterval)
        self.assertFalse(params.fallbackOnUnsupported)
        self.assertTrue(params.collectTiming)
        self.assertTrue(params.collectAttemptTrace)
        self.assertTrue(params.validateStructureEveryIteration)

        status = cuda.DirectJacobianStatus()
        status.failure = cuda.DirectJacobianFailure.StructuralMismatch
        status.factorIndex = 3
        status.detail = "row layout changed"
        self.assertFalse(status.ok())

        result = cuda.SparseLevenbergMarquardtResult()
        result.backend = cuda.SparseLevenbergMarquardtBackend.CpuFallback
        result.fallbackReason = (
            cuda.SparseLevenbergMarquardtFallbackReason.PlanIncompatible)
        result.fallbackStatus = status
        result.fallbackDetail = status.detail
        result.termination = (
            cuda.SparseLevenbergMarquardtTerminationReason.MaxIterations)
        result.iterations = 2
        result.initialError = 10.0
        result.finalError = 1.0

        self.assertEqual(
            cuda.SparseLevenbergMarquardtBackend.CpuFallback,
            result.backend)
        self.assertEqual(3, result.fallbackStatus.factorIndex)
        self.assertEqual(2, result.iterations)
        self.assertLess(result.finalError, result.initialError)

    def test_zero_iterations_runs_without_cuda_setup(self):
        graph, initial = _make_pose2_problem()
        params = cuda.SparseLevenbergMarquardtParams()
        params.setMaxIterations(0)
        params.setErrorTol(0.0)
        params.fallbackOnUnsupported = False

        optimizer = cuda.SparseLevenbergMarquardtOptimizer(
            graph, initial, params)
        actual = optimizer.optimize()
        result = optimizer.result()

        self.assertTrue(initial.equals(actual, 1e-12))
        self.assertEqual(
            cuda.SparseLevenbergMarquardtTerminationReason.MaxIterations,
            result.termination)
        self.assertEqual(0, result.outerLinearizations)
        self.assertEqual(0, result.lambdaAttempts)
        self.assertAlmostEqual(result.initialError, result.finalError)
        self.assertAlmostEqual(result.finalError, optimizer.error())

    def test_general_cuda_pcg_improves_pose2_error(self):
        graph, initial = _make_pose2_problem()
        params = cuda.SparseLevenbergMarquardtParams()
        params.setMaxIterations(2)
        params.fallbackOnUnsupported = False
        linear = cuda.LinearSolverOptions()
        linear.backend = cuda.LinearSolverType.Pcg
        params.linear = linear
        pcg = cuda.PcgOptions()
        pcg.maxIterations = 100
        pcg.relativeTolerance = 1e-10
        params.pcg = pcg

        optimizer = self._run_or_skip_unavailable_runtime(
            lambda: cuda.SparseLevenbergMarquardtOptimizer(
                graph, initial, params))
        actual = self._run_or_skip_unavailable_runtime(optimizer.optimize)

        self.assertLess(graph.error(actual), graph.error(initial))
        self.assertEqual(
            cuda.SparseLevenbergMarquardtBackend.Device,
            optimizer.result().backend)

    def test_sfm_uses_shared_linear_solver_enum(self):
        params = cuda.SfmLevenbergMarquardtParams()
        params.setLinearSolver(cuda.LinearSolverType.Pcg)
        self.assertEqual(cuda.LinearSolverType.Pcg, params.getLinearSolver())


if __name__ == "__main__":
    unittest.main()
