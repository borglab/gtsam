"""Tests for GNC with general CUDA and specialized CUDA SfM inner solvers."""

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import C, P

cuda = getattr(gtsam, "cuda", None)


def _point_problem():
    graph = gtsam.NonlinearFactorGraph()
    noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)
    for _ in range(5):
        graph.add(gtsam.PriorFactorPoint2(0, np.zeros(2), noise))
    graph.add(gtsam.PriorFactorPoint2(0, np.array([20.0, 20.0]), noise))
    initial = gtsam.Values()
    initial.insert(0, np.array([0.1, -0.1]))
    return graph, initial


def _sfm_problem():
    """Individual Bundler projections with one conflicting observation."""
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    calibration = gtsam.Cal3Bundler(150.0, 0.0, 0.0, 0.0, 0.0)
    cameras = [
        gtsam.PinholeCameraCal3Bundler(
            gtsam.Pose3(gtsam.Rot3(), np.array([x, y, 0.0])), calibration)
        for x, y in [(0.0, 0.0), (0.8, 0.1), (-0.3, 0.7), (0.5, -0.6)]
    ]
    noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)
    for i, camera in enumerate(cameras):
        initial.insert(C(i), camera)
    for j in range(16):
        point = np.array([(j % 4 - 1.5) * 0.5,
                          (j // 4 - 1.5) * 0.4, 4.0 + 0.2 * (j % 3)])
        offset = np.array([0.01, -0.02, 0.01])
        if j == 0:
            offset += [0.2, 0.1, 0.1]
        initial.insertPoint3(P(j), point + offset)
        for i, camera in enumerate(cameras):
            graph.add(gtsam.GeneralSFMFactorCal3Bundler(
                camera.project(point), noise, C(i), P(j)))
    bad_measurement = graph.at(0).measured() + [80.0, -60.0]
    graph.add(gtsam.GeneralSFMFactorCal3Bundler(
        bad_measurement, noise, C(0), P(0)))
    return graph, initial


@unittest.skipIf(cuda is None, "GTSAM was not built with CUDA")
class TestCudaGnc(unittest.TestCase):
    def _run_on_device(self, function):
        try:
            return function()
        except RuntimeError as exception:
            message = str(exception)
            if any(fragment in message for fragment in (
                    "no CUDA-capable device", "CUDA driver version is insufficient",
                    "initialization error", "not initialized")):
                self.skipTest(f"CUDA runtime is unavailable: {message}")
            raise

    def test_classes_are_wrapped(self):
        # A CUDA build must provide the bindings even when no device is present.
        for name in ("GncSparseLMParams", "GncSparseLMOptimizer",
                     "GncSfmLMParams", "GncSfmLMOptimizer"):
            with self.subTest(name=name):
                self.assertTrue(hasattr(cuda, name), f"Missing cuda.{name}")

    def test_sparse_params_preserve_backend_settings(self):
        inner = cuda.SparseLevenbergMarquardtParams()
        inner.setMaxIterations(7)
        inner.fallbackOnUnsupported = False
        inner.collectTiming = True
        linear = cuda.LinearSolverOptions()
        linear.backend = cuda.LinearSolverType.Pcg
        inner.linear = linear
        pcg = cuda.PcgOptions()
        pcg.maxIterations = 37
        inner.pcg = pcg
        params = cuda.GncSparseLMParams(inner)
        graph, initial = _point_problem()
        optimizer = cuda.GncSparseLMOptimizer(graph, initial, params)
        actual = optimizer.getParams().baseOptimizerParams
        self.assertIsInstance(actual, cuda.SparseLevenbergMarquardtParams)
        self.assertEqual(7, actual.getMaxIterations())
        self.assertFalse(actual.fallbackOnUnsupported)
        self.assertTrue(actual.collectTiming)
        self.assertEqual(cuda.LinearSolverType.Pcg, actual.linear.backend)
        self.assertEqual(37, actual.pcg.maxIterations)

    def test_sfm_params_preserve_backend_settings(self):
        inner = cuda.SfmLevenbergMarquardtParams()
        inner.setMaxIterations(9)
        inner.setLinearSolver(cuda.LinearSolverType.Pcg)
        inner.setEliminationMode(gtsam.SfmEliminationMode.Schur)
        inner.enableDetailedProfiling = True
        params = cuda.GncSfmLMParams(inner)
        graph, initial = _sfm_problem()
        optimizer = cuda.GncSfmLMOptimizer(graph, initial, params)
        actual = optimizer.getParams().baseOptimizerParams
        self.assertIsInstance(actual, cuda.SfmLevenbergMarquardtParams)
        self.assertEqual(9, actual.getMaxIterations())
        self.assertEqual(cuda.LinearSolverType.Pcg, actual.getLinearSolver())
        self.assertEqual(gtsam.SfmEliminationMode.Schur,
                         actual.getEliminationMode())
        self.assertTrue(actual.enableDetailedProfiling)

    def test_gnc_controls_and_factor_weights(self):
        graph, initial = _point_problem()
        for params_type, optimizer_type in (
                (cuda.GncSparseLMParams, cuda.GncSparseLMOptimizer),
                (cuda.GncSfmLMParams, cuda.GncSfmLMOptimizer)):
            with self.subTest(params=params_type):
                params = params_type()
                params.setLossType(gtsam.GncLossType.GM)
                params.setScheduler(gtsam.GncScheduler.Linear)
                params.setLambdaStep(1.6)
                params.lambdaMax = 1e8
                params.setMaxIterations(12)
                params.setRelativeCostTol(1e-7)
                params.setWeightsTol(1e-5)
                params.setVerbosityGNC(params_type.Verbosity.SILENT)
                params.setKnownInliers([0])
                params.setKnownOutliers([5])
                optimizer = optimizer_type(graph, initial, params)
                actual = optimizer.getParams()
                self.assertEqual(gtsam.GncLossType.GM, actual.lossType)
                self.assertEqual(gtsam.GncScheduler.Linear, actual.scheduler)
                self.assertAlmostEqual(1.6, actual.lambdaStep)
                self.assertEqual(1e8, actual.lambdaMax)
                self.assertEqual(12, actual.maxIterations)
                self.assertAlmostEqual(1e-7, actual.relativeCostTol)
                self.assertAlmostEqual(1e-5, actual.weightsTol)
                np.testing.assert_array_equal([1, 1, 1, 1, 1, 0],
                                              optimizer.getWeights())
                optimizer.setInlierCostThresholds(3.0)
                np.testing.assert_allclose(optimizer.getInlierCostThresholds(), 3.0)
                optimizer.setInlierCostThresholdsAtProbability(0.99)
                np.testing.assert_allclose(optimizer.getInlierCostThresholds(),
                                           -np.log(0.01))
                optimizer.setWeights(np.array([1, .5, .5, .5, .5, 0]))
                self.assertEqual(.5, optimizer.getWeights()[1])
                with self.assertRaisesRegex(RuntimeError, "number of specified weights"):
                    optimizer.setWeights(np.ones(2))

    def test_sparse_zero_iterations_without_device(self):
        graph, initial = _point_problem()
        inner = cuda.SparseLevenbergMarquardtParams()
        inner.setMaxIterations(0)
        inner.fallbackOnUnsupported = False
        params = cuda.GncSparseLMParams(inner)
        params.setMaxIterations(0)
        optimizer = cuda.GncSparseLMOptimizer(graph, initial, params)
        self.assertTrue(initial.equals(optimizer.optimize(), 1e-12))

    def test_sfm_rejects_unsupported_graph_without_device(self):
        # This error proves dispatch to CUDA SfM rather than ordinary CPU LM.
        graph, initial = _point_problem()
        optimizer = cuda.GncSfmLMOptimizer(graph, initial, cuda.GncSfmLMParams())
        with self.assertRaisesRegex(ValueError, "only supports GeneralSFMFactor"):
            optimizer.optimize()

    def test_sparse_rejects_outlier_on_device(self):
        graph, initial = _point_problem()
        for loss in (gtsam.GncLossType.TLS, gtsam.GncLossType.GM):
            with self.subTest(loss=loss):
                inner = cuda.SparseLevenbergMarquardtParams()
                inner.fallbackOnUnsupported = False
                linear = cuda.LinearSolverOptions()
                linear.backend = cuda.LinearSolverType.Pcg
                inner.linear = linear
                params = cuda.GncSparseLMParams(inner)
                params.setLossType(loss)
                params.setKnownInliers(list(range(5)))
                optimizer = cuda.GncSparseLMOptimizer(graph, initial, params)
                result = self._run_on_device(optimizer.optimize)
                reference_params = gtsam.GncLMParams()
                reference_params.setLossType(loss)
                reference_params.setKnownInliers(list(range(5)))
                reference = gtsam.GncLMOptimizer(graph, initial, reference_params)
                expected = reference.optimize()
                np.testing.assert_allclose(result.atPoint2(0), expected.atPoint2(0),
                                           atol=1e-4)
                np.testing.assert_allclose(optimizer.getWeights(),
                                           reference.getWeights(), atol=1e-4)
                self.assertLess(optimizer.getWeights()[-1], .01)

    def test_sfm_updates_weights_on_device(self):
        graph, initial = _sfm_problem()
        # Leave one good observation unknown and start it far enough from its
        # measurement to exercise the outer GNC loop, not only the initial LM.
        inner = cuda.SfmLevenbergMarquardtParams()
        inner.setMaxIterations(30)
        inner.setlambdaInitial(1e-3)
        params = cuda.GncSfmLMParams(inner)
        params.setKnownInliers(list(range(1, graph.size() - 1)))
        params.setKnownOutliers([graph.size() - 1])
        optimizer = cuda.GncSfmLMOptimizer(graph, initial, params)
        weights = optimizer.getWeights().copy()
        weights[0] = .25
        optimizer.setWeights(weights)
        result = self._run_on_device(optimizer.optimize)
        self.assertLess(optimizer.getWeights()[-1], .01)
        np.testing.assert_allclose(optimizer.getWeights()[:-1], 1.0)
        inlier_error = sum(graph.at(i).error(result)
                           for i in range(graph.size() - 1))
        self.assertLess(inlier_error, 1e-3)


if __name__ == "__main__":
    unittest.main()
