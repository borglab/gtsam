"""Python wrapper tests for the public CPU SFM optimizer configuration."""

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import K, L, X


class TestSfmLevenbergMarquardt(unittest.TestCase):

    def test_cpu_defaults_select_fast_path(self):
        params = gtsam.SfmLevenbergMarquardtParams.ceresDefaults()
        solver = (
            gtsam.NonlinearOptimizerParams.LinearSolverType
            .MULTIFRONTAL_SOLVER
        )
        self.assertEqual(gtsam.SfmEliminationMode.Full,
                         params.getEliminationMode())
        self.assertEqual(solver, params.getLinearSolver())

    def test_typed_configuration_round_trip(self):
        params = gtsam.SfmLevenbergMarquardtParams.ceresDefaults()
        params.setEliminationMode(gtsam.SfmEliminationMode.Schur)
        solver = (
            gtsam.NonlinearOptimizerParams.LinearSolverType
            .MULTIFRONTAL_SOLVER
        )
        params.setLinearSolver(solver)

        self.assertEqual(gtsam.SfmEliminationMode.Schur,
                         params.getEliminationMode())
        self.assertEqual(solver, params.getLinearSolver())
        self.assertEqual("MULTIFRONTAL_SOLVER",
                         params.getLinearSolverType())

    def test_string_solver_api_remains_compatible(self):
        params = gtsam.SfmLevenbergMarquardtParams()
        params.setLinearSolverType("SEQUENTIAL_QR")
        expected = (
            gtsam.NonlinearOptimizerParams.LinearSolverType.SEQUENTIAL_QR
        )
        self.assertEqual(expected, params.getLinearSolver())

    def test_full_and_schur_optimize_shared_calibration(self):
        poses = [
            gtsam.Pose3(),
            gtsam.Pose3(gtsam.Rot3.Ypr(0.02, -0.01, 0.01),
                        gtsam.Point3(1.0, 0.0, 0.0)),
        ]
        points = [
            gtsam.Point3(-1.0, -0.5, 5.0),
            gtsam.Point3(0.8, -0.6, 4.5),
            gtsam.Point3(-0.7, 0.9, 5.5),
            gtsam.Point3(1.1, 0.8, 6.0),
        ]
        calibration = gtsam.Cal3_S2(500.0, 505.0, 0.0, 320.0, 240.0)
        measurement_noise = gtsam.noiseModel.Isotropic.Sigma(2, 1.0)

        graph = gtsam.NonlinearFactorGraph()
        for i, pose in enumerate(poses):
            camera = gtsam.PinholeCameraCal3_S2(pose, calibration)
            for j, point in enumerate(points):
                graph.add(gtsam.GeneralSFMFactor2Cal3_S2(
                    camera.project(point), measurement_noise, X(i), L(j),
                    K(0)))
        graph.addPriorPose3(
            X(0), poses[0], gtsam.noiseModel.Isotropic.Sigma(6, 1e-3))
        graph.addPriorPoint3(
            L(0), points[0], gtsam.noiseModel.Isotropic.Sigma(3, 1e-3))
        graph.addPriorCal3_S2(
            K(0), calibration,
            gtsam.noiseModel.Diagonal.Sigmas(
                np.array([50.0, 50.0, 0.1, 10.0, 10.0])))

        initial = gtsam.Values()
        for i, pose in enumerate(poses):
            initial.insert(X(i), pose.retract(
                np.array([0.01, -0.01, 0.01, 0.02, 0.0, -0.01])))
        for j, point in enumerate(points):
            initial.insertPoint3(
                L(j), point + np.array([0.01, -0.02, 0.01]))
        initial.insert(
            K(0), gtsam.Cal3_S2(510.0, 495.0, 0.0, 318.0, 242.0))

        reduced = (
            gtsam.SfmLevenbergMarquardtOptimizer.CreateReducedOrdering(
                graph, initial))
        schur_ordering = (
            gtsam.SfmLevenbergMarquardtOptimizer.CreateSchurOrdering(
                graph, reduced))
        self.assertEqual(len(poses) + 1, reduced.size())
        self.assertEqual(initial.size(), schur_ordering.size())

        solver = (
            gtsam.NonlinearOptimizerParams.LinearSolverType
            .MULTIFRONTAL_SOLVER
        )
        full_params = gtsam.SfmLevenbergMarquardtParams.ceresDefaults()
        full_params.setLinearSolver(solver)
        full_params.setOrdering(schur_ordering)
        schur_params = gtsam.SfmLevenbergMarquardtParams.ceresDefaults()
        schur_params.setEliminationMode(gtsam.SfmEliminationMode.Schur)
        schur_params.setLinearSolver(solver)
        schur_params.setOrdering(reduced)

        initial_error = graph.error(initial)
        full_result = gtsam.SfmLevenbergMarquardtOptimizer(
            graph, initial, full_params).optimize()
        schur_result = gtsam.SfmLevenbergMarquardtOptimizer(
            graph, initial, schur_params).optimize()
        full_error = graph.error(full_result)
        schur_error = graph.error(schur_result)

        self.assertLess(full_error, initial_error)
        self.assertLess(schur_error, initial_error)
        self.assertAlmostEqual(full_error, schur_error, places=7)
        np.testing.assert_allclose(
            full_result.atCal3_S2(K(0)).vector(),
            schur_result.atCal3_S2(K(0)).vector(), rtol=1e-6, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
