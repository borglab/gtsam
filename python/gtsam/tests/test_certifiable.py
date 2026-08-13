"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Python wrapper tests for certifiable optimization.
"""
# pylint: disable=invalid-name, no-name-in-module

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X

try:
    from gtsam import (
        ChordalOrderingType,
        MosekChordalSDP,
        MosekMonolithicSDP,
    )

    HAS_MOSEK_SDP = True
except ImportError:
    ChordalOrderingType = None
    MosekChordalSDP = None
    MosekMonolithicSDP = None
    HAS_MOSEK_SDP = False


def staircase_params(initial_rank):
    """Return deterministic parameters for the small wrapper test problems."""
    alm_params = gtsam.AugmentedLagrangianParams()
    alm_params.maxIterations = 100
    alm_params.absoluteViolationTolerance = 1e-8

    params = gtsam.RiemannianStaircaseParams()
    params.pMin = initial_rank
    params.pMax = initial_rank + 2
    params.alpha = 1e-2
    params.eta = 1e-3
    params.setAlmParams(alm_params)
    return params


def rot2_ring_qcqp(num_rotations=5):
    """Build a gauge-fixed Rot2 ring and its exact ground-truth poses."""
    delta = 2.0 * np.pi / num_rotations
    ground_truth = [
        gtsam.Rot2.fromAngle(index * delta) for index in range(num_rotations)
    ]

    graph = gtsam.NonlinearFactorGraph()
    graph.add(
        gtsam.FrobeniusPriorRot2(
            X(0),
            gtsam.Rot2.Identity().matrix(),
            gtsam.noiseModel.Constrained.All(4),
        )
    )
    for index in range(num_rotations):
        next_index = (index + 1) % num_rotations
        graph.add(
            gtsam.FrobeniusBetweenFactorRot2(
                X(index),
                X(next_index),
                ground_truth[index].between(ground_truth[next_index]),
            )
        )

    return gtsam.QcqpProblem(graph), ground_truth


class TestCertifiableWrappers(unittest.TestCase):
    """Exercise the high-level certifiable wrapper surface."""

    def test_negative_rank_is_rejected(self):
        """Reject negative wrapper ranks before they reach Eigen allocation."""
        values = gtsam.Values()
        values.insert(X(0), np.eye(2))
        with self.assertRaises(ValueError):
            gtsam.RiemannianStaircaseOptimizer.padInitialValues(values, -1)

        params = gtsam.RiemannianStaircaseParams()
        with self.assertRaises(TypeError):
            params.pMin = -1

    def test_params_and_rot2_staircase(self):
        """Configure nested ALM parameters and certify a small Rot2 ring."""
        num_rotations = 5
        delta = 2.0 * np.pi / num_rotations
        ground_truth = [
            gtsam.Rot2.fromAngle(index * delta) for index in range(num_rotations)
        ]

        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()
        for index in range(num_rotations):
            next_index = (index + 1) % num_rotations
            measurement = ground_truth[index].between(ground_truth[next_index])
            graph.add(
                gtsam.FrobeniusBetweenFactorRot2(
                    X(index), X(next_index), measurement
                )
            )
            perturbation = gtsam.Rot2.fromAngle(0.01 * (index + 1))
            initial.insert(
                X(index),
                ground_truth[index].compose(perturbation).matrix().T,
            )

        params = staircase_params(initial_rank=2)
        self.assertEqual(params.pMin, 2)
        self.assertEqual(params.getAlmParams().maxIterations, 100)

        result = gtsam.RiemannianStaircaseOptimizer(
            graph, initial, params
        ).optimize()

        self.assertTrue(result.certified)
        self.assertTrue(result.hasRoundedSolution())
        self.assertLessEqual(result.finalRank, params.pMax)
        ranks = result.getRanksVisited()
        self.assertEqual(len(ranks), len(result.getCostPerLevel()))
        self.assertEqual(
            len(ranks), len(result.getMinEigenvaluePerLevel())
        )
        rounded = result.roundedValues()
        self.assertEqual(rounded.size(), num_rotations)
        for key in rounded.keys():
            self.assertEqual(rounded.atMatrix(key).shape, (2, 2))

    def test_rot3_staircase(self):
        """Certify a compact noncommuting Rot3 graph and inspect its result."""
        ground_truth = [
            gtsam.Rot3.RzRyRx(
                0.14 * index, -0.09 * index, 0.11 * index
            )
            for index in range(4)
        ]
        edges = [(0, 1), (1, 2), (2, 3), (3, 0), (0, 2), (1, 3)]

        graph = gtsam.NonlinearFactorGraph()
        for source, target in edges:
            graph.add(
                gtsam.FrobeniusBetweenFactorRot3(
                    X(source),
                    X(target),
                    ground_truth[source].between(ground_truth[target]),
                )
            )

        initial = gtsam.Values()
        for index, rotation in enumerate(ground_truth):
            perturbation = gtsam.Rot3.Expmap(
                np.array([0.006, -0.004, 0.005]) * (index + 1)
            )
            initial.insert(
                X(index), rotation.compose(perturbation).matrix().T
            )

        params = staircase_params(initial_rank=3)
        result = gtsam.RiemannianStaircaseOptimizer(
            graph, initial, params
        ).optimize()

        self.assertTrue(result.certified)
        self.assertTrue(result.hasRoundedSolution())
        self.assertGreaterEqual(result.totalTime, 0.0)
        ranks = result.getRanksVisited()
        self.assertEqual(
            len(ranks), len(result.getQcqpBuildTimePerLevel())
        )
        self.assertEqual(len(ranks), len(result.getNlpTimePerLevel()))
        self.assertEqual(
            len(ranks), len(result.getVerifyTimePerLevel())
        )
        rounded = result.roundedValues()
        self.assertEqual(rounded.size(), len(ground_truth))
        for key in rounded.keys():
            self.assertEqual(rounded.atMatrix(key).shape, (3, 3))


@unittest.skipUnless(HAS_MOSEK_SDP, "GTSAM was built without MOSEK support")
class TestMosekCertifiableWrappers(unittest.TestCase):
    """Exercise both optional MOSEK-backed SDP formulations."""

    def assert_solver_solution(self, solver, ground_truth):
        """Check the common solve, metadata, and Rot2 recovery surface."""
        with self.assertRaises(RuntimeError):
            solver.qcqpValues()
        with self.assertRaises(RuntimeError):
            solver.variableEVRs()

        self.assertTrue(solver.solve())
        self.assertTrue(solver.problemStatus())
        self.assertTrue(np.isfinite(solver.objectiveValue()))
        self.assertGreaterEqual(solver.solveTimeSeconds(), 0.0)

        expected_keys = [X(index) for index in range(len(ground_truth))]
        self.assertEqual(list(solver.orderedKeys()), expected_keys)
        ordered_key_dims = solver.orderedKeyDims()
        self.assertEqual(set(ordered_key_dims), set(expected_keys))
        self.assertTrue(all(dimension == 5 for dimension in ordered_key_dims.values()))

        variable_evrs = solver.variableEVRs()
        qcqp_values = solver.qcqpValues()
        repeated_qcqp_values = solver.qcqpValues()
        repeated_variable_evrs = solver.variableEVRs()
        self.assertEqual(qcqp_values.size(), len(ground_truth))
        self.assertEqual(len(variable_evrs), len(ground_truth))
        self.assertTrue(all(np.isfinite(evr) for evr in variable_evrs))
        np.testing.assert_allclose(variable_evrs, repeated_variable_evrs)
        for key in expected_keys:
            self.assertEqual(qcqp_values.atMatrix(key).shape, (5, 1))
            np.testing.assert_allclose(
                qcqp_values.atMatrix(key), repeated_qcqp_values.atMatrix(key)
            )

        recovered_poses = gtsam.extractQcqpValuesRot2(qcqp_values)
        pose_errors = [
            abs(
                ground_truth[index]
                .between(recovered_poses.atRot2(X(index)))
                .theta()
            )
            for index in range(len(ground_truth))
        ]
        self.assertEqual(recovered_poses.size(), len(ground_truth))
        self.assertEqual(len(pose_errors), len(ground_truth))
        self.assertLess(max(pose_errors), 1e-5)

    def test_monolithic_rot2_ring(self):
        """Solve and recover a small Rot2 ring with the monolithic SDP."""
        problem, ground_truth = rot2_ring_qcqp()
        solver = MosekMonolithicSDP(problem)
        self.assert_solver_solution(solver, ground_truth)

    def test_chordal_rot2_ring(self):
        """Solve and recover a small Rot2 ring with the chordal SDP."""
        problem, ground_truth = rot2_ring_qcqp()
        solver = MosekChordalSDP(problem, ChordalOrderingType.Colamd)
        self.assertGreater(solver.bayesTree().size(), 0)
        self.assert_solver_solution(solver, ground_truth)


if __name__ == "__main__":
    unittest.main()
