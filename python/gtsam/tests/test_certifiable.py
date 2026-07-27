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


def staircase_params(initial_rank):
    """Return deterministic parameters for the small wrapper test problems."""
    alm_params = gtsam.AugmentedLagrangianParams()
    alm_params.maxIterations = 100
    alm_params.initialMuEq = 10.0
    alm_params.muEqIncreaseRate = 2.0
    alm_params.absoluteViolationTolerance = 1e-8
    alm_params.relativeViolationTolerance = 1e-8
    alm_params.absoluteCostTolerance = 1e-10
    alm_params.relativeCostTolerance = 1e-10

    params = gtsam.RiemannianStaircaseParams()
    params.pMin = initial_rank
    params.pMax = initial_rank + 2
    params.alpha = 1e-2
    params.eta = 1e-3
    params.setAlmParams(alm_params)
    return params


class TestCertifiableWrappers(unittest.TestCase):
    """Exercise the high-level certifiable wrapper surface."""

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
        self.assertEqual(len(ranks), len(result.getNlpTimePerLevel()))
        self.assertEqual(
            len(ranks), len(result.getVerifyTimePerLevel())
        )
        rounded = result.roundedValues()
        self.assertEqual(rounded.size(), len(ground_truth))
        for key in rounded.keys():
            self.assertEqual(rounded.atMatrix(key).shape, (3, 3))


if __name__ == "__main__":
    unittest.main()
