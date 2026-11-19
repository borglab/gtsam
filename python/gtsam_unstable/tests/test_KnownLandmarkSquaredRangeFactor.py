"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for unstable squared-range factors and their QCQP bindings.
"""

import unittest

import gtsam
import gtsam_unstable
import numpy as np


class TestKnownLandmarkSquaredRangeFactor(unittest.TestCase):
    """Exercise unstable points and factors with stable graph bindings."""

    def test_module_ownership(self):
        """Only the unstable extension exposes all four factor specializations."""
        for dimension in (2, 3):
            for state in ("", "AugmentedPoint"):
                name = f"KnownLandmarkSquaredRangeFactor{state}{dimension}"
                self.assertFalse(hasattr(gtsam, name))
                self.assertTrue(hasattr(gtsam_unstable, name))
            self.assertFalse(hasattr(gtsam.Values, f"atAugmentedPoint{dimension}"))
            self.assertFalse(hasattr(gtsam, f"AugmentedPoint{dimension}"))
            self.assertTrue(hasattr(gtsam_unstable, f"AugmentedPoint{dimension}"))

    def test_invalid_landmark_dimensions(self):
        """Both state types reject undersized and oversized landmarks."""
        model = gtsam.noiseModel.Unit.Create(1)
        for dimension in (2, 3):
            for state in ("", "AugmentedPoint"):
                factor_type = getattr(
                    gtsam_unstable, f"KnownLandmarkSquaredRangeFactor{state}{dimension}"
                )
                for size in (dimension - 1, dimension + 1):
                    with self.subTest(dimension=dimension, state=state, size=size):
                        with self.assertRaisesRegex(
                            ValueError, "invalid landmark dimension"
                        ):
                            factor_type(0, np.zeros(size), 5.0, model)

    def test_augmented_factor_errors(self):
        """Augmented and ordinary states give matching residuals and graph costs."""
        model = gtsam.noiseModel.Unit.Create(1)
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                coordinates = np.arange(1.0, dimension + 1)
                point = getattr(gtsam_unstable, f"AugmentedPoint{dimension}")(coordinates)
                factor_type = getattr(
                    gtsam_unstable, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}"
                )
                factor = factor_type(0, np.zeros(dimension), 1.0, model)
                ordinary_type = getattr(
                    gtsam_unstable, f"KnownLandmarkSquaredRangeFactor{dimension}"
                )
                ordinary = ordinary_type(0, np.zeros(dimension), 1.0, model)
                residual = coordinates @ coordinates - 1.0
                np.testing.assert_allclose(factor.evaluateError(point), [residual])
                np.testing.assert_allclose(
                    ordinary.evaluateError(coordinates), [residual]
                )
                exact = factor_type(
                    0, np.zeros(dimension), np.linalg.norm(coordinates), model
                )
                np.testing.assert_allclose(
                    exact.evaluateError(point), [0.0], atol=1e-12
                )
                values = gtsam.Values()
                getattr(gtsam_unstable, f"insertAugmentedPoint{dimension}")(0, point, values)
                graph = gtsam.NonlinearFactorGraph()
                graph.add(factor)
                self.assertAlmostEqual(graph.error(values), float(0.5 * residual**2))

    def test_qcqp_factor_conversion(self):
        """QCQP construction requires an augmented state and a single column."""
        model = gtsam.noiseModel.Unit.Create(1)
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                coordinates = np.arange(1.0, dimension + 1)
                point = getattr(gtsam_unstable, f"AugmentedPoint{dimension}")(coordinates)
                graph = gtsam.NonlinearFactorGraph()
                factor_type = getattr(
                    gtsam_unstable, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}"
                )
                graph.add(factor_type(0, np.zeros(dimension), 1.0, model))
                values = gtsam.Values()
                getattr(gtsam_unstable, f"insertAugmentedPoint{dimension}")(0, point, values)
                lifted = gtsam.Values()
                getattr(gtsam_unstable, f"insertQcqpValue{dimension}")(0, point, lifted)
                problem = gtsam.QcqpProblem(graph)
                cost, equalities, inequalities = problem.evaluate(lifted)
                self.assertAlmostEqual(cost, graph.error(values))
                self.assertAlmostEqual(equalities, 0.0)
                self.assertAlmostEqual(inequalities, 0.0)
                with self.assertRaisesRegex(ValueError, "only supports D=1"):
                    gtsam.QcqpProblem(graph, 2)
                invalid_graph = gtsam.NonlinearFactorGraph()
                invalid_graph.add(factor_type(
                    0, np.zeros(dimension), 1.0,
                    gtsam.noiseModel.Isotropic.Sigma(2, 1.0)
                ))
                with self.assertRaisesRegex(ValueError, "one-dimensional noise"):
                    gtsam.QcqpProblem(invalid_graph)
                ordinary_graph = gtsam.NonlinearFactorGraph()
                ordinary_type = getattr(
                    gtsam_unstable, f"KnownLandmarkSquaredRangeFactor{dimension}"
                )
                ordinary_graph.add(ordinary_type(0, np.zeros(dimension), 1.0, model))
                with self.assertRaisesRegex(RuntimeError, "AugmentedPoint"):
                    gtsam.QcqpProblem(ordinary_graph)


if __name__ == "__main__":
    unittest.main()
