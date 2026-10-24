"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Tests for augmented points, squared-range factors, and their QCQP bindings.
"""

import unittest

import gtsam
import numpy as np


class TestKnownLandmarkSquaredRangeFactor(unittest.TestCase):
    """Exercise the shared 2D and 3D interfaces through actual Python calls."""

    def test_augmented_points(self):
        """Point methods marshal coordinates and validate input dimensions."""
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                point_type = getattr(gtsam, f"AugmentedPoint{dimension}")
                coordinates = np.arange(1.0, dimension + 1)
                point = point_type(coordinates)
                np.testing.assert_allclose(point.vector(), coordinates)
                np.testing.assert_allclose(point_type().vector(), np.zeros(dimension))
                np.testing.assert_allclose(
                    point_type.Identity().vector(), np.zeros(dimension)
                )
                self.assertEqual(point.dim(), dimension)
                self.assertAlmostEqual(point.norm(), np.linalg.norm(coordinates))
                self.assertAlmostEqual(point.dot(point), coordinates @ coordinates)
                self.assertTrue(point.equals(point_type(coordinates)))
                self.assertFalse(point.equals(point_type(coordinates + 1)))
                for size in (dimension - 1, dimension + 1):
                    with self.assertRaisesRegex(ValueError, "invalid point dimension"):
                        point_type(np.zeros(size))

    def test_invalid_landmark_dimensions(self):
        """Both state types reject undersized and oversized landmarks."""
        model = gtsam.noiseModel.Unit.Create(1)
        for dimension in (2, 3):
            for state in ("", "AugmentedPoint"):
                factor_type = getattr(
                    gtsam, f"KnownLandmarkSquaredRangeFactor{state}{dimension}"
                )
                for size in (dimension - 1, dimension + 1):
                    with self.subTest(dimension=dimension, state=state, size=size):
                        with self.assertRaisesRegex(
                            ValueError, "invalid landmark dimension"
                        ):
                            factor_type(0, np.zeros(size), 5.0, model)

    def test_augmented_values(self):
        """Typed Values access preserves coordinates, lifetimes, and 64-bit keys."""
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                point_type = getattr(gtsam, f"AugmentedPoint{dimension}")
                coordinates = np.arange(1.0, dimension + 1)
                key = (1 << 40) + dimension
                point = point_type(coordinates)
                values = gtsam.Values()
                values.insert(key, point)
                read = getattr(values, f"atAugmentedPoint{dimension}")
                values.update(key, point_type(coordinates + 1))
                np.testing.assert_allclose(read(key).vector(), coordinates + 1)
                values.insert_or_assign(key, point)
                values.insert_or_assign(key + 1, point)
                del point
                for _ in range(3):
                    np.testing.assert_allclose(read(key).vector(), coordinates)
                    np.testing.assert_allclose(read(key + 1).vector(), coordinates)

    def test_augmented_factor_errors(self):
        """Augmented and ordinary states give matching residuals and graph costs."""
        model = gtsam.noiseModel.Unit.Create(1)
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                coordinates = np.arange(1.0, dimension + 1)
                point = getattr(gtsam, f"AugmentedPoint{dimension}")(coordinates)
                factor_type = getattr(
                    gtsam, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}"
                )
                factor = factor_type(0, np.zeros(dimension), 1.0, model)
                ordinary_type = getattr(
                    gtsam, f"KnownLandmarkSquaredRangeFactor{dimension}"
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
                values.insert(0, point)
                graph = gtsam.NonlinearFactorGraph()
                graph.add(factor)
                self.assertAlmostEqual(graph.error(values), float(0.5 * residual**2))

    def test_qcqp_conversions(self):
        """Lift, recovery, insertion, and extraction work for both dimensions."""
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                coordinates = np.arange(1.0, dimension + 1)
                point = getattr(gtsam, f"AugmentedPoint{dimension}")(coordinates)
                key = (1 << 40) + dimension
                lift = getattr(gtsam, f"qcqpValue{dimension}")(point)
                expected = np.r_[
                    1.0, coordinates, coordinates @ coordinates
                ].reshape(-1, 1)
                np.testing.assert_allclose(lift, expected)
                recover = getattr(gtsam, f"fromQcqpValue{dimension}")
                self.assertTrue(recover(lift).equals(point))
                self.assertTrue(recover(-lift).equals(point))
                for invalid in (np.zeros((dimension + 1, 1)), np.zeros_like(lift)):
                    with self.assertRaises(ValueError):
                        recover(invalid)
                lifted = gtsam.Values()
                insert = getattr(gtsam, f"insertQcqpValue{dimension}")
                insert(key, point, lifted)
                insert(key, point, lifted)
                np.testing.assert_allclose(lifted.atMatrix(key), expected)
                recovered = getattr(gtsam, f"extractQcqpValues{dimension}")(lifted)
                read = getattr(recovered, f"atAugmentedPoint{dimension}")
                self.assertTrue(read(key).equals(point))

    def test_qcqp_factor_conversion(self):
        """QCQP construction requires an augmented state and a single column."""
        model = gtsam.noiseModel.Unit.Create(1)
        for dimension in (2, 3):
            with self.subTest(dimension=dimension):
                coordinates = np.arange(1.0, dimension + 1)
                point = getattr(gtsam, f"AugmentedPoint{dimension}")(coordinates)
                graph = gtsam.NonlinearFactorGraph()
                factor_type = getattr(
                    gtsam, f"KnownLandmarkSquaredRangeFactorAugmentedPoint{dimension}"
                )
                graph.add(factor_type(0, np.zeros(dimension), 1.0, model))
                values = gtsam.Values()
                values.insert(0, point)
                lifted = gtsam.Values()
                getattr(gtsam, f"insertQcqpValue{dimension}")(0, point, lifted)
                problem = gtsam.QcqpProblem(graph)
                cost, equalities, inequalities = problem.evaluate(lifted)
                self.assertAlmostEqual(cost, graph.error(values))
                self.assertAlmostEqual(equalities, 0.0)
                self.assertAlmostEqual(inequalities, 0.0)
                with self.assertRaisesRegex(ValueError, "only supports D=1"):
                    gtsam.QcqpProblem(graph, 2)
                ordinary_graph = gtsam.NonlinearFactorGraph()
                ordinary_type = getattr(
                    gtsam, f"KnownLandmarkSquaredRangeFactor{dimension}"
                )
                ordinary_graph.add(ordinary_type(0, np.zeros(dimension), 1.0, model))
                with self.assertRaisesRegex(RuntimeError, "AugmentedPoint"):
                    gtsam.QcqpProblem(ordinary_graph)


if __name__ == "__main__":
    unittest.main()
