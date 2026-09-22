"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the quadratic range factor used by certifiable range-aided
SLAM.
"""
# pylint: disable=invalid-name, no-name-in-module

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import L, T, U
from gtsam.utils.test_case import GtsamTestCase


class TestQuadraticRangeFactor3(GtsamTestCase):
    """Tests for the SE(3) instantiation, whose auxiliary is a Unit3."""

    def setUp(self):
        self.range = 1.4
        self.weight = 2.25
        self.factor = gtsam.QuadraticRangeFactor3(
            T(0), L(0), U(0), self.range, self.weight
        )

    def test_accessors(self):
        """The measurement and precision survive construction."""
        self.assertAlmostEqual(self.range, self.factor.range())
        self.assertAlmostEqual(self.weight, self.factor.weight())

    def test_zero_error_at_consistent_values(self):
        """target - t - range * u vanishes when the three agree."""
        translation = np.array([0.3, -0.2, 0.1])
        direction = gtsam.Unit3(np.array([0.0, 0.0, 1.0]))
        target = translation + self.range * direction.unitVector()

        values = gtsam.Values()
        values.insert(T(0), translation)
        values.insert(L(0), target)
        values.insert(U(0), direction)
        self.assertAlmostEqual(0.0, self.factor.error(values))

    def test_error_matches_weighted_residual(self):
        """Away from consistency the error is 0.5 * weight * ||residual||^2."""
        translation = np.array([0.3, -0.2, 0.1])
        target = np.array([1.0, 0.5, 0.9])
        direction = gtsam.Unit3(np.array([0.2, -0.4, 0.9]))

        values = gtsam.Values()
        values.insert(T(0), translation)
        values.insert(L(0), target)
        values.insert(U(0), direction)

        residual = target - translation - self.range * direction.unitVector()
        expected = 0.5 * self.weight * float(residual @ residual)
        self.assertAlmostEqual(expected, self.factor.error(values))

    def test_minimum_over_sphere_is_the_range_residual(self):
        """The auxiliary makes the term equal the raw range residual."""
        translation = np.zeros(3)
        target = np.array([0.0, 0.0, 2.0])
        # The best direction points from the translation to the target.
        direction = gtsam.Unit3(target - translation)

        values = gtsam.Values()
        values.insert(T(0), translation)
        values.insert(L(0), target)
        values.insert(U(0), direction)

        distance = np.linalg.norm(target - translation)
        expected = 0.5 * self.weight * (distance - self.range) ** 2
        self.assertAlmostEqual(expected, self.factor.error(values))

    def test_rejects_invalid_measurements(self):
        """A non-positive precision or a negative range is refused."""
        with self.assertRaises(Exception):
            gtsam.QuadraticRangeFactor3(T(0), L(0), U(0), 1.0, 0.0)
        with self.assertRaises(Exception):
            gtsam.QuadraticRangeFactor3(T(0), L(0), U(0), -1.0, 1.0)


class TestQuadraticRangeFactor2(GtsamTestCase):
    """Tests for the SE(2) instantiation, whose auxiliary is a Rot2."""

    def setUp(self):
        self.range = 1.1
        self.weight = 2.0
        self.factor = gtsam.QuadraticRangeFactor2(
            T(0), L(0), U(0), self.range, self.weight
        )

    def test_direction_is_the_rotation_first_column(self):
        """A Rot2 auxiliary stands for the unit vector R * e_1."""
        translation = np.array([0.4, -0.2])
        rotation = gtsam.Rot2.fromAngle(0.35)
        direction = np.array([rotation.c(), rotation.s()])
        target = translation + self.range * direction

        values = gtsam.Values()
        values.insert(T(0), translation)
        values.insert(L(0), target)
        values.insert(U(0), rotation)
        self.assertAlmostEqual(0.0, self.factor.error(values))

    def test_error_matches_weighted_residual(self):
        """Away from consistency the error is 0.5 * weight * ||residual||^2."""
        translation = np.array([0.4, -0.2])
        target = np.array([1.9, 0.7])
        rotation = gtsam.Rot2.fromAngle(0.35)

        values = gtsam.Values()
        values.insert(T(0), translation)
        values.insert(L(0), target)
        values.insert(U(0), rotation)

        direction = np.array([rotation.c(), rotation.s()])
        residual = target - translation - self.range * direction
        expected = 0.5 * self.weight * float(residual @ residual)
        self.assertAlmostEqual(expected, self.factor.error(values))


if __name__ == "__main__":
    unittest.main()
