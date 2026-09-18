"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the relative translation factor used by certifiable SE(d)
synchronization.
"""
# pylint: disable=invalid-name, no-name-in-module

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import R, T
from gtsam.utils.test_case import GtsamTestCase


class TestRelativeTranslationFactor3(GtsamTestCase):
    """Tests for the SE(3) instantiation."""

    def setUp(self):
        self.measured = np.array([0.4, -0.2, 0.7])
        self.weight = 2.5
        self.factor = gtsam.RelativeTranslationFactor3(
            R(0), T(0), T(1), self.measured, self.weight
        )

    def test_accessors(self):
        """The measurement and precision survive the round trip to Python."""
        np.testing.assert_allclose(self.factor.measured(), self.measured)
        self.assertAlmostEqual(self.factor.weight(), self.weight)
        self.assertEqual(self.factor.keys(), [R(0), T(0), T(1)])

    def test_unwhitened_error(self):
        """The residual is sqrt(weight) * (t_j - t_i - R_i * measured)."""
        rotation = gtsam.Rot3.RzRyRx(0.2, -0.1, 0.3)
        ti = np.array([1.0, 2.0, 3.0])
        tj = np.array([1.5, 1.0, 3.25])

        values = gtsam.Values()
        values.insert(R(0), rotation)
        values.insert(T(0), gtsam.Point3(ti))
        values.insert(T(1), gtsam.Point3(tj))

        expected = np.sqrt(self.weight) * (
            tj - ti - rotation.matrix() @ self.measured
        )
        np.testing.assert_allclose(
            self.factor.unwhitenedError(values), expected, atol=1e-9
        )

    def test_zero_at_consistent_poses(self):
        """A measurement generated from two poses produces zero residual."""
        pose_i = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.1, 0.2, -0.3),
                             gtsam.Point3(np.array([0.5, -1.0, 2.0])))
        pose_j = gtsam.Pose3(gtsam.Rot3.RzRyRx(-0.2, 0.4, 0.1),
                             gtsam.Point3(np.array([1.5, 0.5, 1.0])))
        measured = pose_i.rotation().unrotate(
            pose_j.translation() - pose_i.translation()
        )

        factor = gtsam.RelativeTranslationFactor3(R(0), T(0), T(1), measured, 1.0)
        values = gtsam.Values()
        values.insert(R(0), pose_i.rotation())
        values.insert(T(0), gtsam.Point3(pose_i.translation()))
        values.insert(T(1), gtsam.Point3(pose_j.translation()))

        np.testing.assert_allclose(
            factor.unwhitenedError(values), np.zeros(3), atol=1e-12
        )

    def test_rejects_nonpositive_weight(self):
        """A nonpositive precision is rejected at construction."""
        with self.assertRaises(Exception):
            gtsam.RelativeTranslationFactor3(R(0), T(0), T(1), self.measured, 0.0)


class TestRelativeTranslationFactor2(GtsamTestCase):
    """Tests for the SE(2) instantiation."""

    def test_unwhitened_error(self):
        """The planar residual matches the same weighted expression."""
        measured = np.array([0.3, -0.6])
        weight = 4.0
        rotation = gtsam.Rot2(0.35)
        ti = np.array([-1.0, 0.5])
        tj = np.array([0.25, 0.75])

        factor = gtsam.RelativeTranslationFactor2(R(0), T(0), T(1), measured, weight)
        np.testing.assert_allclose(factor.measured(), measured)
        self.assertAlmostEqual(factor.weight(), weight)

        values = gtsam.Values()
        values.insert(R(0), rotation)
        values.insert(T(0), gtsam.Point2(ti))
        values.insert(T(1), gtsam.Point2(tj))

        expected = np.sqrt(weight) * (tj - ti - rotation.matrix() @ measured)
        np.testing.assert_allclose(
            factor.unwhitenedError(values), expected, atol=1e-9
        )


if __name__ == "__main__":
    unittest.main()
