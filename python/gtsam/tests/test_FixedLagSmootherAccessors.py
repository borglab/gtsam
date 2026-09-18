"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for the state accessors on the fixed-lag smoothers.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam

X1 = 1
X2 = 2


def seeded_smoother(smoother):
    """Drive a smoother through two Pose2 states one second apart."""
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.3, 0.3, 0.1]))
    odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.05, 0.05, 0.05]))

    new_factors = gtsam.NonlinearFactorGraph()
    new_values = gtsam.Values()
    new_timestamps = {}

    new_factors.push_back(
        gtsam.PriorFactorPose2(X1, gtsam.Pose2(0, 0, 0), prior_noise))
    new_values.insert(X1, gtsam.Pose2(0.01, 0.01, 0.01))
    new_timestamps[X1] = 0.0

    new_factors.push_back(
        gtsam.BetweenFactorPose2(X1, X2, gtsam.Pose2(1.0, 0.0, 0.0),
                                 odometry_noise))
    new_values.insert(X2, gtsam.Pose2(1.01, 0.01, 0.01))
    new_timestamps[X2] = 1.0

    smoother.update(new_factors, new_values, new_timestamps)
    return smoother


class TestBatchFixedLagSmootherAccessors(GtsamTestCase):
    """BatchFixedLagSmoother exposes its optimizer state."""

    def setUp(self):
        self.smoother = seeded_smoother(gtsam.BatchFixedLagSmoother(10.0))

    def test_linearization_point(self):
        theta = self.smoother.getLinearizationPoint()

        self.assertIsInstance(theta, gtsam.Values)
        self.assertEqual(theta.size(), 2)
        self.assertTrue(theta.exists(X1))
        self.assertTrue(theta.exists(X2))

    def test_ordering(self):
        ordering = self.smoother.getOrdering()

        self.assertIsInstance(ordering, gtsam.Ordering)
        self.assertEqual(ordering.size(), 2)

    def test_delta(self):
        delta = self.smoother.getDelta()

        self.assertIsInstance(delta, gtsam.VectorValues)
        self.assertTrue(delta.exists(X1))
        self.assertEqual(len(delta.at(X1)), 3)


class TestIncrementalFixedLagSmootherAccessors(GtsamTestCase):
    """IncrementalFixedLagSmoother forwards ISAM2's state accessors."""

    def setUp(self):
        self.smoother = seeded_smoother(
            gtsam.IncrementalFixedLagSmoother(10.0))

    def test_linearization_point(self):
        theta = self.smoother.getLinearizationPoint()

        self.assertIsInstance(theta, gtsam.Values)
        self.assertEqual(theta.size(), 2)

    def test_linearization_point_matches_isam2(self):
        """The accessor forwards to the contained ISAM2."""
        self.gtsamAssertEquals(
            self.smoother.getLinearizationPoint().atPose2(X1),
            self.smoother.getISAM2().getLinearizationPoint().atPose2(X1),
            1e-12)

    def test_delta(self):
        delta = self.smoother.getDelta()

        self.assertIsInstance(delta, gtsam.VectorValues)
        self.assertTrue(delta.exists(X1))
        self.assertEqual(len(delta.at(X1)), 3)

    def test_delta_matches_isam2(self):
        np.testing.assert_allclose(
            self.smoother.getDelta().at(X2),
            self.smoother.getISAM2().getDelta().at(X2), atol=1e-12)


class TestFixedLagSmootherResult(GtsamTestCase):
    """The update result reports its optimizer statistics.

    Only BatchFixedLagSmoother populates these; IncrementalFixedLagSmoother
    fills the Result with fixed values, so they are tested here against the
    batch smoother.
    """

    def test_intermediate_steps(self):
        smoother = gtsam.BatchFixedLagSmoother(10.0)

        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1]))
        new_factors = gtsam.NonlinearFactorGraph()
        new_factors.push_back(
            gtsam.PriorFactorPose2(X1, gtsam.Pose2(0, 0, 0), prior_noise))
        new_values = gtsam.Values()
        new_values.insert(X1, gtsam.Pose2(0.01, 0.01, 0.01))

        result = smoother.update(new_factors, new_values, {X1: 0.0})

        # The outer LM loop is a do/while, so it runs at least once, and each
        # outer iteration performs at least one damped solve.
        self.assertGreaterEqual(result.getIterations(), 1)
        self.assertGreaterEqual(result.getIntermediateSteps(),
                                result.getIterations())


if __name__ == "__main__":
    unittest.main()
