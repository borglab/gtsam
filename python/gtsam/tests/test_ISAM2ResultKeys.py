"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for the per-update key sets recorded on ISAM2Result.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import gtsam
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


def odometry_step(previous, current, initial):
    """One prior-free odometry increment between two poses."""
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()

    noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.2)
    step = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0))

    graph.add(gtsam.BetweenFactorPose3(previous, current, step, noise))
    values.insert(current, initial)
    return graph, values


class TestISAM2ResultKeys(GtsamTestCase):
    """The key sets describing what an update touched are readable."""

    @staticmethod
    def seeded_isam():
        """An ISAM2 with a single anchored pose already in it."""
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.PriorFactorPose3(
            X(0), gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
        values.insert(X(0), gtsam.Pose3())

        isam = gtsam.ISAM2()
        isam.update(graph, values)
        return isam

    def test_observed_keys_lists_touched_variables(self):
        isam = self.seeded_isam()
        graph, values = odometry_step(
            X(0), X(1), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)))
        result = isam.update(graph, values)

        observed = list(result.getObservedKeys())
        self.assertIn(X(0), observed)
        self.assertIn(X(1), observed)

    def test_factors_recalculated_is_reported(self):
        isam = self.seeded_isam()
        graph, values = odometry_step(
            X(0), X(1), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)))
        result = isam.update(graph, values)

        self.assertGreaterEqual(result.getFactorsRecalculated(), 0)

    def test_unused_and_removed_key_sets_are_empty_here(self):
        """Nothing is removed in a plain incremental update."""
        isam = self.seeded_isam()
        graph, values = odometry_step(
            X(0), X(1), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)))
        result = isam.update(graph, values)

        self.assertEqual(len(result.getUnusedKeys()), 0)
        self.assertEqual(len(result.getKeysWithRemovedFactors()), 0)

    def test_marked_keys_is_readable(self):
        isam = self.seeded_isam()
        graph, values = odometry_step(
            X(0), X(1), gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0)))
        result = isam.update(graph, values)

        # markedKeys drives which part of the tree is recalculated; with a
        # fresh increment it must at minimum cover the newly observed keys.
        marked = set(result.getMarkedKeys())
        self.assertTrue(marked.issuperset({X(0), X(1)}))


if __name__ == "__main__":
    unittest.main()
