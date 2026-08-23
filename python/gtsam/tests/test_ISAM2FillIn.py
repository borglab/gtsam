"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for the Bayes tree fill-in diagnostics on ISAM2Result and for
the adaptive reorder parameters on ISAM2Params.
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase


def pose_chain(length):
    """Build a simple odometry chain of the given length."""
    graph = gtsam.NonlinearFactorGraph()
    values = gtsam.Values()

    prior_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
    odometry_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.2)
    step = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0))

    graph.add(gtsam.PriorFactorPose3(X(0), gtsam.Pose3(), prior_noise))
    values.insert(X(0), gtsam.Pose3())

    pose = gtsam.Pose3()
    for i in range(1, length):
        graph.add(gtsam.BetweenFactorPose3(X(i - 1), X(i), step,
                                           odometry_noise))
        pose = pose.compose(step)
        values.insert(X(i), pose)

    return graph, values


class TestAdaptiveReorderParams(GtsamTestCase):
    """ISAM2Params exposes the adaptive reorder settings."""

    def test_defaults(self):
        params = gtsam.ISAM2Params()
        self.assertFalse(params.enableAdaptiveReorder)
        self.assertAlmostEqual(params.adaptiveReorderThreshold, 2.0)

    def test_settable(self):
        params = gtsam.ISAM2Params()
        params.enableAdaptiveReorder = True
        params.adaptiveReorderThreshold = 1.5

        self.assertTrue(params.enableAdaptiveReorder)
        self.assertAlmostEqual(params.adaptiveReorderThreshold, 1.5)

    def test_round_trip_through_isam(self):
        """The setting survives into the constructed ISAM2."""
        params = gtsam.ISAM2Params()
        params.enableAdaptiveReorder = True
        params.adaptiveReorderThreshold = 1.25

        isam = gtsam.ISAM2(params)
        self.assertTrue(isam.params().enableAdaptiveReorder)
        self.assertAlmostEqual(isam.params().adaptiveReorderThreshold, 1.25)


class TestTreeNnz(GtsamTestCase):
    """Bayes tree fill-in is observable from Python."""

    def test_result_reports_nnz(self):
        graph, values = pose_chain(5)
        isam = gtsam.ISAM2()
        result = isam.update(graph, values)

        self.assertGreater(result.getTreeNnz(), 0)

    def test_isam_accessor_matches_result(self):
        """ISAM2.treeNnz() agrees with the value recorded in the result."""
        graph, values = pose_chain(5)
        isam = gtsam.ISAM2()
        result = isam.update(graph, values)

        self.assertEqual(isam.treeNnz(), result.getTreeNnz())

    def test_nnz_grows_with_the_tree(self):
        """Fill-in increases as poses are added incrementally."""
        isam = gtsam.ISAM2()

        graph, values = pose_chain(3)
        isam.update(graph, values)
        small = isam.treeNnz()

        graph2 = gtsam.NonlinearFactorGraph()
        values2 = gtsam.Values()
        odometry_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.2)
        step = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1.0, 0.0, 0.0))
        pose = isam.calculateEstimatePose3(X(2))
        for i in range(3, 8):
            graph2.add(gtsam.BetweenFactorPose3(X(i - 1), X(i), step,
                                                odometry_noise))
            pose = pose.compose(step)
            values2.insert(X(i), pose)
        isam.update(graph2, values2)

        self.assertGreater(isam.treeNnz(), small)

    def test_batch_reorder_flag_defaults_false(self):
        """With adaptive reorder off, the flag is never raised."""
        graph, values = pose_chain(5)
        isam = gtsam.ISAM2()
        result = isam.update(graph, values)

        self.assertFalse(result.getBatchReorderTriggered())


if __name__ == "__main__":
    unittest.main()
