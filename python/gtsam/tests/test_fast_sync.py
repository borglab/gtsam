"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Tests for the generated FAST-Sync Python wrappers.
"""

# pylint: disable=no-member

import unittest

import numpy as np
import gtsam
from gtsam import Pose2, Pose3, Rot2, Rot3, SL4, Similarity2, Similarity3


class TestFastSync(unittest.TestCase):
    """Exercise every wrapped fastSync<T> instantiation."""

    @staticmethod
    def _triangle(values, between_factor, prior_factor, d):
        model = gtsam.noiseModel.Isotropic.Sigma(d, 0.1)
        graph = gtsam.NonlinearFactorGraph()
        keys = (10, 42, 77)
        graph.add(between_factor(keys[0], keys[1], values[0].between(values[1]), model))
        graph.add(between_factor(keys[1], keys[2], values[1].between(values[2]), model))
        graph.add(between_factor(keys[2], keys[0], values[2].between(values[0]), model))
        graph.add(prior_factor(keys[0], values[0], model))
        return graph, keys

    def _check(self, values, between_factor, prior_factor, d, solver,
               accessor, tolerance=1e-7):
        graph, keys = self._triangle(values, between_factor, prior_factor, d)
        result = solver(graph)
        self.assertIsInstance(result, gtsam.Values)
        for key, expected in zip(keys, values):
            estimated = accessor(result, key)
            self.assertTrue(expected.equals(estimated, tolerance))
            self.assertTrue(np.all(np.isfinite(estimated.matrix())))
            self.assertGreater(np.linalg.det(estimated.matrix()), 0.0)
            if hasattr(estimated, "scale"):
                self.assertGreater(estimated.scale(), 0.0)
        expected_relative = values[0].between(values[1])
        actual_relative = accessor(result, keys[0]).between(
            accessor(result, keys[1]))
        self.assertTrue(expected_relative.equals(actual_relative, tolerance))

    def test_rot2(self):
        """Test the fastSync<Rot2> instantiation."""
        values = [Rot2(0.2), Rot2(0.7), Rot2(-0.4)]
        self._check(values, gtsam.BetweenFactorRot2, gtsam.PriorFactorRot2, 1,
                    gtsam.fastSyncRot2, lambda result, key: result.atRot2(key))

    def test_ordering_selection(self):
        """Test generated and caller-supplied FAST-Sync orderings."""
        values = [Rot2(0.2), Rot2(0.7), Rot2(-0.4)]
        graph, keys = self._triangle(values, gtsam.BetweenFactorRot2,
                                     gtsam.PriorFactorRot2, 1)
        colamd = gtsam.fastSyncRot2(
            graph, gtsam.Ordering.OrderingType.COLAMD)
        custom = gtsam.fastSyncRot2(
            graph, gtsam.Ordering(keys=[keys[2], keys[0], keys[1]]))
        for key, expected in zip(keys, values):
            self.assertTrue(expected.equals(colamd.atRot2(key), 1e-7))
            self.assertTrue(expected.equals(custom.atRot2(key), 1e-7))

    def test_rot3(self):
        """Test the fastSync<Rot3> instantiation."""
        values = [
            Rot3.Expmap(np.array([0.1, -0.2, 0.05])),
            Rot3.Expmap(np.array([-0.3, 0.1, 0.2])),
            Rot3.Expmap(np.array([0.2, 0.25, -0.1])),
        ]
        self._check(values, gtsam.BetweenFactorRot3, gtsam.PriorFactorRot3, 3,
                    gtsam.fastSyncRot3, lambda result, key: result.atRot3(key))

    def test_pose2(self):
        """Test the fastSync<Pose2> instantiation."""
        values = [Pose2(1.0, -2.0, 0.2), Pose2(2.0, 0.5, 0.7),
                  Pose2(-1.0, 1.5, -0.4)]
        self._check(values, gtsam.BetweenFactorPose2, gtsam.PriorFactorPose2, 3,
                    gtsam.fastSyncPose2, lambda result, key: result.atPose2(key))

    def test_pose3(self):
        """Test the fastSync<Pose3> instantiation."""
        values = [
            Pose3(Rot3.Expmap(np.array([0.1, -0.2, 0.05])),
                  np.array([1.0, -2.0, 0.5])),
            Pose3(Rot3.Expmap(np.array([-0.3, 0.1, 0.2])),
                  np.array([2.0, 0.5, -1.0])),
            Pose3(Rot3.Expmap(np.array([0.2, 0.25, -0.1])),
                  np.array([-1.0, 1.5, 2.0])),
        ]
        self._check(values, gtsam.BetweenFactorPose3, gtsam.PriorFactorPose3, 6,
                    gtsam.fastSyncPose3, lambda result, key: result.atPose3(key))

    def test_similarity2(self):
        """Test the fastSync<Similarity2> instantiation."""
        values = [
            Similarity2(Rot2(0.2), np.array([1.0, -2.0]), 1.1),
            Similarity2(Rot2(0.7), np.array([2.0, 0.5]), 0.9),
            Similarity2(Rot2(-0.4), np.array([-1.0, 1.5]), 1.3),
        ]
        self._check(values, gtsam.BetweenFactorSimilarity2,
                    gtsam.PriorFactorSimilarity2, 4, gtsam.fastSyncSimilarity2,
                    lambda result, key: result.atSimilarity2(key), 1e-6)

    def test_similarity3(self):
        """Test the fastSync<Similarity3> instantiation."""
        values = [
            Similarity3(Rot3.Expmap(np.array([0.1, -0.2, 0.05])),
                        np.array([1.0, -2.0, 0.5]), 1.1),
            Similarity3(Rot3.Expmap(np.array([-0.3, 0.1, 0.2])),
                        np.array([2.0, 0.5, -1.0]), 0.9),
            Similarity3(Rot3.Expmap(np.array([0.2, 0.25, -0.1])),
                        np.array([-1.0, 1.5, 2.0]), 1.3),
        ]
        self._check(values, gtsam.BetweenFactorSimilarity3,
                    gtsam.PriorFactorSimilarity3, 7, gtsam.fastSyncSimilarity3,
                    lambda result, key: result.atSimilarity3(key), 1e-6)

    def test_sl4(self):
        """Test the fastSync<SL4> instantiation."""
        values = [
            SL4.Expmap(np.linspace(0.001, 0.015, 15)),
            SL4.Expmap(np.linspace(-0.01, 0.02, 15)),
            SL4.Expmap(np.linspace(0.02, -0.01, 15)),
        ]
        self._check(values, gtsam.BetweenFactorSL4, gtsam.PriorFactorSL4, 15,
                    gtsam.fastSyncSL4, lambda result, key: result.atSL4(key), 1e-6)


if __name__ == "__main__":
    unittest.main()
