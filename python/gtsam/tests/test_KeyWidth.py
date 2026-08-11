"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Regression tests for 64-bit keys on platforms where size_t is 32 bits.
"""

# pylint: disable=no-member

import unittest

import numpy as np

import gtsam
from gtsam.symbol_shorthand import X


class TestKeyWidth(unittest.TestCase):
    """Checks that wrapped keys are never narrowed to size_t."""

    def test_large_symbol_indices(self):
        """Symbol helpers should preserve indices larger than 32 bits."""
        index = (1 << 32) + 7

        key = gtsam.symbol("x", index)
        self.assertEqual(gtsam.symbolIndex(key), index)
        self.assertEqual(X(index), key)

        labeled = gtsam.LabeledSymbol(ord("x"), ord("A"), index)
        self.assertEqual(labeled.index(), index)
        labeled_key = gtsam.mrsymbol(ord("x"), ord("A"), index)
        self.assertEqual(gtsam.mrsymbolIndex(labeled_key), index)

    def test_keyed_collections(self):
        """Keyed collection APIs should accept an encoded symbolic key."""
        key = X(0)

        keys = gtsam.KeyList()
        keys.push_back(key)
        self.assertEqual(keys.front(), key)
        self.assertEqual(keys.back(), key)

        ordering = gtsam.Ordering([key])
        self.assertEqual(ordering.at(0), key)

        values = gtsam.VectorValues()
        values.insert(key, np.array([1.0, 2.0]))
        self.assertTrue(values.exists(key))
        self.assertEqual(values.dim(key), 2)
        np.testing.assert_array_equal(values.at(key), np.array([1.0, 2.0]))

        groups = gtsam.KeyGroupMap()
        groups.insert2(key, -1)
        self.assertEqual(groups.at(key), -1)

        assignment = gtsam.DiscreteValues()
        assignment[key] = 1
        self.assertEqual(len(assignment), 1)
        self.assertEqual(list(assignment.keys()), [key])

        density = gtsam.GaussianDensity.FromMeanAndStddev(
            key, np.array([0.0]), 1.0
        )
        self.assertEqual(gtsam.KalmanFilter.step(density), key)

    def test_keyed_factor_constructor(self):
        """Factor constructors should retain symbolic keys."""
        key = X(0)
        equality = gtsam.NonlinearEqualityPose2(key, gtsam.Pose2())
        self.assertEqual(equality.keys(), [key])

        attitude = gtsam.Rot3AttitudeFactor(
            key,
            gtsam.Unit3(np.array([0.0, 0.0, 1.0])),
            gtsam.noiseModel.Isotropic.Sigma(2, 1.0),
        )
        self.assertEqual(attitude.keys(), [key])

    def test_joint_marginal_key_query(self):
        """Joint-marginal lookup should take Keys rather than indices."""
        key = X(0)
        graph = gtsam.NonlinearFactorGraph()
        graph.add(
            gtsam.PriorFactorPose2(
                key,
                gtsam.Pose2(),
                gtsam.noiseModel.Isotropic.Sigma(3, 1.0),
            )
        )
        values = gtsam.Values()
        values.insert(key, gtsam.Pose2())

        joint = gtsam.Marginals(graph, values).jointMarginalCovariance([key])
        np.testing.assert_array_equal(joint.at(key, key), np.eye(3))

    def test_discrete_bayes_tree_key_query(self):
        """Bayes-tree lookup should take a Key rather than an index."""
        key0, key1 = X(0), X(1)
        graph = gtsam.DiscreteFactorGraph()
        graph.add([(key0, 2)], "1 1")
        graph.add([(key0, 2), (key1, 2)], "1 2 3 4")

        tree = graph.eliminateMultifrontal(gtsam.Ordering([key0, key1]))
        self.assertIsNotNone(tree[key0])


if __name__ == "__main__":
    unittest.main()
