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

        density = gtsam.GaussianDensity.FromMeanAndStddev(
            key, np.array([0.0]), 1.0
        )
        self.assertEqual(gtsam.KalmanFilter.step(density), key)

    def test_keyed_factor_constructors(self):
        """Factor constructors should retain symbolic keys."""
        key0, key1, key2 = X(0), X(1), X(2)

        equality = gtsam.NonlinearEqualityPose2(key0, gtsam.Pose2())
        self.assertEqual(equality.keys(), [key0])

        velocity = gtsam.ConstantVelocityFactor(
            key0,
            key1,
            1.0,
            gtsam.noiseModel.Isotropic.Sigma(9, 1.0),
        )
        self.assertEqual(velocity.keys(), [key0, key1])

        mode = (gtsam.symbol("m", 0), 2)
        parameters = [(np.array([0.0]), 1.0), (np.array([1.0]), 1.0)]
        conditional = gtsam.HybridGaussianConditional(
            mode, key0, np.eye(1), key1, np.eye(1), key2, parameters
        )
        self.assertEqual(conditional.keys()[:3], [key0, key1, key2])

    def test_discrete_bayes_tree_key_queries(self):
        """Bayes-tree lookup and joint queries should take Keys, not indices."""
        key0, key1 = X(0), X(1)
        graph = gtsam.DiscreteFactorGraph()
        graph.add([(key0, 2)], "1 1")
        graph.add([(key0, 2), (key1, 2)], "1 2 3 4")

        tree = graph.eliminateMultifrontal(gtsam.Ordering([key0, key1]))
        self.assertIsNotNone(tree[key0])
        self.assertIsNotNone(tree.clique(key1))
        self.assertEqual(tree.joint(key0, key1).size(), 2)
        self.assertEqual(tree.jointBayesNet(key0, key1).size(), 2)


if __name__ == "__main__":
    unittest.main()
