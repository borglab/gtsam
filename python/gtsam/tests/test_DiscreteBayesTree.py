"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Bayes trees.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

import numpy as np
from gtsam.symbol_shorthand import A, X
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import (DiscreteBayesNet, DiscreteBayesTreeClique,
                   DiscreteConditional, DiscreteFactorGraph,
                   DiscreteValues, Ordering)


class TestDiscreteBayesNet(GtsamTestCase):
    """Tests for Discrete Bayes Nets."""

    def test_elimination(self):
        """Test Multifrontal elimination."""

        # Define DiscreteKey pairs.
        keys = [(j, 2) for j in range(15)]

        # Create thin-tree Bayes net.
        bayesNet = DiscreteBayesNet()

        bayesNet.add(keys[0], [keys[8], keys[12]], "2/3 1/4 3/2 4/1")
        bayesNet.add(keys[1], [keys[8], keys[12]], "4/1 2/3 3/2 1/4")
        bayesNet.add(keys[2], [keys[9], keys[12]], "1/4 8/2 2/3 4/1")
        bayesNet.add(keys[3], [keys[9], keys[12]], "1/4 2/3 3/2 4/1")

        bayesNet.add(keys[4], [keys[10], keys[13]], "2/3 1/4 3/2 4/1")
        bayesNet.add(keys[5], [keys[10], keys[13]], "4/1 2/3 3/2 1/4")
        bayesNet.add(keys[6], [keys[11], keys[13]], "1/4 3/2 2/3 4/1")
        bayesNet.add(keys[7], [keys[11], keys[13]], "1/4 2/3 3/2 4/1")

        bayesNet.add(keys[8], [keys[12], keys[14]], "T 1/4 3/2 4/1")
        bayesNet.add(keys[9], [keys[12], keys[14]], "4/1 2/3 F 1/4")
        bayesNet.add(keys[10], [keys[13], keys[14]], "1/4 3/2 2/3 4/1")
        bayesNet.add(keys[11], [keys[13], keys[14]], "1/4 2/3 3/2 4/1")

        bayesNet.add(keys[12], [keys[14]], "3/1 3/1")
        bayesNet.add(keys[13], [keys[14]], "1/3 3/1")

        bayesNet.add(keys[14], "1/3")

        # Create a factor graph out of the Bayes net.
        factorGraph = DiscreteFactorGraph(bayesNet)

        # Create a BayesTree out of the factor graph.
        ordering = Ordering()
        for j in range(15):
            ordering.push_back(j)
        bayesTree = factorGraph.eliminateMultifrontal(ordering)

        # Uncomment these for visualization:
        # print(bayesTree)
        # for key in range(15):
        #     bayesTree[key].printSignature()
        # bayesTree.saveGraph("test_DiscreteBayesTree.dot")

        # The root is P( 8 12 14), we can retrieve it by key:
        root = bayesTree[8]
        self.assertIsInstance(root, DiscreteBayesTreeClique)
        self.assertTrue(root.isRoot())
        self.assertIsInstance(root.conditional(), DiscreteConditional)

        # Test all methods in DiscreteBayesTree
        self.gtsamAssertEquals(bayesTree, bayesTree)

        # Check value at 0
        zero_values = DiscreteValues()
        for j in range(15):
            zero_values[j] = 0
        value_at_zeros = bayesTree.evaluate(zero_values)
        self.assertAlmostEqual(value_at_zeros, 0.0)

        # Check value at max
        values_star = factorGraph.optimize()
        max_value = bayesTree.evaluate(values_star)
        self.assertAlmostEqual(max_value, 0.002548)

        # Check operator sugar
        max_value = bayesTree(values_star)
        self.assertAlmostEqual(max_value, 0.002548)

        self.assertFalse(bayesTree.empty())
        self.assertEqual(12, bayesTree.size())

    def test_discrete_bayes_tree_lookup(self):
        """Check that we can have a multi-frontal lookup table."""
        # Make a small planning-like graph: 3 states, 2 actions
        graph = DiscreteFactorGraph()
        x1, x2, x3 = (X(1), 3), (X(2), 3), (X(3), 3)
        a1, a2 = (A(1), 2), (A(2), 2)

        # Constraint on start and goal
        graph.add([x1], np.array([1, 0, 0]))
        graph.add([x3], np.array([0, 0, 1]))

        # Should I stay or should I go?
        # "Reward" (exp(-cost)) for an action is 10, and rewards multiply:
        r = 10
        table = np.array([
            r, 0, 0, 0, r, 0,  # x1 = 0
            0, r, 0, 0, 0, r,  # x1 = 1
            0, 0, r, 0, 0, r   # x1 = 2
        ])
        graph.add([x1, a1, x2], table)
        graph.add([x2, a2, x3], table)

        # Eliminate for MPE (maximum probable explanation).
        ordering = Ordering(keys=[A(2), X(3), X(1), A(1), X(2)])
        lookup = graph.eliminateMultifrontal(ordering, gtsam.EliminateForMPE)

        # Check that the lookup table is correct
        assert lookup.size() == 2
        lookup_x1_a1_x2 = lookup[X(1)].conditional()
        assert lookup_x1_a1_x2.nrFrontals() == 3
        # Check that sum is 1.0 (not 100, as we now normalize to prevent underflow)
        empty = gtsam.DiscreteValues()
        self.assertAlmostEqual(lookup_x1_a1_x2.sum(3)(empty), 1.0)
        # And that only non-zero reward is for x1 a1 x2 == 0 1 1
        values = DiscreteValues()
        values[X(1)] = 0
        values[A(1)] = 1
        values[X(2)] = 1
        self.assertAlmostEqual(lookup_x1_a1_x2(values), 1.0)

        lookup_a2_x3 = lookup[X(3)].conditional()
        # Check that the sum depends on x2 and is non-zero only for x2 in {1, 2}
        sum_x2 = lookup_a2_x3.sum(2)
        values = DiscreteValues()
        values[X(2)] = 0
        self.assertAlmostEqual(sum_x2(values), 0)
        values[X(2)] = 1
        self.assertAlmostEqual(sum_x2(values), 1.0)  # not 10, as we normalize
        values[X(2)] = 2
        self.assertAlmostEqual(sum_x2(values), 2.0)  # not 20, as we normalize
        assert lookup_a2_x3.nrFrontals() == 2
        # And that the non-zero rewards are for x2 a2 x3 == 1 1 2
        values = DiscreteValues()
        values[X(2)] = 1
        values[A(2)] = 1
        values[X(3)] = 2
        self.assertAlmostEqual(lookup_a2_x3(values), 1.0)  # not 10...

if __name__ == "__main__":
    unittest.main()
