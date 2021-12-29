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

from gtsam import (DiscreteBayesNet, DiscreteBayesTreeClique,
                   DiscreteConditional, DiscreteFactorGraph, Ordering)
from gtsam.utils.test_case import GtsamTestCase


class TestDiscreteBayesNet(GtsamTestCase):
    """Tests for Discrete Bayes Nets."""

    def test_elimination(self):
        """Test Multifrontal elimination."""

        # Define DiscreteKey pairs.
        keys = [(j, 2) for j in range(15)]

        # Create thin-tree Bayesnet.
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

        self.assertFalse(bayesTree.empty())
        self.assertEqual(12, bayesTree.size())

        # The root is P( 8 12 14), we can retrieve it by key:
        root = bayesTree[8]
        self.assertIsInstance(root, DiscreteBayesTreeClique)
        self.assertTrue(root.isRoot())
        self.assertIsInstance(root.conditional(), DiscreteConditional)


if __name__ == "__main__":
    unittest.main()
