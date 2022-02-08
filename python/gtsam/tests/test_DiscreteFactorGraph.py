"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Discrete Factor Graphs.
Author: Frank Dellaert
"""

# pylint: disable=no-name-in-module, invalid-name

import unittest

from gtsam import DiscreteFactorGraph, DiscreteKeys, DiscreteValues, Ordering
from gtsam.utils.test_case import GtsamTestCase

OrderingType = Ordering.OrderingType


class TestDiscreteFactorGraph(GtsamTestCase):
    """Tests for Discrete Factor Graphs."""

    def test_evaluation(self):
        """Test constructing and evaluating a discrete factor graph."""

        # Three keys
        P1 = (0, 2)
        P2 = (1, 2)
        P3 = (2, 3)

        # Create the DiscreteFactorGraph
        graph = DiscreteFactorGraph()

        # Add two unary factors (priors)
        graph.add(P1, [0.9, 0.3])
        graph.add(P2, "0.9 0.6")

        # Add a binary factor
        graph.add([P1, P2], "4 1 10 4")

        # Instantiate Values
        assignment = DiscreteValues()
        assignment[0] = 1
        assignment[1] = 1

        # Check if graph evaluation works ( 0.3*0.6*4 )
        self.assertAlmostEqual(.72, graph(assignment))

        # Create a new test with third node and adding unary and ternary factor
        graph.add(P3, "0.9 0.2 0.5")
        keys = DiscreteKeys()
        keys.push_back(P1)
        keys.push_back(P2)
        keys.push_back(P3)
        graph.add(keys, "1 2 3 4 5 6 7 8 9 10 11 12")

        # Below assignment selects the 8th index in the ternary factor table
        assignment[0] = 1
        assignment[1] = 0
        assignment[2] = 1

        # Check if graph evaluation works (0.3*0.9*1*0.2*8)
        self.assertAlmostEqual(4.32, graph(assignment))

        # Below assignment selects the 3rd index in the ternary factor table
        assignment[0] = 0
        assignment[1] = 1
        assignment[2] = 0

        # Check if graph evaluation works (0.9*0.6*1*0.9*4)
        self.assertAlmostEqual(1.944, graph(assignment))

        # Check if graph product works
        product = graph.product()
        self.assertAlmostEqual(1.944, product(assignment))

    def test_optimize(self):
        """Test constructing and optizing a discrete factor graph."""

        # Three keys
        C = (0, 2)
        B = (1, 2)
        A = (2, 2)

        # A simple factor graph (A)-fAC-(C)-fBC-(B)
        # with smoothness priors
        graph = DiscreteFactorGraph()
        graph.add([A, C], "3 1 1 3")
        graph.add([C, B], "3 1 1 3")

        # Test optimization
        expectedValues = DiscreteValues()
        expectedValues[0] = 0
        expectedValues[1] = 0
        expectedValues[2] = 0
        actualValues = graph.optimize()
        self.assertEqual(list(actualValues.items()),
                         list(expectedValues.items()))

    def test_MPE(self):
        """Test maximum probable explanation (MPE): same as optimize."""

        # Declare a bunch of keys
        C, A, B = (0, 2), (1, 2), (2, 2)

        # Create Factor graph
        graph = DiscreteFactorGraph()
        graph.add([C, A], "0.2 0.8 0.3 0.7")
        graph.add([C, B], "0.1 0.9 0.4 0.6")

        # We know MPE
        mpe = DiscreteValues()
        mpe[0] = 0
        mpe[1] = 1
        mpe[2] = 1

        # Use maxProduct
        dag = graph.maxProduct(OrderingType.COLAMD)
        actualMPE = dag.argmax()
        self.assertEqual(list(actualMPE.items()),
                         list(mpe.items()))

        # All in one
        actualMPE2 = graph.optimize()
        self.assertEqual(list(actualMPE2.items()),
                         list(mpe.items()))

    def test_sumProduct(self):
        """Test sumProduct."""

        # Declare a bunch of keys
        C, A, B = (0, 2), (1, 2), (2, 2)

        # Create Factor graph
        graph = DiscreteFactorGraph()
        graph.add([C, A], "0.2 0.8 0.3 0.7")
        graph.add([C, B], "0.1 0.9 0.4 0.6")

        # We know MPE
        mpe = DiscreteValues()
        mpe[0] = 0
        mpe[1] = 1
        mpe[2] = 1

        # Use default sumProduct
        bayesNet = graph.sumProduct()
        mpeProbability = bayesNet(mpe)
        self.assertAlmostEqual(mpeProbability, 0.36)  # regression

        # Use sumProduct
        for ordering_type in [OrderingType.COLAMD, OrderingType.METIS, OrderingType.NATURAL,
                              OrderingType.CUSTOM]:
            bayesNet = graph.sumProduct(ordering_type)
            self.assertEqual(bayesNet(mpe), mpeProbability)


if __name__ == "__main__":
    unittest.main()
