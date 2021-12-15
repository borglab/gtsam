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

from gtsam import DiscreteFactorGraph, DiscreteKeys, DiscreteValues
from gtsam.utils.test_case import GtsamTestCase


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
        graph.add(P1, "0.9 0.3")
        graph.add(P2, "0.9 0.6")

        # Add a binary factor
        graph.add(P1, P2, "4 1 10 4")

        # Instantiate Values
        assignment = DiscreteValues()
        assignment[0] = 1
        assignment[1] = 1

        # Check if graph evaluation works ( 0.3*0.6*4 )
        self.assertAlmostEqual(.72, graph.evaluate(assignment))

        # Creating a new test with third node and adding unary and ternary factors on it
        graph.add(P3, "0.9 0.2 0.5")
        keys = DiscreteKeys()
        keys.push_back(P1)
        keys.push_back(P2)
        keys.push_back(P3)
        graph.add(keys, "1 2 3 4 5 6 7 8 9 10 11 12")

        # Below assignment lead to selecting the 8th index in the ternary factor table
        assignment[0] = 1
        assignment[1] = 0
        assignment[2] = 1

        # Check if graph evaluation works (0.3*0.9*1*0.2*8)
        self.assertAlmostEqual(4.32, graph.evaluate(assignment))

        # Below assignment lead to selecting the 3rd index in the ternary factor table
        assignment[0] = 0
        assignment[1] = 1
        assignment[2] = 0

        # Check if graph evaluation works (0.9*0.6*1*0.9*4)
        self.assertAlmostEqual(1.944, graph.evaluate(assignment))

        # Check if graph product works
        product = graph.product()
        self.assertAlmostEqual(1.944, product.evaluate(assignment))

    def test_optimize(self):
        """Test constructing and optizing a discrete factor graph."""

        # Three keys
        C = (0, 2)
        B = (1, 2)
        A = (2, 2)

        # A simple factor graph (A)-fAC-(C)-fBC-(B)
        # with smoothness priors
        graph = DiscreteFactorGraph()
        graph.add(A, C, "3 1 1 3")
        graph.add(C, B, "3 1 1 3")

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
        graph.add(C, A, "0.2 0.8 0.3 0.7")
        graph.add(C, B, "0.1 0.9 0.4 0.6")

        actualMPE = graph.optimize()

        expectedMPE = DiscreteValues()
        expectedMPE[0] = 0
        expectedMPE[1] = 1
        expectedMPE[2] = 1
        self.assertEqual(list(actualMPE.items()),
                         list(expectedMPE.items()))


if __name__ == "__main__":
    unittest.main()
