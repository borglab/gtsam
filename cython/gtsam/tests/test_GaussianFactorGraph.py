"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Linear Factor Graphs.
Author: Frank Dellaert & Gerry Chen
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import gtsam
from gtsam.utils.test_case import GtsamTestCase

import numpy as np

def create_graph():
    """Create a basic linear factor graph for testing"""
    graph = gtsam.GaussianFactorGraph()
    
    x0 = gtsam.symbol(ord('x'), 0)
    x1 = gtsam.symbol(ord('x'), 1)
    x2 = gtsam.symbol(ord('x'), 2)
    
    BETWEEN_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.ones(1))
    PRIOR_NOISE = gtsam.noiseModel_Diagonal.Sigmas(np.ones(1))

    graph.add(x1, np.eye(1), x0, -np.eye(1), np.ones(1), BETWEEN_NOISE)
    graph.add(x2, np.eye(1), x1, -np.eye(1), 2*np.ones(1), BETWEEN_NOISE)
    graph.add(x0, np.eye(1), np.zeros(1), PRIOR_NOISE)

    return graph, (x0, x1, x2)

class TestGaussianFactorGraph(GtsamTestCase):
    """Tests for Gaussian Factor Graphs."""

    def test_fg(self):
        """Test solving a linear factor graph"""
        graph, X = create_graph()
        result = graph.optimize()

        EXPECTEDX = [0, 1, 3]

        # check solutions
        self.assertAlmostEqual(EXPECTEDX[0], result.at(X[0]), delta=1e-8)
        self.assertAlmostEqual(EXPECTEDX[1], result.at(X[1]), delta=1e-8)
        self.assertAlmostEqual(EXPECTEDX[2], result.at(X[2]), delta=1e-8)

    def test_convertNonlinear(self):
        """Test converting a linear factor graph to a nonlinear one"""
        graph, X = create_graph()

        EXPECTEDM = [1, 2, 3]

        # create nonlinear factor graph for marginalization
        nfg = gtsam.LinearContainerFactor.ConvertLinearGraph(graph)
        optimizer = gtsam.LevenbergMarquardtOptimizer(nfg, gtsam.Values())
        nlresult = optimizer.optimizeSafely()

        # marginalize
        marginals = gtsam.Marginals(nfg, nlresult)
        m = [marginals.marginalCovariance(x) for x in X]

        # check linear marginalizations
        self.assertAlmostEqual(EXPECTEDM[0], m[0], delta=1e-8)
        self.assertAlmostEqual(EXPECTEDM[1], m[1], delta=1e-8)
        self.assertAlmostEqual(EXPECTEDM[2], m[2], delta=1e-8)
    
    def test_linearMarginalization(self):
        """Marginalize a linear factor graph"""
        graph, X = create_graph()
        result = graph.optimize()

        EXPECTEDM = [1, 2, 3]

        # linear factor graph marginalize
        marginals = gtsam.Marginals(graph, result)
        m = [marginals.marginalCovariance(x) for x in X]

        # check linear marginalizations
        self.assertAlmostEqual(EXPECTEDM[0], m[0], delta=1e-8)
        self.assertAlmostEqual(EXPECTEDM[1], m[1], delta=1e-8)
        self.assertAlmostEqual(EXPECTEDM[2], m[2], delta=1e-8)

if __name__ == '__main__':
    unittest.main()
