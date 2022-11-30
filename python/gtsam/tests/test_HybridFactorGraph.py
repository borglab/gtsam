"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Factor Graphs.
Author: Fan Jiang
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import C, X
from gtsam.utils.test_case import GtsamTestCase


class TestHybridGaussianFactorGraph(GtsamTestCase):
    """Unit tests for HybridGaussianFactorGraph."""

    def test_create(self):
        """Test contruction of hybrid factor graph."""
        noiseModel = gtsam.noiseModel.Unit.Create(3)
        dk = gtsam.DiscreteKeys()
        dk.push_back((C(0), 2))

        jf1 = gtsam.JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)),
                                   noiseModel)
        jf2 = gtsam.JacobianFactor(X(0), np.eye(3), np.ones((3, 1)),
                                   noiseModel)

        gmf = gtsam.GaussianMixtureFactor.FromFactors([X(0)], dk, [jf1, jf2])

        hfg = gtsam.HybridGaussianFactorGraph()
        hfg.add(jf1)
        hfg.add(jf2)
        hfg.push_back(gmf)

        hbn = hfg.eliminateSequential(
            gtsam.Ordering.ColamdConstrainedLastHybridGaussianFactorGraph(
                hfg, [C(0)]))

        # print("hbn = ", hbn)
        self.assertEqual(hbn.size(), 2)

        mixture = hbn.at(0).inner()
        self.assertIsInstance(mixture, gtsam.GaussianMixture)
        self.assertEqual(len(mixture.keys()), 2)

        discrete_conditional = hbn.at(hbn.size() - 1).inner()
        self.assertIsInstance(discrete_conditional, gtsam.DiscreteConditional)

    def test_optimize(self):
        """Test contruction of hybrid factor graph."""
        noiseModel = gtsam.noiseModel.Unit.Create(3)
        dk = gtsam.DiscreteKeys()
        dk.push_back((C(0), 2))

        jf1 = gtsam.JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)),
                                   noiseModel)
        jf2 = gtsam.JacobianFactor(X(0), np.eye(3), np.ones((3, 1)),
                                   noiseModel)

        gmf = gtsam.GaussianMixtureFactor.FromFactors([X(0)], dk, [jf1, jf2])

        hfg = gtsam.HybridGaussianFactorGraph()
        hfg.add(jf1)
        hfg.add(jf2)
        hfg.push_back(gmf)

        dtf = gtsam.DecisionTreeFactor([(C(0), 2)],"0 1")
        hfg.add(dtf)

        hbn = hfg.eliminateSequential(
            gtsam.Ordering.ColamdConstrainedLastHybridGaussianFactorGraph(
                hfg, [C(0)]))

        # print("hbn = ", hbn)
        hv = hbn.optimize()
        self.assertEqual(hv.atDiscrete(C(0)), 1)

if __name__ == "__main__":
    unittest.main()
