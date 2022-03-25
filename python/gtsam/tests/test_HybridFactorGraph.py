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
from gtsam.symbol_shorthand import X, C
from gtsam.utils.test_case import GtsamTestCase


class TestHybridFactorGraph(GtsamTestCase):
    def test_create(self):
        noiseModel = gtsam.noiseModel.Unit.Create(3)
        dk = gtsam.DiscreteKeys()
        dk.push_back((C(0), 2))

        # print(dk.at(0))
        jf1 = gtsam.JacobianFactor(X(0), np.eye(3), np.zeros((3, 1)),
                                   noiseModel)
        jf2 = gtsam.JacobianFactor(X(0), np.eye(3), np.ones((3, 1)),
                                   noiseModel)

        gmf = gtsam.GaussianMixtureFactor.FromFactorList([X(0)], dk,
                                                         [jf1, jf2])

        hfg = gtsam.HybridFactorGraph()
        hfg.add(jf1)
        hfg.add(jf2)
        hfg.push_back(gmf)

        hfg.eliminateSequential(
            gtsam.Ordering.ColamdConstrainedLastHybridFactorGraph(
                hfg, [C(0)])).print()


if __name__ == "__main__":
    unittest.main()
