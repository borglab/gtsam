"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for Hybrid Nonlinear Factor Graphs.
Author: Fan Jiang
"""
# pylint: disable=invalid-name, no-name-in-module, no-member

from __future__ import print_function

import unittest

import gtsam
import numpy as np
from gtsam.symbol_shorthand import C, X
from gtsam.utils.test_case import GtsamTestCase
from gtsam import BetweenFactorPoint3, noiseModel, PriorFactorPoint3, Point3


class TestHybridGaussianFactorGraph(GtsamTestCase):
    """Unit tests for HybridGaussianFactorGraph."""

    def test_nonlinear_hybrid(self):
        nlfg = gtsam.HybridNonlinearFactorGraph()
        dk = gtsam.DiscreteKeys()
        dk.push_back((10, 2))
        nlfg.push_back(BetweenFactorPoint3(1, 2, Point3(
            1, 2, 3), noiseModel.Diagonal.Variances([1, 1, 1])))
        nlfg.push_back(
            PriorFactorPoint3(2, Point3(1, 2, 3),
                              noiseModel.Diagonal.Variances([0.5, 0.5, 0.5])))
        nlfg.push_back(
            gtsam.MixtureFactor([1], dk, [
                PriorFactorPoint3(1, Point3(0, 0, 0),
                                  noiseModel.Unit.Create(3)),
                PriorFactorPoint3(1, Point3(1, 2, 1),
                                  noiseModel.Unit.Create(3))
            ]))
        nlfg.push_back(gtsam.DecisionTreeFactor((10, 2), "1 3"))
        values = gtsam.Values()
        values.insert_point3(1, Point3(0, 0, 0))
        values.insert_point3(2, Point3(2, 3, 1))
        hfg = nlfg.linearize(values)
        o = gtsam.Ordering()
        o.push_back(1)
        o.push_back(2)
        o.push_back(10)
        hbn = hfg.eliminateSequential(o)
        hbv = hbn.optimize()
        self.assertEqual(hbv.atDiscrete(10), 0)


if __name__ == "__main__":
    unittest.main()
