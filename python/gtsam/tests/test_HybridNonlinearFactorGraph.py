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

from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import BetweenFactorPoint3, Point3, PriorFactorPoint3, noiseModel


class TestHybridGaussianFactorGraph(GtsamTestCase):
    """Unit tests for HybridGaussianFactorGraph."""

    def test_nonlinear_hybrid(self):
        nlfg = gtsam.HybridNonlinearFactorGraph()
        nlfg.push_back(
            BetweenFactorPoint3(1, 2, Point3(1, 2, 3),
                                noiseModel.Diagonal.Variances([1, 1, 1])))
        nlfg.push_back(
            PriorFactorPoint3(2, Point3(1, 2, 3),
                              noiseModel.Diagonal.Variances([0.5, 0.5, 0.5])))

        factors = [(PriorFactorPoint3(1, Point3(0, 0, 0),
                                      noiseModel.Unit.Create(3)), 0.0),
                   (PriorFactorPoint3(1, Point3(1, 2, 1),
                                      noiseModel.Unit.Create(3)), 0.0)]
        mode = (10, 2)
        nlfg.push_back(gtsam.HybridNonlinearFactor(mode, factors))
        nlfg.push_back(gtsam.DecisionTreeFactor(mode, "1 3"))
        values = gtsam.Values()
        values.insert_point3(1, Point3(0, 0, 0))
        values.insert_point3(2, Point3(2, 3, 1))
        hfg = nlfg.linearize(values)
        hbn = hfg.eliminateSequential()
        hbv = hbn.optimize()
        self.assertEqual(hbv.atDiscrete(10), 0)


if __name__ == "__main__":
    unittest.main()
