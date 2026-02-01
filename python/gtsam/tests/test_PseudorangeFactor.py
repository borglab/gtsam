"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

PseudorangeFactor python binding unit tests.
Author: Sammy Guo
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestPseudorangeFactor(GtsamTestCase):
    def test_singularity(self):
        pos = np.zeros(3)
        values = gtsam.Values()
        values.insert(0, pos)
        values.insert(1, 0.0)

        model = gtsam.noiseModel.Unit.Create(1)
        factor = gtsam.PseudorangeFactor(0, 1, 0.0, pos, 0.0, model)
        self.assertEqual(factor.error(values), 0)
        # test print:
        factor.print("factor")

    def test_errors(self):
        satpos = np.array([0.0, 0.0, 3.0])
        model = gtsam.noiseModel.Unit.Create(1)
        factor = gtsam.PseudorangeFactor(0, 1, 4.0, satpos, 0.0, model)
        error = factor.evaluateError(np.zeros(3), 0.0)
        self.assertEqual(error[0], -1.0)

    def test_equality(self):
        satpos = np.array([0.0, 0.0, 3.0])
        model = gtsam.noiseModel.Unit.Create(1)
        factor1 = gtsam.PseudorangeFactor(0, 1, 4.0, satpos, 0.0, model)
        factor2 = gtsam.PseudorangeFactor(2, 1, 4.0, satpos, 0.0, model)
        factor3 = gtsam.PseudorangeFactor(0, 1, 40.0, satpos, 10.0, model)
        self.assertTrue(factor1.equals(factor1, 1e-6))
        self.assertFalse(factor1.equals(factor2, 1e-6))
        self.assertTrue(factor1.equals(factor3, 1e99))

    def test_serialization(self):
        satpos = np.array([0.0, 0.0, 3.0])
        model = gtsam.noiseModel.Unit.Create(1)
        factor = gtsam.PseudorangeFactor(0, 1, 4.0, satpos, 0.0, model)
        factor.serialize()


if __name__ == "__main__":
    unittest.main()
