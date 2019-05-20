"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

PriorFactor unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestPriorFactor(GtsamTestCase):

    def test_PriorFactor(self):
        values = gtsam.Values()

        key = 5
        priorPose3 = gtsam.Pose3()
        model = gtsam.noiseModel_Unit.Create(6)
        factor = gtsam.PriorFactorPose3(key, priorPose3, model)
        values.insert(key, priorPose3)
        self.assertEqual(factor.error(values), 0)

        key = 3
        priorVector = np.array([0., 0., 0.])
        model = gtsam.noiseModel_Unit.Create(3)
        factor = gtsam.PriorFactorVector(key, priorVector, model)
        values.insert(key, priorVector)
        self.assertEqual(factor.error(values), 0)

if __name__ == "__main__":
    unittest.main()
