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
        model = gtsam.noiseModel.Unit.Create(6)
        factor = gtsam.PriorFactorPose3(key, priorPose3, model)
        values.insert(key, priorPose3)
        self.assertEqual(factor.error(values), 0)

        key = 3
        priorVector = np.array([0., 0., 0.])
        model = gtsam.noiseModel.Unit.Create(3)
        factor = gtsam.PriorFactorVector(key, priorVector, model)
        values.insert(key, priorVector)
        self.assertEqual(factor.error(values), 0)

    def test_AddPrior(self):
        """
        Test adding prior factors directly to factor graph via the .addPrior method.
        """
        # define factor graph
        graph = gtsam.NonlinearFactorGraph()

        # define and add Pose3 prior
        key = 5
        priorPose3 = gtsam.Pose3()
        model = gtsam.noiseModel.Unit.Create(6)
        graph.addPriorPose3(key, priorPose3, model)
        self.assertEqual(graph.size(), 1)

        # define and add Vector prior
        key = 3
        priorVector = np.array([0., 0., 0.])
        model = gtsam.noiseModel.Unit.Create(3)
        graph.addPriorVector(key, priorVector, model)
        self.assertEqual(graph.size(), 2)


if __name__ == "__main__":
    unittest.main()
