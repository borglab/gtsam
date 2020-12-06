"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SFM unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
import gtsam.utils.visual_data_generator as generator
from gtsam import symbol
from gtsam.noiseModel import Isotropic, Diagonal
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import X, P

class TestSFMExample(GtsamTestCase):

    def test_SFMExample(self):
        options = generator.Options()
        options.triangle = False
        options.nrCameras = 10

        [data, truth] = generator.generate_data(options)

        measurementNoiseSigma = 1.0
        pointNoiseSigma = 0.1
        poseNoiseSigmas = np.array([0.001, 0.001, 0.001, 0.1, 0.1, 0.1])

        graph = gtsam.NonlinearFactorGraph()

        # Add factors for all measurements
        measurementNoise = Isotropic.Sigma(2, measurementNoiseSigma)
        for i in range(len(data.Z)):
            for k in range(len(data.Z[i])):
                j = data.J[i][k]
                graph.add(gtsam.GenericProjectionFactorCal3_S2(
                    data.Z[i][k], measurementNoise,
                    X(i), P(j), data.K))

        posePriorNoise = Diagonal.Sigmas(poseNoiseSigmas)
        graph.add(gtsam.PriorFactorPose3(X(0),
                                   truth.cameras[0].pose(), posePriorNoise))
        pointPriorNoise = Isotropic.Sigma(3, pointNoiseSigma)
        graph.add(gtsam.PriorFactorPoint3(P(0),
                                    truth.points[0], pointPriorNoise))

        # Initial estimate
        initialEstimate = gtsam.Values()
        for i in range(len(truth.cameras)):
            pose_i = truth.cameras[i].pose()
            initialEstimate.insert(X(i), pose_i)
        for j in range(len(truth.points)):
            point_j = truth.points[j]
            initialEstimate.insert(P(j), point_j)

        # Optimization
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
        for i in range(5):
            optimizer.iterate()
        result = optimizer.values()

        # Marginalization
        marginals = gtsam.Marginals(graph, result)
        marginals.marginalCovariance(P(0))
        marginals.marginalCovariance(X(0))

        # Check optimized results, should be equal to ground truth
        for i in range(len(truth.cameras)):
            pose_i = result.atPose3(X(i))
            self.gtsamAssertEquals(pose_i, truth.cameras[i].pose(), 1e-5)

        for j in range(len(truth.points)):
            point_j = result.atPoint3(P(j))
            self.gtsamAssertEquals(point_j, truth.points[j], 1e-5)

if __name__ == "__main__":
    unittest.main()
