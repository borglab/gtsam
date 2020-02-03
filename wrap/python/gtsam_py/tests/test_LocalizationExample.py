"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Localization unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestLocalizationExample(GtsamTestCase):

    def test_LocalizationExample(self):
        # Create the graph (defined in pose2SLAM.h, derived from
        # NonlinearFactorGraph)
        graph = gtsam.NonlinearFactorGraph()

        # Add two odometry factors
        # create a measurement for both factors (the same in this case)
        odometry = gtsam.Pose2(2.0, 0.0, 0.0)
        odometryNoise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.2, 0.2, 0.1]))  # 20cm std on x,y, 0.1 rad on theta
        graph.add(gtsam.BetweenFactorPose2(0, 1, odometry, odometryNoise))
        graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometryNoise))

        # Add three "GPS" measurements
        # We use Pose2 Priors here with high variance on theta
        groundTruth = gtsam.Values()
        groundTruth.insert(0, gtsam.Pose2(0.0, 0.0, 0.0))
        groundTruth.insert(1, gtsam.Pose2(2.0, 0.0, 0.0))
        groundTruth.insert(2, gtsam.Pose2(4.0, 0.0, 0.0))
        model = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 10.]))
        for i in range(3):
            graph.add(gtsam.PriorFactorPose2(i, groundTruth.atPose2(i), model))

        # Initialize to noisy points
        initialEstimate = gtsam.Values()
        initialEstimate.insert(0, gtsam.Pose2(0.5, 0.0, 0.2))
        initialEstimate.insert(1, gtsam.Pose2(2.3, 0.1, -0.2))
        initialEstimate.insert(2, gtsam.Pose2(4.1, 0.1, 0.1))

        # Optimize using Levenberg-Marquardt optimization with an ordering from
        # colamd
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
        result = optimizer.optimizeSafely()

        # Plot Covariance Ellipses
        marginals = gtsam.Marginals(graph, result)
        P = [None] * result.size()
        for i in range(0, result.size()):
            pose_i = result.atPose2(i)
            self.gtsamAssertEquals(pose_i, groundTruth.atPose2(i), 1e-4)
            P[i] = marginals.marginalCovariance(i)

if __name__ == "__main__":
    unittest.main()
