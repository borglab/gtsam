"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Odometry unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestOdometryExample(GtsamTestCase):

    def test_OdometryExample(self):
        # Create the graph (defined in pose2SLAM.h, derived from
        # NonlinearFactorGraph)
        graph = gtsam.NonlinearFactorGraph()

        # Add a Gaussian prior on pose x_1
        priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior mean is at origin
        priorNoise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1]))  # 30cm std on x,y, 0.1 rad on theta
        # add directly to graph
        graph.add(gtsam.PriorFactorPose2(1, priorMean, priorNoise))

        # Add two odometry factors
        # create a measurement for both factors (the same in this case)
        odometry = gtsam.Pose2(2.0, 0.0, 0.0)
        odometryNoise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.2, 0.2, 0.1]))  # 20cm std on x,y, 0.1 rad on theta
        graph.add(gtsam.BetweenFactorPose2(1, 2, odometry, odometryNoise))
        graph.add(gtsam.BetweenFactorPose2(2, 3, odometry, odometryNoise))

        # Initialize to noisy points
        initialEstimate = gtsam.Values()
        initialEstimate.insert(1, gtsam.Pose2(0.5, 0.0, 0.2))
        initialEstimate.insert(2, gtsam.Pose2(2.3, 0.1, -0.2))
        initialEstimate.insert(3, gtsam.Pose2(4.1, 0.1, 0.1))

        # Optimize using Levenberg-Marquardt optimization with an ordering from
        # colamd
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
        result = optimizer.optimizeSafely()
        marginals = gtsam.Marginals(graph, result)
        marginals.marginalCovariance(1)

        # Check first pose equality
        pose_1 = result.atPose2(1)
        self.gtsamAssertEquals(pose_1, gtsam.Pose2(), 1e-4)

if __name__ == "__main__":
    unittest.main()
