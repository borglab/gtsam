import unittest
from gtsam import *
from math import *
import numpy as np

class TestOdometryExample(unittest.TestCase):

    def test_OdometryExample(self):
        # Create the graph (defined in pose2SLAM.h, derived from
        # NonlinearFactorGraph)
        graph = NonlinearFactorGraph()

        # Add a Gaussian prior on pose x_1
        priorMean = Pose2(0.0, 0.0, 0.0)  # prior mean is at origin
        priorNoise = noiseModel_Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1]))  # 30cm std on x,y, 0.1 rad on theta
        # add directly to graph
        graph.add(PriorFactorPose2(1, priorMean, priorNoise))

        # Add two odometry factors
        # create a measurement for both factors (the same in this case)
        odometry = Pose2(2.0, 0.0, 0.0)
        odometryNoise = noiseModel_Diagonal.Sigmas(
            np.array([0.2, 0.2, 0.1]))  # 20cm std on x,y, 0.1 rad on theta
        graph.add(BetweenFactorPose2(1, 2, odometry, odometryNoise))
        graph.add(BetweenFactorPose2(2, 3, odometry, odometryNoise))

        # Initialize to noisy points
        initialEstimate = Values()
        initialEstimate.insertPose2(1, Pose2(0.5, 0.0, 0.2))
        initialEstimate.insertPose2(2, Pose2(2.3, 0.1, -0.2))
        initialEstimate.insertPose2(3, Pose2(4.1, 0.1, 0.1))

        # Optimize using Levenberg-Marquardt optimization with an ordering from
        # colamd
        optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate)
        result = optimizer.optimizeSafely()
        marginals = Marginals(graph, result)
        marginals.marginalCovariance(1)

        # Check first pose equality
        pose_1 = result.atPose2(1)
        self.assertTrue(pose_1.equals(Pose2(), 1e-4))

if __name__ == "__main__":
    unittest.main()
