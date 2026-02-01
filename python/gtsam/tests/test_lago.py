"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)
See LICENSE for the license information

Author: John Lambert (Python)
"""

import unittest

import numpy as np

import gtsam
from gtsam import BetweenFactorPose2, Point3, Pose2, PriorFactorPose2, Values


class TestLago(unittest.TestCase):
    """Test selected LAGO methods."""

    def test_initialize(self) -> None:
        """Smokescreen to ensure LAGO can be imported and run on toy data stored in a g2o file."""
        g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt")

        graph = gtsam.NonlinearFactorGraph()
        graph, initial = gtsam.readG2o(g2oFile)

        # Add prior on the pose having index (key) = 0
        priorModel = gtsam.noiseModel.Diagonal.Variances(Point3(1e-6, 1e-6, 1e-8))
        graph.add(PriorFactorPose2(0, Pose2(), priorModel))

        estimateLago: Values = gtsam.lago.initialize(graph)
        assert isinstance(estimateLago, Values)

    def test_initialize2(self) -> None:
        """Smokescreen to ensure LAGO can be imported and run on toy data stored in a g2o file."""
        # 1. Create a NonlinearFactorGraph with Pose2 factors
        graph = gtsam.NonlinearFactorGraph()

        # Add a prior on the first pose
        prior_mean = Pose2(0.0, 0.0, 0.0)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
        graph.add(PriorFactorPose2(0, prior_mean, prior_noise))

        # Add odometry factors (simulating moving in a square)
        odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))
        graph.add(BetweenFactorPose2(0, 1, Pose2(2.0, 0.0, 0.0), odometry_noise))
        graph.add(BetweenFactorPose2(1, 2, Pose2(2.0, 0.0, np.pi / 2), odometry_noise))
        graph.add(BetweenFactorPose2(2, 3, Pose2(2.0, 0.0, np.pi / 2), odometry_noise))
        graph.add(BetweenFactorPose2(3, 4, Pose2(2.0, 0.0, np.pi / 2), odometry_noise))

        # Add a loop closure factor
        loop_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.25, 0.25, 0.15]))
        # Ideal loop closure would be Pose2(2.0, 0.0, np.pi/2)
        measured_loop = Pose2(2.1, 0.1, np.pi / 2 + 0.05)
        graph.add(BetweenFactorPose2(4, 0, measured_loop, loop_noise))

        estimateLago: Values = gtsam.lago.initialize(graph)
        assert isinstance(estimateLago, Values)


if __name__ == "__main__":
    unittest.main()
