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
from gtsam import Point3, Pose2, PriorFactorPose2, Values


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


if __name__ == "__main__":
    unittest.main()
