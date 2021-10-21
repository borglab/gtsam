"""
GTSAM Copyright 2010, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)
See LICENSE for the license information

Author: John Lambert (Python)
"""

import numpy as np

import gtsam
from gtsam import Pose2, PriorFactorPose2, Values


def vector3(x: float, y: float, z: float) -> np.ndarray:
    """Create 3d double numpy array."""
    return np.array([x, y, z], dtype=float)


def test_lago() -> None:
    """Smokescreen to ensure LAGO can be imported and run on toy data stored in a g2o file."""
    g2oFile = gtsam.findExampleDataFile("noisyToyGraph.txt")

    graph = gtsam.NonlinearFactorGraph()
    graph, initial = gtsam.readG2o(g2oFile)

    # Add prior on the pose having index (key) = 0
    priorModel = gtsam.noiseModel.Diagonal.Variances(vector3(1e-6, 1e-6, 1e-8))
    graph.add(PriorFactorPose2(0, Pose2(), priorModel))

    estimateLago: Values = gtsam.lago.initialize(graph)
    assert isinstance(estimateLago, Values)
