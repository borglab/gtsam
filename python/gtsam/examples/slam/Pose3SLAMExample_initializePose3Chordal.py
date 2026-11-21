"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Initialize PoseSLAM with Chordal init
Author: Luca Carlone, Frank Dellaert (python port)
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import gtsam
import numpy as np


def main():
    """Main runner."""
    # Read graph from file
    g2oFile = gtsam.findExampleDataFile("pose3example.txt")

    is3D = True
    graph, initial = gtsam.readG2o(g2oFile, is3D)

    # Add prior on the first key. TODO: assumes first key ios z
    priorModel = gtsam.noiseModel.Diagonal.Variances(
        np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
    firstKey = initial.keys()[0]
    graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), priorModel))

    # Initializing Pose3 - chordal relaxation
    initialization = gtsam.InitializePose3.initialize(graph)

    print(initialization)


if __name__ == "__main__":
    main()
