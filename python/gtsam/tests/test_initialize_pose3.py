"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for 3D SLAM initialization, using rotation relaxation.
Author: Luca Carlone and Frank Dellaert (Python)
"""
# pylint: disable=invalid-name, E1101, E0611
import unittest

import numpy as np

import gtsam
from gtsam import NonlinearFactorGraph, Point3, Pose3, Rot3, Values
from gtsam.utils.test_case import GtsamTestCase

x0, x1, x2, x3 = 0, 1, 2, 3


class TestValues(GtsamTestCase):

    def setUp(self):

        model = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)

        # We consider a small graph:
        #                            symbolic FG
        #               x2               0  1
        #             / | \              1  2
        #            /  |  \             2  3
        #          x3   |   x1           2  0
        #           \   |   /            0  3
        #            \  |  /
        #               x0
        #
        p0 = Point3(0, 0, 0)
        self.R0 = Rot3.Expmap(np.array([0.0, 0.0, 0.0]))
        p1 = Point3(1, 2, 0)
        self.R1 = Rot3.Expmap(np.array([0.0, 0.0, 1.570796]))
        p2 = Point3(0, 2, 0)
        self.R2 = Rot3.Expmap(np.array([0.0, 0.0, 3.141593]))
        p3 = Point3(-1, 1, 0)
        self.R3 = Rot3.Expmap(np.array([0.0, 0.0, 4.712389]))

        pose0 = Pose3(self.R0, p0)
        pose1 = Pose3(self.R1, p1)
        pose2 = Pose3(self.R2, p2)
        pose3 = Pose3(self.R3, p3)

        g = NonlinearFactorGraph()
        g.add(gtsam.BetweenFactorPose3(x0, x1, pose0.between(pose1), model))
        g.add(gtsam.BetweenFactorPose3(x1, x2, pose1.between(pose2), model))
        g.add(gtsam.BetweenFactorPose3(x2, x3, pose2.between(pose3), model))
        g.add(gtsam.BetweenFactorPose3(x2, x0, pose2.between(pose0), model))
        g.add(gtsam.BetweenFactorPose3(x0, x3, pose0.between(pose3), model))
        g.add(gtsam.PriorFactorPose3(x0, pose0, model))
        self.graph = g

    def test_buildPose3graph(self):
        pose3graph = gtsam.InitializePose3.buildPose3graph(self.graph)

    def test_orientations(self):
        pose3Graph = gtsam.InitializePose3.buildPose3graph(self.graph)
        initial = gtsam.InitializePose3.computeOrientationsChordal(pose3Graph)
    
        # comparison is up to M_PI, that's why we add some multiples of 2*M_PI
        self.gtsamAssertEquals(initial.atRot3(x0), self.R0, 1e-6)
        self.gtsamAssertEquals(initial.atRot3(x1), self.R1, 1e-6)
        self.gtsamAssertEquals(initial.atRot3(x2), self.R2, 1e-6)
        self.gtsamAssertEquals(initial.atRot3(x3), self.R3, 1e-6)

    def test_initializePoses(self):
        g2oFile = gtsam.findExampleDataFile("pose3example-grid")
        is3D = True
        inputGraph, expectedValues = gtsam.readG2o(g2oFile, is3D)
        priorModel = gtsam.noiseModel.Unit.Create(6)
        inputGraph.add(gtsam.PriorFactorPose3(0, Pose3(), priorModel))

        initial = gtsam.InitializePose3.initialize(inputGraph)
        # TODO(frank): very loose !!
        self.gtsamAssertEquals(initial, expectedValues, 0.1)


if __name__ == "__main__":
    unittest.main()
