"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Stereo VO unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
from gtsam import symbol
from gtsam.utils.test_case import GtsamTestCase


class TestStereoVOExample(GtsamTestCase):

    def test_StereoVOExample(self):
        ## Assumptions
        #  - For simplicity this example is in the camera's coordinate frame
        #  - X: right, Y: down, Z: forward
        #  - Pose x1 is at the origin, Pose 2 is 1 meter forward (along Z-axis)
        #  - x1 is fixed with a constraint, x2 is initialized with noisy values
        #  - No noise on measurements

        ## Create keys for variables
        x1 = symbol('x',1) 
        x2 = symbol('x',2) 
        l1 = symbol('l',1) 
        l2 = symbol('l',2) 
        l3 = symbol('l',3)

        ## Create graph container and add factors to it
        graph = gtsam.NonlinearFactorGraph()

        ## add a constraint on the starting pose
        first_pose = gtsam.Pose3()
        graph.add(gtsam.NonlinearEqualityPose3(x1, first_pose))

        ## Create realistic calibration and measurement noise model
        # format: fx fy skew cx cy baseline
        K = gtsam.Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2)
        stereo_model = gtsam.noiseModel.Diagonal.Sigmas(np.array([1.0, 1.0, 1.0]))

        ## Add measurements
        # pose 1
        graph.add(gtsam.GenericStereoFactor3D(gtsam.StereoPoint2(520, 480, 440), stereo_model, x1, l1, K))
        graph.add(gtsam.GenericStereoFactor3D(gtsam.StereoPoint2(120,  80, 440), stereo_model, x1, l2, K))
        graph.add(gtsam.GenericStereoFactor3D(gtsam.StereoPoint2(320, 280, 140), stereo_model, x1, l3, K))

        #pose 2
        graph.add(gtsam.GenericStereoFactor3D(gtsam.StereoPoint2(570, 520, 490), stereo_model, x2, l1, K))
        graph.add(gtsam.GenericStereoFactor3D(gtsam.StereoPoint2( 70,  20, 490), stereo_model, x2, l2, K))
        graph.add(gtsam.GenericStereoFactor3D(gtsam.StereoPoint2(320, 270, 115), stereo_model, x2, l3, K))

        ## Create initial estimate for camera poses and landmarks
        initialEstimate = gtsam.Values()
        initialEstimate.insert(x1, first_pose)
        # noisy estimate for pose 2
        initialEstimate.insert(x2, gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0.1,-.1,1.1)))
        expected_l1 = gtsam.Point3( 1,  1, 5)
        initialEstimate.insert(l1, expected_l1)
        initialEstimate.insert(l2, gtsam.Point3(-1,  1, 5))
        initialEstimate.insert(l3, gtsam.Point3( 0,-.5, 5))

        ## optimize
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initialEstimate)
        result = optimizer.optimize()

        ## check equality for the first pose and point
        pose_x1 = result.atPose3(x1)
        self.gtsamAssertEquals(pose_x1, first_pose,1e-4)

        point_l1 = result.atPoint3(l1)
        self.gtsamAssertEquals(point_l1, expected_l1,1e-4)

if __name__ == "__main__":
    unittest.main()
