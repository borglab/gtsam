import unittest
from gtsam import *
from math import *
import numpy as np
from gtsam_utils import Vector, Matrix


class TestStereoVOExample(unittest.TestCase):

    def test_StereoVOExample(self):
        ## Assumptions
        #  - For simplicity this example is in the camera's coordinate frame
        #  - X: right, Y: down, Z: forward
        #  - Pose x1 is at the origin, Pose 2 is 1 meter forward (along Z-axis)
        #  - x1 is fixed with a constraint, x2 is initialized with noisy values
        #  - No noise on measurements

        ## Create keys for variables
        x1 = symbol(ord('x'),1) 
        x2 = symbol(ord('x'),2) 
        l1 = symbol(ord('l'),1) 
        l2 = symbol(ord('l'),2) 
        l3 = symbol(ord('l'),3)

        ## Create graph container and add factors to it
        graph = NonlinearFactorGraph()

        ## add a constraint on the starting pose
        first_pose = Pose3()
        graph.add(NonlinearEqualityPose3(x1, first_pose))

        ## Create realistic calibration and measurement noise model
        # format: fx fy skew cx cy baseline
        K = Cal3_S2Stereo(1000, 1000, 0, 320, 240, 0.2)
        stereo_model = noiseModel_Diagonal.Sigmas(Vector([1.0, 1.0, 1.0]))

        ## Add measurements
        # pose 1
        graph.add(GenericStereoFactor3D(StereoPoint2(520, 480, 440), stereo_model, x1, l1, K))
        graph.add(GenericStereoFactor3D(StereoPoint2(120,  80, 440), stereo_model, x1, l2, K))
        graph.add(GenericStereoFactor3D(StereoPoint2(320, 280, 140), stereo_model, x1, l3, K))

        #pose 2
        graph.add(GenericStereoFactor3D(StereoPoint2(570, 520, 490), stereo_model, x2, l1, K))
        graph.add(GenericStereoFactor3D(StereoPoint2( 70,  20, 490), stereo_model, x2, l2, K))
        graph.add(GenericStereoFactor3D(StereoPoint2(320, 270, 115), stereo_model, x2, l3, K))

        ## Create initial estimate for camera poses and landmarks
        initialEstimate = Values()
        initialEstimate.insertPose3(x1, first_pose)
        # noisy estimate for pose 2
        initialEstimate.insertPose3(x2, Pose3(Rot3(), Point3(0.1,-.1,1.1)))
        expected_l1 = Point3( 1,  1, 5)
        initialEstimate.insertPoint3(l1, expected_l1)
        initialEstimate.insertPoint3(l2, Point3(-1,  1, 5))
        initialEstimate.insertPoint3(l3, Point3( 0,-.5, 5))

        ## optimize
        optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate)
        result = optimizer.optimize()

        ## check equality for the first pose and point
        pose_x1 = result.atPose3(x1)
        self.assertTrue(pose_x1.equals(first_pose,1e-4))

        point_l1 = result.atPoint3(l1)
        self.assertTrue(point_l1.equals(expected_l1,1e-4))

if __name__ == "__main__":
    unittest.main()
