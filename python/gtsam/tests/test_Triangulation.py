"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Test Triangulation
Author: Frank Dellaert & Fan Jiang (Python)
"""
import unittest

import numpy as np

import gtsam as g
from gtsam.utils.test_case import GtsamTestCase
from gtsam import Cal3_S2, Cal3Bundler, Rot3, Pose3, \
    PinholeCameraCal3_S2, PinholeCameraCal3Bundler, Point3, \
    Point2Vector, Pose3Vector, triangulatePoint3, \
    CameraSetCal3_S2, CameraSetCal3Bundler

class TestVisualISAMExample(GtsamTestCase):
    def setUp(self):
        # Set up two camera poses
        # Looking along X-axis, 1 meter above ground plane (x-y)
        upright = Rot3.Ypr(-np.pi / 2, 0., -np.pi / 2)
        self.pose1 = Pose3(upright, Point3(0, 0, 1))

        # create second camera 1 meter to the right of first camera
        self.pose2 = self.pose1.compose(Pose3(Rot3(), Point3(1, 0, 0)))

        # landmark ~5 meters infront of camera
        self.landmark = Point3(5, 0.5, 1.2)

    
    def test_TriangulationExample(self):
        # Some common constants
        sharedCal = Cal3_S2(1500, 1200, 0, 640, 480)        
        camera1 = PinholeCameraCal3_S2(self.pose1, sharedCal)
        camera2 = PinholeCameraCal3_S2(self.pose2, sharedCal)

        # 1. Project two landmarks into two cameras and triangulate
        z1 = camera1.project(self.landmark)
        z2 = camera2.project(self.landmark)

        # twoPoses
        poses = Pose3Vector()
        measurements = Point2Vector()

        poses.append(self.pose1)
        poses.append(self.pose2)
        measurements.append(z1)
        measurements.append(z2)

        optimize = True
        rank_tol = 1e-9

        triangulated_landmark = triangulatePoint3(poses,sharedCal, measurements, rank_tol, optimize)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark,1e-9)

        # 2. Add some noise and try again: result should be ~ (4.995, 0.499167, 1.19814)
        measurements = Point2Vector()
        measurements.append(z1 - np.array([0.1, 0.5]))
        measurements.append(z2 - np.array([-0.2, 0.3]))

        triangulated_landmark = triangulatePoint3(poses,sharedCal, measurements, rank_tol, optimize)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark,1e-2)

    def test_distinct_Ks(self):
        # two cameras
        K1 = Cal3_S2(1500, 1200, 0, 640, 480)
        camera1 = PinholeCameraCal3_S2(self.pose1, K1)

        K2 = Cal3_S2(1600, 1300, 0, 650, 440)
        camera2 = PinholeCameraCal3_S2(self.pose2, K2)

        cameras = CameraSetCal3_S2()
        cameras.append(camera1)
        cameras.append(camera2)

        # Project two landmarks into two cameras and triangulate
        z1 = camera1.project(self.landmark)
        z2 = camera2.project(self.landmark)
        
        measurements = Point2Vector()
        measurements.append(z1)
        measurements.append(z2)

        optimize = True
        rank_tol = 1e-9

        triangulated_landmark = triangulatePoint3(cameras, measurements, rank_tol, optimize)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-9)

    def test_distinct_Ks_Bundler(self):
        # two cameras
        K1 = Cal3Bundler(1500, 0, 0, 640, 480)
        camera1 = PinholeCameraCal3Bundler(self.pose1, K1)

        K2 = Cal3Bundler(1600, 0, 0, 650, 440)
        camera2 = PinholeCameraCal3Bundler(self.pose2, K2)

        cameras = CameraSetCal3Bundler()
        cameras.append(camera1)
        cameras.append(camera2)

        # Project two landmarks into two cameras and triangulate
        z1 = camera1.project(self.landmark)
        z2 = camera2.project(self.landmark)

        measurements = Point2Vector()
        measurements.append(z1)
        measurements.append(z2)

        optimize = True
        rank_tol = 1e-9

        triangulated_landmark = triangulatePoint3(cameras, measurements, rank_tol, optimize)
        self.gtsamAssertEquals(self.landmark, triangulated_landmark, 1e-9) 

if __name__ == "__main__":
    unittest.main()
