"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Cal3Unified unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np
from gtsam.symbol_shorthand import K, L, P
from gtsam.utils.test_case import GtsamTestCase

import gtsam


class TestCal3Unified(GtsamTestCase):

    @classmethod
    def setUpClass(cls):
        """
        Stereographic fisheye projection
        
        An equidistant fisheye projection with focal length f is defined
        as the relation r/f = 2*tan(theta/2), with r being the radius in the 
        image plane and theta the incident angle of the object point.
        """
        x, y, z = 1.0, 0.0, 1.0
        r = np.linalg.norm([x, y, z])
        u, v = 2*x/(z+r), 0.0
        cls.obj_point = np.array([x, y, z])
        cls.img_point = np.array([u, v])

        fx, fy, s, u0, v0 = 2, 2, 0, 0, 0
        k1, k2, p1, p2 = 0, 0, 0, 0
        xi = 1
        cls.stereographic = gtsam.Cal3Unified(fx, fy, s, u0, v0, k1, k2, p1, p2, xi)

        p1 = [-1.0, 0.0, -1.0]
        p2 = [ 1.0, 0.0, -1.0]
        q1 = gtsam.Rot3(1.0, 0.0, 0.0, 0.0)
        q2 = gtsam.Rot3(1.0, 0.0, 0.0, 0.0)
        pose1 = gtsam.Pose3(q1, p1)
        pose2 = gtsam.Pose3(q2, p2)
        camera1 = gtsam.PinholeCameraCal3Unified(pose1, cls.stereographic)
        camera2 = gtsam.PinholeCameraCal3Unified(pose2, cls.stereographic)
        cls.origin = np.array([0.0, 0.0, 0.0])
        cls.poses = gtsam.Pose3Vector([pose1, pose2])
        cls.cameras = gtsam.CameraSetCal3Unified([camera1, camera2])
        cls.measurements = gtsam.Point2Vector(
            [k.project(cls.origin) for k in cls.cameras])

    def test_Cal3Unified(self):
        K = gtsam.Cal3Unified()
        self.assertEqual(K.fx(), 1.)
        self.assertEqual(K.fx(), 1.)

    def test_distortion(self):
        """Stereographic fisheye model of focal length f, defined as r/f = 2*tan(theta/2)"""
        x, y, z = self.obj_point
        r = np.linalg.norm([x, y, z])
        # Note: 2*tan(atan2(x, z)/2) = 2*x/(z+sqrt(x^2+z^2))
        self.assertAlmostEqual(2*np.tan(np.arctan2(x, z)/2), 2*x/(z+r))
        perspective_pt = self.obj_point[0:2]/self.obj_point[2]
        distorted_pt = self.stereographic.uncalibrate(perspective_pt)
        rectified_pt = self.stereographic.calibrate(distorted_pt)
        self.gtsamAssertEquals(distorted_pt, self.img_point)
        self.gtsamAssertEquals(rectified_pt, perspective_pt)

    def test_pinhole(self):
        """Spatial stereographic camera projection"""
        x, y, z = self.obj_point
        u, v = self.img_point
        r = np.linalg.norm(self.obj_point)
        pose = gtsam.Pose3()
        camera = gtsam.PinholeCameraCal3Unified(pose, self.stereographic)
        pt1 = camera.Project(self.obj_point)
        self.gtsamAssertEquals(pt1, np.array([x/z, y/z]))
        pt2 = camera.project(self.obj_point)
        self.gtsamAssertEquals(pt2, self.img_point)
        obj1 = camera.backproject(self.img_point, z)
        self.gtsamAssertEquals(obj1, self.obj_point)
        r1 = camera.range(self.obj_point)
        self.assertEqual(r1, r)

    def test_generic_factor(self):
        """Evaluate residual using pose and point as state variables"""
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = self.img_point
        noise_model = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        pose_key, point_key = P(0), L(0)
        k = self.stereographic
        state.insert_pose3(pose_key, gtsam.Pose3())
        state.insert_point3(point_key, self.obj_point)
        factor = gtsam.GenericProjectionFactorCal3Unified(measured, noise_model, pose_key, point_key, k)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_sfm_factor2(self):
        """Evaluate residual with camera, pose and point as state variables"""
        r = np.linalg.norm(self.obj_point)
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = self.img_point
        noise_model = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        camera_key, pose_key, landmark_key = K(0), P(0), L(0)
        k = self.stereographic
        state.insert_cal3unified(camera_key, k)
        state.insert_pose3(pose_key, gtsam.Pose3())
        state.insert_point3(landmark_key, self.obj_point)
        factor = gtsam.GeneralSFMFactor2Cal3Unified(measured, noise_model, pose_key, landmark_key, camera_key)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_jacobian(self):
        """Evaluate jacobian at optical axis"""
        obj_point_on_axis = np.array([0, 0, 1])
        img_point = np.array([0.0, 0.0])
        pose = gtsam.Pose3()
        camera = gtsam.Cal3Unified()
        state = gtsam.Values()
        camera_key, pose_key, landmark_key = K(0), P(0), L(0)
        state.insert_cal3unified(camera_key, camera)
        state.insert_point3(landmark_key, obj_point_on_axis)
        state.insert_pose3(pose_key, pose)
        g = gtsam.NonlinearFactorGraph()
        noise_model = gtsam.noiseModel.Unit.Create(2)
        factor = gtsam.GeneralSFMFactor2Cal3Unified(img_point, noise_model, pose_key, landmark_key, camera_key)
        g.add(factor)
        f = g.error(state)
        gaussian_factor_graph = g.linearize(state)
        H, z = gaussian_factor_graph.jacobian()
        self.assertAlmostEqual(f, 0)
        self.gtsamAssertEquals(z, np.zeros(2))
        self.gtsamAssertEquals(H @ H.T, 4*np.eye(2))

        Dcal = np.zeros((2, 10), order='F')
        Dp = np.zeros((2, 2), order='F')
        camera.calibrate(img_point, Dcal, Dp)

        self.gtsamAssertEquals(Dcal, np.array(
            [[ 0.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.,  0.],
            [ 0.,  0.,  0.,  0., -1.,  0.,  0.,  0.,  0.,  0.]]))
        self.gtsamAssertEquals(Dp, np.array(
            [[ 1., -0.],
            [-0.,  1.]]))

    @unittest.skip("triangulatePoint3 currently seems to require perspective projections.")
    def test_triangulation(self):
        """Estimate spatial point from image measurements"""
        triangulated = gtsam.triangulatePoint3(self.cameras, self.measurements, rank_tol=1e-9, optimize=True)
        self.gtsamAssertEquals(triangulated, self.origin)

    def test_triangulation_rectify(self):
        """Estimate spatial point from image measurements using rectification"""
        rectified = gtsam.Point2Vector([k.calibration().calibrate(pt) for k, pt in zip(self.cameras, self.measurements)])
        shared_cal = gtsam.Cal3_S2()
        triangulated = gtsam.triangulatePoint3(self.poses, shared_cal, rectified, rank_tol=1e-9, optimize=False)
        self.gtsamAssertEquals(triangulated, self.origin)

    def test_retract(self):
        expected = gtsam.Cal3Unified(100 + 2, 105 + 3, 0.0 + 4, 320 + 5, 240 + 6,
                                     1e-3 + 7, 2.0*1e-3 + 8, 3.0*1e-3 + 9, 4.0*1e-3 + 10, 0.1 + 1)
        K = gtsam.Cal3Unified(100, 105, 0.0, 320, 240,
                              1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3, 0.1)
        d = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10, 1], order='F')
        actual = K.retract(d)
        self.gtsamAssertEquals(actual, expected)
        np.testing.assert_allclose(d, K.localCoordinates(actual))


if __name__ == "__main__":
    unittest.main()
