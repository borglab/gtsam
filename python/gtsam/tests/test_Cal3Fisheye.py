"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Cal3Fisheye unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
Refactored: Roderick Koehle
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.symbol_shorthand import K, L, P


def ulp(ftype=np.float64):
    """
    Unit in the last place of floating point datatypes
    """
    f = np.finfo(ftype)
    return f.tiny / ftype(1 << f.nmant)


class TestCal3Fisheye(GtsamTestCase):
    
    @classmethod
    def setUpClass(cls):
        """
        Equidistant fisheye projection
        
        An equidistant fisheye projection with focal length f is defined
        as the relation r/f = tan(theta), with r being the radius in the 
        image plane and theta the incident angle of the object point.
        """
        x, y, z = 1.0, 0.0, 1.0
        u, v = np.arctan2(x, z), 0.0
        cls.obj_point = np.array([x, y, z])
        cls.img_point = np.array([u, v])

        p1 = [-1.0, 0.0, -1.0]
        p2 = [ 1.0, 0.0, -1.0]
        q1 = gtsam.Rot3(1.0, 0.0, 0.0, 0.0)
        q2 = gtsam.Rot3(1.0, 0.0, 0.0, 0.0)
        pose1 = gtsam.Pose3(q1, p1)
        pose2 = gtsam.Pose3(q2, p2)
        camera1 = gtsam.PinholeCameraCal3Fisheye(pose1)
        camera2 = gtsam.PinholeCameraCal3Fisheye(pose2)
        cls.origin = np.array([0.0, 0.0, 0.0])
        cls.poses = gtsam.Pose3Vector([pose1, pose2])
        cls.cameras = gtsam.CameraSetCal3Fisheye([camera1, camera2])
        cls.measurements = gtsam.Point2Vector([k.project(cls.origin) for k in cls.cameras])
        
    def test_Cal3Fisheye(self):
        K = gtsam.Cal3Fisheye()
        self.assertEqual(K.fx(), 1.)
        self.assertEqual(K.fy(), 1.)

    def test_distortion(self):
        """Fisheye distortion and rectification"""
        equidistant = gtsam.Cal3Fisheye()
        perspective_pt = self.obj_point[0:2]/self.obj_point[2]
        distorted_pt = equidistant.uncalibrate(perspective_pt)
        rectified_pt = equidistant.calibrate(distorted_pt)
        self.gtsamAssertEquals(distorted_pt, self.img_point)
        self.gtsamAssertEquals(rectified_pt, perspective_pt)

    def test_pinhole(self):
        """Spatial equidistant camera projection"""
        camera = gtsam.PinholeCameraCal3Fisheye()
        pt1 = camera.Project(self.obj_point) # Perspective projection
        pt2 = camera.project(self.obj_point) # Equidistant projection
        x, y, z = self.obj_point
        obj1 = camera.backproject(self.img_point, z)
        r1 = camera.range(self.obj_point)
        r = np.linalg.norm(self.obj_point)
        self.gtsamAssertEquals(pt1, np.array([x/z, y/z]))
        self.gtsamAssertEquals(pt2, self.img_point)
        self.gtsamAssertEquals(obj1, self.obj_point)
        self.assertEqual(r1, r)

    def test_generic_factor(self):
        """Evaluate residual using pose and point as state variables"""
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = self.img_point
        noise_model = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        pose_key, point_key = P(0), L(0)
        k = gtsam.Cal3Fisheye()
        state.insert_pose3(pose_key, gtsam.Pose3())
        state.insert_point3(point_key, self.obj_point)
        factor = gtsam.GenericProjectionFactorCal3Fisheye(measured, noise_model, pose_key, point_key, k)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_sfm_factor2(self):
        """Evaluate residual with camera, pose and point as state variables"""
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = self.img_point
        noise_model = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        camera_key, pose_key, landmark_key = K(0), P(0), L(0)
        k = gtsam.Cal3Fisheye()
        state.insert_cal3fisheye(camera_key, k)
        state.insert_pose3(pose_key, gtsam.Pose3())
        state.insert_point3(landmark_key, gtsam.Point3(self.obj_point))
        factor = gtsam.GeneralSFMFactor2Cal3Fisheye(measured, noise_model, pose_key, landmark_key, camera_key)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_jacobian_on_axis(self):
        """Check of jacobian at optical axis"""
        obj_point_on_axis = np.array([0, 0, 1])
        img_point = np.array([0, 0])
        f, z, H = self.evaluate_jacobian(obj_point_on_axis, img_point)
        self.assertAlmostEqual(f, 0)
        self.gtsamAssertEquals(z, np.zeros(2))
        self.gtsamAssertEquals(H @ H.T, 3*np.eye(2))

    def test_jacobian_convergence(self):
        """Test stability of jacobian close to optical axis"""
        t = ulp(np.float64)
        obj_point_close_to_axis = np.array([t, 0, 1])
        img_point = np.array([np.sqrt(t), 0])
        f, z, H = self.evaluate_jacobian(obj_point_close_to_axis, img_point)
        self.assertAlmostEqual(f, 0)
        self.gtsamAssertEquals(z, np.zeros(2))
        self.gtsamAssertEquals(H @ H.T, 3*np.eye(2))

        # With a height of sqrt(ulp), this may cause an overflow
        t = ulp(np.float64)
        obj_point_close_to_axis = np.array([np.sqrt(t), 0, 1])
        img_point = np.array([np.sqrt(t), 0])
        f, z, H = self.evaluate_jacobian(obj_point_close_to_axis, img_point)
        self.assertAlmostEqual(f, 0)
        self.gtsamAssertEquals(z, np.zeros(2))
        self.gtsamAssertEquals(H @ H.T, 3*np.eye(2))

    def test_scaling_factor(self):
        """Check convergence of atan2(r, z)/r ~ 1/z for small r"""
        r = ulp(np.float64)
        s = np.arctan(r) / r
        self.assertEqual(s, 1.0)
        z = 1
        s = self.scaling_factor(r, z)
        self.assertEqual(s, 1.0/z)
        z = 2
        s = self.scaling_factor(r, z)
        self.assertEqual(s, 1.0/z)
        s = self.scaling_factor(2*r, z)
        self.assertEqual(s, 1.0/z)

    @staticmethod
    def scaling_factor(r, z):
        """Projection factor theta/r for equidistant fisheye lens model"""
        return np.arctan2(r, z) / r if r/z != 0 else 1.0/z

    @staticmethod
    def evaluate_jacobian(obj_point, img_point):
        """Evaluate jacobian at given object point"""
        pose = gtsam.Pose3()
        camera = gtsam.Cal3Fisheye()
        state = gtsam.Values()
        camera_key, pose_key, landmark_key = K(0), P(0), L(0)
        state.insert_point3(landmark_key, obj_point)
        state.insert_pose3(pose_key, pose)
        g = gtsam.NonlinearFactorGraph()
        noise_model = gtsam.noiseModel.Unit.Create(2)
        factor = gtsam.GenericProjectionFactorCal3Fisheye(img_point, noise_model, pose_key, landmark_key, camera)
        g.add(factor)
        f = g.error(state)
        gaussian_factor_graph = g.linearize(state)
        H, z = gaussian_factor_graph.jacobian()
        return f, z, H

    @unittest.skip("triangulatePoint3 currently seems to require perspective projections.")
    def test_triangulation_skipped(self):
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
        expected = gtsam.Cal3Fisheye(100 + 2, 105 + 3, 0.0 + 4, 320 + 5, 240 + 6,
                                     1e-3 + 7, 2.0*1e-3 + 8, 3.0*1e-3 + 9, 4.0*1e-3 + 10)
        k = gtsam.Cal3Fisheye(100, 105, 0.0, 320, 240,
                              1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3)
        d = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10], order='F')
        actual = k.retract(d)
        self.gtsamAssertEquals(actual, expected)
        np.testing.assert_allclose(d, k.localCoordinates(actual))


if __name__ == "__main__":
    unittest.main()
