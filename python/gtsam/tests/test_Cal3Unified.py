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

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestCal3Unified(GtsamTestCase):

    def test_Cal3Unified(self):
        K = gtsam.Cal3Unified()
        self.assertEqual(K.fx(), 1.)
        self.assertEqual(K.fx(), 1.)

    def test_distortion(self):
        "Stereographic fisheye model of focal length f, defined as r/f = 2*tan(theta/2)"
        fx, fy, s, u0, v0 = 2, 2, 0, 0, 0
        k1, k2, p1, p2 = 0, 0, 0, 0
        xi = 1
        stereographic = gtsam.Cal3Unified(fx, fy, s, u0, v0, k1, k2, p1, p2, xi)
        x, y, z = 1, 0, 1
        u, v = stereographic.uncalibrate([x, y])
        r = np.linalg.norm([x, y, z])
        # Note: 2*tan(atan2(x, z)/2) = 2/(1+sqrt(x^2+z^2))
        self.assertAlmostEqual(2*np.tan(np.arctan2(x, z)/2), 2/(1+r))
        self.assertAlmostEqual(u, 2/(1+r))
        x2, y2 = stereographic.calibrate([u, v])
        self.assertAlmostEqual(x2, x)

    def test_pinhole(self):
        "Spatial stereographic camera projection"
        x, y, z = 1.0, 0.0, 1.0
        r = np.linalg.norm([x, y, z])
        u, v = 2/(1+r), 0.0
        objPoint = np.array([x, y, z])
        imgPoint = np.array([u, v])
        fx, fy, s, u0, v0 = 2, 2, 0, 0, 0
        k1, k2, p1, p2 = 0, 0, 0, 0
        xi = 1
        stereographic = gtsam.Cal3Unified(fx, fy, s, u0, v0, k1, k2, p1, p2, xi)
        pose = gtsam.Pose3()
        camera = gtsam.PinholeCameraCal3Unified(pose, stereographic)
        pt1 = camera.Project(objPoint)
        self.gtsamAssertEquals(pt1, np.array([x/z, y/z]))
        pt2 = camera.project(objPoint)
        self.gtsamAssertEquals(pt2, np.array([u, v]))
        obj1 = camera.backproject([u, v], z)
        self.gtsamAssertEquals(obj1, np.array([x, y, z]))
        r1 = camera.range(np.array([x, y, z]))
        self.assertEqual(r1, r)

    def test_generic_factor(self):
        "Evaluate residual using pose and point as state variables"
        fx, fy, s, u0, v0 = 2, 2, 0, 0, 0
        k1, k2, p1, p2 = 0, 0, 0, 0
        xi = 1
        objPoint = np.array([1, 0, 1])
        r = np.linalg.norm(objPoint)
        imgPoint = np.array([2/(1+r), 0])
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = imgPoint
        noiseModel = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        poseKey = gtsam.symbol_shorthand.P(0)
        pointKey = gtsam.symbol_shorthand.L(0)
        k = gtsam.Cal3Unified(fx, fy, s, u0, v0, k1, k2, p1, p2, xi)
        state.insert_pose3(poseKey, gtsam.Pose3())
        state.insert_point3(pointKey, gtsam.Point3(objPoint))
        factor = gtsam.GenericProjectionFactorCal3Unified(measured, noiseModel, poseKey, pointKey, k)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_sfm_factor2(self):
        "Evaluate residual with camera, pose and point as state variables"
        fx, fy, s, u0, v0 = 2, 2, 0, 0, 0
        k1, k2, p1, p2 = 0, 0, 0, 0
        xi = 1
        objPoint = np.array([1, 0, 1])
        r = np.linalg.norm(objPoint)
        imgPoint = np.array([2/(1+r), 0])
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = imgPoint
        noiseModel = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        cameraKey = gtsam.symbol_shorthand.K(0)
        poseKey = gtsam.symbol_shorthand.P(0)
        landmarkKey = gtsam.symbol_shorthand.L(0)
        k = gtsam.Cal3Unified(fx, fy, s, u0, v0, k1, k2, p1, p2, xi)
        state.insert_cal3unified(cameraKey, k)
        state.insert_pose3(poseKey, gtsam.Pose3())
        state.insert_point3(landmarkKey, gtsam.Point3(objPoint))
        factor = gtsam.GeneralSFMFactor2Cal3Unified(measured, noiseModel, poseKey, landmarkKey, cameraKey)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

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
