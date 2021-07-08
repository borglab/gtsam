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


class TestCal3Fisheye(GtsamTestCase):

    def test_Cal3Fisheye(self):
        K = gtsam.Cal3Fisheye()
        self.assertEqual(K.fx(), 1.)
        self.assertEqual(K.fy(), 1.)

    def test_distortion(self):
        "Equidistant fisheye model of focal length f, defined as r/f = tan(theta)"
        equidistant = gtsam.Cal3Fisheye()
        x, y, z = 1, 0, 1
        u, v = equidistant.uncalibrate([x, y])
        x2, y2 = equidistant.calibrate([u, v])
        self.assertAlmostEqual(u, np.arctan2(x, z))
        self.assertAlmostEqual(v, 0)
        self.assertAlmostEqual(x2, x)
        self.assertAlmostEqual(y2, 0)

    def test_pinhole(self):
        "Spatial equidistant camera projection"
        x, y, z = 1.0, 0.0, 1.0
        u, v = np.arctan2(x, z), 0.0
        camera = gtsam.PinholeCameraCal3Fisheye()

        pt1 = camera.Project([x, y, z])
        self.gtsamAssertEquals(pt1, np.array([x/z, y/z]))

        pt2 = camera.project([x, y, z])
        self.gtsamAssertEquals(pt2, np.array([u, v]))

        obj1 = camera.backproject([u, v], z)
        self.gtsamAssertEquals(obj1, np.array([x, y, z]))

        r1 = camera.range(np.array([x, y, z]))
        self.assertEqual(r1, np.linalg.norm([x, y, z]))

    def test_generic_factor(self):
        "Evaluate residual using pose and point as state variables"
        objPoint = np.array([1, 0, 1])
        imgPoint = np.array([np.arctan2(objPoint[0], objPoint[2]), 0])
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = imgPoint
        noiseModel = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        poseKey = gtsam.symbol_shorthand.P(0)
        pointKey = gtsam.symbol_shorthand.L(0)
        k = gtsam.Cal3Fisheye()
        state.insert_pose3(poseKey, gtsam.Pose3())
        state.insert_point3(pointKey, gtsam.Point3(objPoint))
        factor = gtsam.GenericProjectionFactorCal3Fisheye(measured, noiseModel, poseKey, pointKey, k)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_sfm_factor2(self):
        "Evaluate residual with camera, pose and point as state variables"
        objPoint = np.array([1, 0, 1])
        imgPoint = np.array([np.arctan2(objPoint[0], objPoint[2]), 0])
        graph = gtsam.NonlinearFactorGraph()
        state = gtsam.Values()
        measured = imgPoint
        noiseModel = gtsam.noiseModel.Isotropic.Sigma(2, 1)
        cameraKey = gtsam.symbol_shorthand.K(0)
        poseKey = gtsam.symbol_shorthand.P(0)
        landmarkKey = gtsam.symbol_shorthand.L(0)
        k = gtsam.Cal3Fisheye()
        state.insert_cal3fisheye(cameraKey, k)
        state.insert_pose3(poseKey, gtsam.Pose3())
        state.insert_point3(landmarkKey, gtsam.Point3(objPoint))
        factor = gtsam.GeneralSFMFactor2Cal3Fisheye(measured, noiseModel, poseKey, landmarkKey, cameraKey)
        graph.add(factor)
        score = graph.error(state)
        self.assertAlmostEqual(score, 0)

    def test_retract(self):
        expected = gtsam.Cal3Fisheye(100 + 2, 105 + 3, 0.0 + 4, 320 + 5, 240 + 6,
                                     1e-3 + 7, 2.0*1e-3 + 8, 3.0*1e-3 + 9, 4.0*1e-3 + 10)
        K = gtsam.Cal3Fisheye(100, 105, 0.0, 320, 240,
                              1e-3, 2.0*1e-3, 3.0*1e-3, 4.0*1e-3)
        d = np.array([2, 3, 4, 5, 6, 7, 8, 9, 10], order='F')
        actual = K.retract(d)
        self.gtsamAssertEquals(actual, expected)
        np.testing.assert_allclose(d, K.localCoordinates(actual))


if __name__ == "__main__":
    unittest.main()
