"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SimpleCamera unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import math
import unittest

import numpy as np

import gtsam
from gtsam import Cal3_S2, Point3, Pose2, Pose3, Rot3, PinholeCameraCal3_S2 as SimpleCamera
from gtsam.utils.test_case import GtsamTestCase

K = Cal3_S2(625, 625, 0, 0, 0)


class TestSimpleCamera(GtsamTestCase):

    def test_constructor(self):
        pose1 = Pose3(Rot3(np.diag([1, -1, -1])), Point3(0, 0, 0.5))
        camera = SimpleCamera(pose1, K)
        self.gtsamAssertEquals(camera.calibration(), K, 1e-9)
        self.gtsamAssertEquals(camera.pose(), pose1, 1e-9)

    def test_level2(self):
        # Create a level camera, looking in Y-direction
        pose2 = Pose2(0.4, 0.3, math.pi/2.0)
        camera = SimpleCamera.Level(K, pose2, 0.1)

        # expected
        x = Point3(1, 0, 0)
        y = Point3(0, 0, -1)
        z = Point3(0, 1, 0)
        wRc = Rot3(x, y, z)
        expected = Pose3(wRc, Point3(0.4, 0.3, 0.1))
        self.gtsamAssertEquals(camera.pose(), expected, 1e-9)


if __name__ == "__main__":
    unittest.main()
