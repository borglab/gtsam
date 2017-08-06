import math
import numpy as np
import unittest

from gtsam import Pose2, Point3, Rot3, Pose3, Cal3_S2, SimpleCamera

K = Cal3_S2(625, 625, 0, 0, 0)

class TestSimpleCamera(unittest.TestCase):

    def test_constructor(self):
        pose1 = Pose3(Rot3(np.diag([1, -1, -1])), Point3(0, 0, 0.5))
        camera = SimpleCamera(pose1, K)
        self.assertTrue(camera.calibration().equals(K, 1e-9))
        self.assertTrue(camera.pose().equals(pose1, 1e-9))

    def test_level2(self):
        # Create a level camera, looking in Y-direction
        pose2 = Pose2(0.4,0.3,math.pi/2.0)
        camera = SimpleCamera.Level(K, pose2, 0.1)

        # expected
        x = Point3(1,0,0)
        y = Point3(0,0,-1)
        z = Point3(0,1,0)
        wRc = Rot3(x,y,z)
        expected = Pose3(wRc,Point3(0.4,0.3,0.1))
        self.assertTrue(camera.pose().equals(expected, 1e-9))


if __name__ == "__main__":
    unittest.main()
