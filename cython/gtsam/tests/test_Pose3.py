import math
import unittest

from gtsam import Point3, Rot3, Pose3


class TestPose3(unittest.TestCase):

    def test__between(self):
        T2 = Pose3(Rot3.Rodrigues(0.3,0.2,0.1),Point3(3.5,-8.2,4.2))
        T3 = Pose3(Rot3.Rodrigues(-90, 0, 0), Point3(1, 2, 3))
        expected = T2.inverse().compose(T3)
        actual = T2.between(T3)
        self.assertTrue(actual.equals(expected, 1e-6))

    def test_transform_to(self):
        transform = Pose3(Rot3.Rodrigues(0,0,-1.570796), Point3(2,4, 0))
        actual = transform.transform_to(Point3(3,2,10))
        expected = Point3 (2,1,10)
        self.assertTrue(actual.equals(expected, 1e-6))

    def test_range(self):
        l1 = Point3(1, 0, 0)
        l2 = Point3(1, 1, 0)
        x1 = Pose3()

        xl1 = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(1, 0, 0))
        xl2 = Pose3(Rot3.Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0))

        # establish range is indeed zero
        self.assertEqual(1,x1.range(point=l1))

        # establish range is indeed sqrt2
        self.assertEqual(math.sqrt(2.0),x1.range(point=l2))

        # establish range is indeed zero
        self.assertEqual(1,x1.range(pose=xl1))
        
        # establish range is indeed sqrt2
        self.assertEqual(math.sqrt(2.0),x1.range(pose=xl2))


if __name__ == "__main__":
    unittest.main()
