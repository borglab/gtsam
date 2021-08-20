"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Utilities unit tests.
Author: Varun Agrawal
"""
# pylint: disable=invalid-name, E1101, E0611
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestUtilites(GtsamTestCase):
    """Test various GTSAM utilities."""
    def test_createKeyList(self):
        """Test createKeyList."""
        I = [0, 1, 2]
        kl = gtsam.utilities.createKeyList(I)
        self.assertEqual(kl.size(), 3)

        kl = gtsam.utilities.createKeyList("s", I)
        self.assertEqual(kl.size(), 3)

    def test_createKeyVector(self):
        """Test createKeyVector."""
        I = [0, 1, 2]
        kl = gtsam.utilities.createKeyVector(I)
        self.assertEqual(len(kl), 3)

        kl = gtsam.utilities.createKeyVector("s", I)
        self.assertEqual(len(kl), 3)

    def test_createKeySet(self):
        """Test createKeySet."""
        I = [0, 1, 2]
        kl = gtsam.utilities.createKeySet(I)
        self.assertEqual(kl.size(), 3)

        kl = gtsam.utilities.createKeySet("s", I)
        self.assertEqual(kl.size(), 3)

    def test_extractPoint2(self):
        """Test extractPoint2."""
        initial = gtsam.Values()
        point2 = gtsam.Point2(0.0, 0.1)
        initial.insert(1, gtsam.Pose2(0.0, 0.1, 0.1))
        initial.insert(2, point2)
        np.testing.assert_equal(gtsam.utilities.extractPoint2(initial),
                                point2.reshape(-1, 2))

    def test_extractPoint3(self):
        """Test extractPoint3."""
        initial = gtsam.Values()
        point3 = gtsam.Point3(0.0, 0.1, 0.0)
        initial.insert(1, gtsam.Pose2(0.0, 0.1, 0.1))
        initial.insert(2, point3)
        np.testing.assert_equal(gtsam.utilities.extractPoint3(initial),
                                point3.reshape(-1, 3))

    def test_allPose2s(self):
        """Test allPose2s."""
        initial = gtsam.Values()
        initial.insert(0, gtsam.Pose2())
        initial.insert(1, gtsam.Pose2(1, 1, 1))
        initial.insert(2, gtsam.Point2(1, 1))
        initial.insert(3, gtsam.Point3(1, 2, 3))
        result = gtsam.utilities.allPose2s(initial)
        self.assertEqual(result.size(), 2)

    def test_extractPose2(self):
        """Test extractPose2."""
        initial = gtsam.Values()
        pose2 = np.asarray((0.0, 0.1, 0.1))

        initial.insert(1, gtsam.Pose2(*pose2))
        initial.insert(2, gtsam.Point3(0.0, 0.1, 0.0))
        np.testing.assert_allclose(gtsam.utilities.extractPose2(initial),
                                   pose2.reshape(-1, 3))

    def test_allPose3s(self):
        """Test allPose3s."""
        initial = gtsam.Values()
        initial.insert(0, gtsam.Pose3())
        initial.insert(2, gtsam.Point2(1, 1))
        initial.insert(1, gtsam.Pose3())
        initial.insert(3, gtsam.Point3(1, 2, 3))
        result = gtsam.utilities.allPose3s(initial)
        self.assertEqual(result.size(), 2)

    def test_extractPose3(self):
        """Test extractPose3."""
        initial = gtsam.Values()
        pose3 = np.asarray([1., 0., 0., 0., 1., 0., 0., 0., 1., 0., 0., 0.])
        initial.insert(1, gtsam.Pose2(0.0, 0.1, 0.1))
        initial.insert(2, gtsam.Pose3())
        np.testing.assert_allclose(gtsam.utilities.extractPose3(initial),
                                   pose3.reshape(-1, 12))


if __name__ == "__main__":
    unittest.main()
