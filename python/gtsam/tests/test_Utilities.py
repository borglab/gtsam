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

    def test_perturbPoint2(self):
        """Test perturbPoint2."""
        values = gtsam.Values()
        values.insert(0, gtsam.Pose3())
        values.insert(1, gtsam.Point2(1, 1))
        gtsam.utilities.perturbPoint2(values, 1.0)
        self.assertTrue(
            not np.allclose(values.atPoint2(1), gtsam.Point2(1, 1)))

    def test_perturbPose2(self):
        """Test perturbPose2."""
        values = gtsam.Values()
        values.insert(0, gtsam.Pose2())
        values.insert(1, gtsam.Point2(1, 1))
        gtsam.utilities.perturbPose2(values, 1, 1)
        self.assertTrue(values.atPose2(0) != gtsam.Pose2())

    def test_perturbPoint3(self):
        """Test perturbPoint3."""
        values = gtsam.Values()
        point3 = gtsam.Point3(0, 0, 0)
        values.insert(0, gtsam.Pose2())
        values.insert(1, point3)
        gtsam.utilities.perturbPoint3(values, 1)
        self.assertTrue(not np.allclose(values.atPoint3(1), point3))

    def test_insertBackprojections(self):
        """Test insertBackprojections."""
        values = gtsam.Values()
        cam = gtsam.PinholeCameraCal3_S2()
        gtsam.utilities.insertBackprojections(
            values, cam, [0, 1, 2], np.asarray([[20, 30, 40], [20, 30, 40]]),
            10)
        np.testing.assert_allclose(values.atPoint3(0),
                                   gtsam.Point3(200, 200, 10))

    def test_insertProjectionFactors(self):
        """Test insertProjectionFactors."""
        graph = gtsam.NonlinearFactorGraph()
        gtsam.utilities.insertProjectionFactors(
            graph, 0, [0, 1], np.asarray([[20, 30], [20, 30]]),
            gtsam.noiseModel.Isotropic.Sigma(2, 0.1), gtsam.Cal3_S2())
        self.assertEqual(graph.size(), 2)

        graph = gtsam.NonlinearFactorGraph()
        gtsam.utilities.insertProjectionFactors(
            graph, 0, [0, 1], np.asarray([[20, 30], [20, 30]]),
            gtsam.noiseModel.Isotropic.Sigma(2, 0.1), gtsam.Cal3_S2(),
            gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(1, 0, 0)))
        self.assertEqual(graph.size(), 2)

    def test_reprojectionErrors(self):
        """Test reprojectionErrors."""
        pixels = np.asarray([[20, 30], [20, 30]])
        I = [1, 2]
        K = gtsam.Cal3_S2()
        graph = gtsam.NonlinearFactorGraph()
        gtsam.utilities.insertProjectionFactors(
            graph, 0, I, pixels, gtsam.noiseModel.Isotropic.Sigma(2, 0.1), K)
        values = gtsam.Values()
        values.insert(0, gtsam.Pose3())
        cam = gtsam.PinholeCameraCal3_S2(gtsam.Pose3(), K)
        gtsam.utilities.insertBackprojections(values, cam, I, pixels, 10)
        errors = gtsam.utilities.reprojectionErrors(graph, values)
        np.testing.assert_allclose(errors, np.zeros((2, 2)))

    def test_localToWorld(self):
        """Test localToWorld."""
        local = gtsam.Values()
        local.insert(0, gtsam.Point2(10, 10))
        local.insert(1, gtsam.Pose2(6, 11, 0.0))
        base = gtsam.Pose2(1, 0, 0)
        world = gtsam.utilities.localToWorld(local, base)

        expected_point2 = gtsam.Point2(11, 10)
        expected_pose2 = gtsam.Pose2(7, 11, 0)
        np.testing.assert_allclose(world.atPoint2(0), expected_point2)
        np.testing.assert_allclose(
            world.atPose2(1).matrix(), expected_pose2.matrix())

        user_keys = [1]
        world = gtsam.utilities.localToWorld(local, base, user_keys)
        np.testing.assert_allclose(
            world.atPose2(1).matrix(), expected_pose2.matrix())

        # Raise error since 0 is not in user_keys
        self.assertRaises(RuntimeError, world.atPoint2, 0)


if __name__ == "__main__":
    unittest.main()
