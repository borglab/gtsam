"""
GTSAM Copyright 2010-2025, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Wrapped constructors must reject wrong-shaped Eigen input.

Author: GTSAM contributors
"""

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase


class TestMatrixShapeChecks(GtsamTestCase):
    """Fixed-size Eigen parameters must be enforced at the pybind11 boundary.

    Declaring a wrapper parameter as the dynamic gtsam::Matrix / gtsam::Vector
    when the C++ signature takes a fixed-size type lets any shape through, and
    the dynamic-to-fixed Eigen conversion is undefined behaviour once NDEBUG is
    set. gtsam.Rot3(np.eye(2)) used to segfault, and gtsam.Pose3(np.eye(2))
    used to return a garbage transform.
    """

    def test_rot3_rejects_wrong_shape(self) -> None:
        for bad in (np.eye(2), np.eye(4), np.zeros((3, 4)), np.zeros((1, 3))):
            with self.subTest(shape=bad.shape), self.assertRaises(TypeError):
                gtsam.Rot3(bad)
        gtsam.Rot3(np.eye(3))  # still accepted

    def test_pose3_rejects_wrong_shape(self) -> None:
        for bad in (np.eye(2), np.eye(3), np.zeros((4, 3))):
            with self.subTest(shape=bad.shape), self.assertRaises(TypeError):
                gtsam.Pose3(bad)
        gtsam.Pose3(np.eye(4))

    def test_pose2_vector_constructor_is_gone(self) -> None:
        """Pose2(vector) was Expmap in disguise; use Pose2.Expmap instead."""
        with self.assertRaises(TypeError):
            gtsam.Pose2(np.array([1.0, 2.0, 3.0]))
        # The replacement is explicit about being the exponential map, and is
        # declared Vector3 so it rejects wrong lengths.
        pose = gtsam.Pose2.Expmap(np.array([1.0, 2.0, 3.0]))
        self.assertAlmostEqual(pose.theta(), 3.0)
        for bad in (np.zeros(2), np.zeros(4)):
            with self.subTest(size=bad.size), self.assertRaises(TypeError):
                gtsam.Pose2.Expmap(bad)
        # Componentwise construction is a different thing, and still works.
        p = gtsam.Pose2(1.0, 2.0, 3.0)
        self.assertAlmostEqual(p.x(), 1.0)
        self.assertAlmostEqual(p.y(), 2.0)

    def test_oriented_plane3_rejects_wrong_length(self) -> None:
        for bad in (np.zeros(2), np.zeros(3), np.zeros(5)):
            with self.subTest(size=bad.size), self.assertRaises(TypeError):
                gtsam.OrientedPlane3(bad)
        gtsam.OrientedPlane3(np.array([0.0, 0.0, 1.0, 1.0]))

    def test_cameras_reject_wrong_length(self) -> None:
        for ctor in (gtsam.CalibratedCamera, gtsam.SphericalCamera):
            for bad in (np.zeros(2), np.zeros(7)):
                with self.subTest(ctor=ctor.__name__, size=bad.size), \
                        self.assertRaises(TypeError):
                    ctor(bad)
            ctor(np.zeros(6))


if __name__ == "__main__":
    unittest.main()
