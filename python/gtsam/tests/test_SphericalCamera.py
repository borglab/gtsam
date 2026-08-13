"""
GTSAM Copyright 2010-2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

SphericalCamera unit tests.
Author: auto-generated
"""
import unittest

import numpy as np

import gtsam
from gtsam import Point3, Pose3, Rot3, SphericalCamera, Unit3
from gtsam.utils.test_case import GtsamTestCase

# Camera pose: Rot3 with diagonal (1, -1, -1), translation (0, 0, 0.5)
pose = Pose3(Rot3(np.diag([1., -1., -1.])), Point3(0, 0, 0.5))
camera = SphericalCamera(pose)

# Test points
point1 = Point3(-0.08, -0.08, 0.0)
point2 = Point3(-0.08, 0.08, 0.0)
point3 = Point3(0.08, 0.08, 0.0)
point4 = Point3(0.08, -0.08, 0.0)

# Manually computed bearing vectors (from Matlab)
bearing1 = Unit3(np.array([-0.156054862928174, 0.156054862928174, 0.975342893301088]))
bearing2 = Unit3(np.array([-0.156054862928174, -0.156054862928174, 0.975342893301088]))
bearing3 = Unit3(np.array([0.156054862928174, -0.156054862928174, 0.975342893301088]))
bearing4 = Unit3(np.array([0.156054862928174, 0.156054862928174, 0.975342893301088]))

depth = 0.512640224719052


class TestSphericalCamera(GtsamTestCase):

    def test_constructor(self):
        """Test construction from Pose3 and pose() accessor."""
        self.gtsamAssertEquals(camera.pose(), pose)

    def test_project(self):
        """Test projection of 4 points to bearing vectors."""
        self.gtsamAssertEquals(camera.project(point1), bearing1)
        self.gtsamAssertEquals(camera.project(point2), bearing2)
        self.gtsamAssertEquals(camera.project(point3), bearing3)
        self.gtsamAssertEquals(camera.project(point4), bearing4)

    def test_backproject(self):
        """Test back-projection from bearing + depth to 3D point."""
        self.gtsamAssertEquals(camera.backproject(bearing1, depth), point1)
        self.gtsamAssertEquals(camera.backproject(bearing2, depth), point2)
        self.gtsamAssertEquals(camera.backproject(bearing3, depth), point3)
        self.gtsamAssertEquals(camera.backproject(bearing4, depth), point4)

    def test_reprojection_error(self):
        """Test that reprojection error is zero at ground truth."""
        result = camera.reprojectionError(point1, bearing1)
        np.testing.assert_allclose(result, np.zeros(2), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
