"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

visual_isam unit tests.
Author: Frank Dellaert & Pablo Alcantarilla
"""

import unittest

import gtsam
import numpy as np
from gtsam import (EssentialMatrix, EssentialMatrixConstraint, Point3, Pose3,
                   Rot3, Unit3, symbol)
from gtsam.utils.test_case import GtsamTestCase


class TestVisualISAMExample(GtsamTestCase):
    def test_VisualISAMExample(self):

        # Create a factor
        poseKey1 = symbol('x', 1)
        poseKey2 = symbol('x', 2)
        trueRotation = Rot3.RzRyRx(0.15, 0.15, -0.20)
        trueTranslation = Point3(+0.5, -1.0, +1.0)
        trueDirection = Unit3(trueTranslation)
        E = EssentialMatrix(trueRotation, trueDirection)
        model = gtsam.noiseModel.Isotropic.Sigma(5, 0.25)
        factor = EssentialMatrixConstraint(poseKey1, poseKey2, E, model)

        #  Create a linearization point at the zero-error point
        pose1 = Pose3(Rot3.RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0))
        pose2 = Pose3(
            Rot3.RzRyRx(0.179693265735950, 0.002945368776519,
                        0.102274823253840),
            Point3(-3.37493895, 6.14660244, -8.93650986))

        expected = np.zeros((5, 1))
        actual = factor.evaluateError(pose1, pose2)
        self.gtsamAssertEquals(actual, expected, 1e-8)


if __name__ == "__main__":
    unittest.main()
