"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Values unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
# pylint: disable=invalid-name, E1101, E0611
import unittest

import numpy as np

import gtsam
from gtsam import (Cal3_S2, Cal3Bundler, Cal3DS2, EssentialMatrix, Point2,
                   Point3, Pose2, Pose3, Rot2, Rot3, Unit3, Values, imuBias)
from gtsam.utils.test_case import GtsamTestCase


class TestValues(GtsamTestCase):

    def test_values(self):
        values = Values()
        E = EssentialMatrix(Rot3(), Unit3())
        tol = 1e-9

        values.insert(0, Point2(0, 0))
        values.insert(1, Point3(0, 0, 0))
        values.insert(2, Rot2())
        values.insert(3, Pose2())
        values.insert(4, Rot3())
        values.insert(5, Pose3())
        values.insert(6, Cal3_S2())
        values.insert(7, Cal3DS2())
        values.insert(8, Cal3Bundler())
        values.insert(9, E)
        values.insert(10, imuBias.ConstantBias())

        # Special cases for Vectors and Matrices
        # Note that gtsam's Eigen Vectors and Matrices requires double-precision
        # floating point numbers in column-major (Fortran style) storage order,
        # whereas by default, numpy.array is in row-major order and the type is
        # in whatever the number input type is, e.g. np.array([1,2,3])
        # will have 'int' type.
        #
        # The wrapper will automatically fix the type and storage order for you,
        # but for performance reasons, it's recommended to specify the correct
        # type and storage order.
        # for vectors, the order is not important, but dtype still is
        vec = np.array([1., 2., 3.])
        values.insert(11, vec)
        mat = np.array([[1., 2.], [3., 4.]], order='F')
        values.insert(12, mat)
        # Test with dtype int and the default order='C'
        # This still works as the wrapper converts to the correct type and order for you
        # but is nornally not recommended!
        mat2 = np.array([[1, 2, ], [3, 5]])
        values.insert(13, mat2)

        self.gtsamAssertEquals(values.atPoint2(0), Point2(0,0), tol)
        self.gtsamAssertEquals(values.atPoint3(1), Point3(0,0,0), tol)
        self.gtsamAssertEquals(values.atRot2(2), Rot2(), tol)
        self.gtsamAssertEquals(values.atPose2(3), Pose2(), tol)
        self.gtsamAssertEquals(values.atRot3(4), Rot3(), tol)
        self.gtsamAssertEquals(values.atPose3(5), Pose3(), tol)
        self.gtsamAssertEquals(values.atCal3_S2(6), Cal3_S2(), tol)
        self.gtsamAssertEquals(values.atCal3DS2(7), Cal3DS2(), tol)
        self.gtsamAssertEquals(values.atCal3Bundler(8), Cal3Bundler(), tol)
        self.gtsamAssertEquals(values.atEssentialMatrix(9), E, tol)
        self.gtsamAssertEquals(values.atConstantBias(
            10), imuBias.ConstantBias(), tol)

        # special cases for Vector and Matrix:
        actualVector = values.atVector(11)
        np.testing.assert_allclose(vec, actualVector, tol)
        actualMatrix = values.atMatrix(12)
        np.testing.assert_allclose(mat, actualMatrix, tol)
        actualMatrix2 = values.atMatrix(13)
        np.testing.assert_allclose(mat2, actualMatrix2, tol)


if __name__ == "__main__":
    unittest.main()
