"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Pose3 unit tests.
Author: Frank Dellaert, Duy Nguyen Ta
"""
# pylint: disable=no-name-in-module
import math
import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase

import gtsam
from gtsam import Point3, Pose3, Rot3
from gtsam.utils.numerical_derivative import numericalDerivative11, numericalDerivative21, numericalDerivative22

class TestPose3(GtsamTestCase):
    """Test selected Pose3 methods."""

    def test_between(self):
        """Test between method."""
        T2 = Pose3(Rot3.Rodrigues(0.3, 0.2, 0.1), Point3(3.5, -8.2, 4.2))
        T3 = Pose3(Rot3.Rodrigues(-90, 0, 0), Point3(1, 2, 3))
        expected = T2.inverse().compose(T3)
        actual = T2.between(T3)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        #test jacobians
        jacobian = np.zeros((6, 6), order='F')
        jacobian_other = np.zeros((6, 6), order='F')
        T2.between(T3, jacobian, jacobian_other)
        jacobian_numerical = numericalDerivative21(Pose3.between, T2, T3)
        jacobian_numerical_other = numericalDerivative22(Pose3.between, T2, T3)
        self.gtsamAssertEquals(jacobian, jacobian_numerical)
        self.gtsamAssertEquals(jacobian_other, jacobian_numerical_other)

    def test_inverse(self):
        """Test between method."""
        pose = Pose3(Rot3.Rodrigues(0, 0, -math.pi/2), Point3(2, 4, 0))
        expected = Pose3(Rot3.Rodrigues(0, 0, math.pi/2), Point3(4, -2, 0))
        actual = pose.inverse()
        self.gtsamAssertEquals(actual, expected, 1e-6)

        #test jacobians
        jacobian = np.zeros((6, 6), order='F')
        pose.inverse(jacobian)
        jacobian_numerical = numericalDerivative11(Pose3.inverse, pose)
        self.gtsamAssertEquals(jacobian, jacobian_numerical)

    def test_slerp(self):
        """Test slerp method."""
        pose0 = gtsam.Pose3()
        pose1 = Pose3(Rot3.Rodrigues(0, 0, -math.pi/2), Point3(2, 4, 0))
        actual_alpha_0 = pose0.slerp(0, pose1)
        self.gtsamAssertEquals(actual_alpha_0, pose0)
        actual_alpha_1 = pose0.slerp(1, pose1)
        self.gtsamAssertEquals(actual_alpha_1, pose1)
        actual_alpha_half = pose0.slerp(0.5, pose1)
        expected_alpha_half = Pose3(Rot3.Rodrigues(0, 0, -math.pi/4), Point3(0.17157288, 2.41421356, 0))
        self.gtsamAssertEquals(actual_alpha_half, expected_alpha_half, tol=1e-6)

        # test jacobians
        jacobian = np.zeros((6, 6), order='F')
        jacobian_other = np.zeros((6, 6), order='F')
        pose0.slerp(0.5, pose1, jacobian, jacobian_other)
        jacobian_numerical = numericalDerivative11(lambda x: x.slerp(0.5, pose1), pose0)
        jacobian_numerical_other = numericalDerivative11(lambda x: pose0.slerp(0.5, x), pose1)
        self.gtsamAssertEquals(jacobian, jacobian_numerical)
        self.gtsamAssertEquals(jacobian_other, jacobian_numerical_other)

    def test_transformTo(self):
        """Test transformTo method."""
        pose = Pose3(Rot3.Rodrigues(0, 0, -math.pi/2), Point3(2, 4, 0))
        actual = pose.transformTo(Point3(3, 2, 10))
        expected = Point3(2, 1, 10)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        #test jacobians
        point = Point3(3, 2, 10)
        jacobian_pose = np.zeros((3, 6), order='F')
        jacobian_point = np.zeros((3, 3), order='F')
        pose.transformTo(point, jacobian_pose, jacobian_point)
        jacobian_numerical_pose = numericalDerivative21(Pose3.transformTo, pose, point)
        jacobian_numerical_point = numericalDerivative22(Pose3.transformTo, pose, point)
        self.gtsamAssertEquals(jacobian_pose, jacobian_numerical_pose)
        self.gtsamAssertEquals(jacobian_point, jacobian_numerical_point)

        # multi-point version
        points = np.stack([Point3(3, 2, 10), Point3(3, 2, 10)]).T
        actual_array = pose.transformTo(points)
        self.assertEqual(actual_array.shape, (3, 2))
        expected_array = np.stack([expected, expected]).T
        np.testing.assert_allclose(actual_array, expected_array, atol=1e-6)

    def test_transformFrom(self):
        """Test transformFrom method."""
        pose = Pose3(Rot3.Rodrigues(0, 0, -math.pi/2), Point3(2, 4, 0))
        actual = pose.transformFrom(Point3(2, 1, 10))
        expected = Point3(3, 2, 10)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        #test jacobians
        point = Point3(3, 2, 10)
        jacobian_pose = np.zeros((3, 6), order='F')
        jacobian_point = np.zeros((3, 3), order='F')
        pose.transformFrom(point, jacobian_pose, jacobian_point)
        jacobian_numerical_pose = numericalDerivative21(Pose3.transformFrom, pose, point)
        jacobian_numerical_point = numericalDerivative22(Pose3.transformFrom, pose, point)
        self.gtsamAssertEquals(jacobian_pose, jacobian_numerical_pose)
        self.gtsamAssertEquals(jacobian_point, jacobian_numerical_point)

        # multi-point version
        points = np.stack([Point3(2, 1, 10), Point3(2, 1, 10)]).T
        actual_array = pose.transformFrom(points)
        self.assertEqual(actual_array.shape, (3, 2))
        expected_array = np.stack([expected, expected]).T
        np.testing.assert_allclose(actual_array, expected_array, atol=1e-6)

    def test_range(self):
        """Test range method."""
        l1 = Point3(1, 0, 0)
        l2 = Point3(1, 1, 0)
        x1 = Pose3()

        xl1 = Pose3(Rot3.Ypr(0.0, 0.0, 0.0), Point3(1, 0, 0))
        xl2 = Pose3(Rot3.Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0))

        # establish range is indeed zero
        self.assertEqual(1, x1.range(point=l1))

        # establish range is indeed sqrt2
        self.assertEqual(math.sqrt(2.0), x1.range(point=l2))

        # establish range is indeed zero
        self.assertEqual(1, x1.range(pose=xl1))

        # establish range is indeed sqrt2
        self.assertEqual(math.sqrt(2.0), x1.range(pose=xl2))

    def test_adjoint(self):
        """Test adjoint methods."""
        T = Pose3()
        xi = np.array([1, 2, 3, 4, 5, 6])
        # test calling functions
        T.AdjointMap()
        T.Adjoint(xi)
        T.AdjointTranspose(xi)
        Pose3.adjointMap(xi)
        Pose3.adjoint(xi, xi)
        # test correctness of adjoint(x, y)
        expected = np.dot(Pose3.adjointMap_(xi), xi)
        actual = Pose3.adjoint_(xi, xi)
        np.testing.assert_array_equal(actual, expected)

    def test_serialization(self):
        """Test if serialization is working normally"""
        expected = Pose3(Rot3.Ypr(0.0, 1.0, 0.0), Point3(1, 1, 0))
        actual = Pose3()
        serialized = expected.serialize()
        actual.deserialize(serialized)
        self.gtsamAssertEquals(expected, actual, 1e-10)

    def test_align_squares(self):
        """Test if Align method can align 2 squares."""
        square = np.array([[0,0,0],[0,1,0],[1,1,0],[1,0,0]], float).T
        sTt = Pose3(Rot3.Rodrigues(0, 0, -math.pi), Point3(2, 4, 0))
        transformed = sTt.transformTo(square)

        st_pairs = []
        for j in range(4):
            st_pairs.append((square[:,j], transformed[:,j]))

        # Recover the transformation sTt
        estimated_sTt = Pose3.Align(st_pairs)
        self.gtsamAssertEquals(estimated_sTt, sTt, 1e-10)

        # Matrix version
        estimated_sTt = Pose3.Align(square, transformed)
        self.gtsamAssertEquals(estimated_sTt, sTt, 1e-10)


if __name__ == "__main__":
    unittest.main()
