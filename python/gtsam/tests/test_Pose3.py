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

import gtsam
from gtsam import Point3, Pose3, Rot3, Point3Pairs
from gtsam.utils.test_case import GtsamTestCase


def numerical_derivative_pose(pose, method, delta=1e-5):
    jacobian = np.zeros((6, 6))
    for idx in range(6):
        xplus = np.zeros(6)
        xplus[idx] = delta
        xminus = np.zeros(6)
        xminus[idx] = -delta
        pose_plus = pose.retract(xplus).__getattribute__(method)()
        pose_minus = pose.retract(xminus).__getattribute__(method)()
        jacobian[:, idx] = pose_minus.localCoordinates(pose_plus) / (2 * delta)
    return jacobian


def numerical_derivative_2_poses(pose, other_pose, method, delta=1e-5, inputs=()):
    jacobian = np.zeros((6, 6))
    other_jacobian = np.zeros((6, 6))
    for idx in range(6):
        xplus = np.zeros(6)
        xplus[idx] = delta
        xminus = np.zeros(6)
        xminus[idx] = -delta

        pose_plus = pose.retract(xplus).__getattribute__(method)(*inputs, other_pose)
        pose_minus = pose.retract(xminus).__getattribute__(method)(*inputs, other_pose)
        jacobian[:, idx] = pose_minus.localCoordinates(pose_plus) / (2 * delta)

        other_pose_plus = pose.__getattribute__(method)(*inputs, other_pose.retract(xplus))
        other_pose_minus = pose.__getattribute__(method)(*inputs, other_pose.retract(xminus))
        other_jacobian[:, idx] = other_pose_minus.localCoordinates(other_pose_plus) / (2 * delta)
    return jacobian, other_jacobian


def numerical_derivative_pose_point(pose, point, method, delta=1e-5):
    jacobian = np.zeros((3, 6))
    point_jacobian = np.zeros((3, 3))
    for idx in range(6):
        xplus = np.zeros(6)
        xplus[idx] = delta
        xminus = np.zeros(6)
        xminus[idx] = -delta

        point_plus = pose.retract(xplus).__getattribute__(method)(point)
        point_minus = pose.retract(xminus).__getattribute__(method)(point)
        jacobian[:, idx] = (point_plus - point_minus) / (2 * delta)

        if idx < 3:
            xplus = np.zeros(3)
            xplus[idx] = delta
            xminus = np.zeros(3)
            xminus[idx] = -delta
            point_plus = pose.__getattribute__(method)(point + xplus)
            point_minus = pose.__getattribute__(method)(point + xminus)
            point_jacobian[:, idx] = (point_plus - point_minus) / (2 * delta)
    return jacobian, point_jacobian


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
        jacobian_numerical, jacobian_numerical_other = numerical_derivative_2_poses(T2, T3, 'between')
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
        jacobian_numerical = numerical_derivative_pose(pose, 'inverse')
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
        jacobian_numerical, jacobian_numerical_other = numerical_derivative_2_poses(pose0, pose1, 'slerp', inputs=[0.5])
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
        jacobian_numerical_pose, jacobian_numerical_point = numerical_derivative_pose_point(pose, point, 'transformTo')
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
        jacobian_numerical_pose, jacobian_numerical_point = numerical_derivative_pose_point(pose, point, 'transformFrom')
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

        st_pairs = Point3Pairs()
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