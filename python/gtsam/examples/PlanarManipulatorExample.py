"""
GTSAM Copyright 2010-2018, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
Authors: Frank Dellaert, et al. (see THANKS for the full author list)

See LICENSE for the license information

Kinematics of three-link manipulator with GTSAM poses and product of exponential maps.
Author: Frank Dellaert
"""
# pylint: disable=invalid-name, E1101

from __future__ import print_function

import math
import unittest
from functools import reduce

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # pylint: disable=W0611

import gtsam
import gtsam.utils.plot as gtsam_plot
from gtsam import Pose2
from gtsam.utils.test_case import GtsamTestCase


def vector3(x, y, z):
    """Create 3D double numpy array."""
    return np.array([x, y, z], dtype=float)


def compose(*poses):
    """Compose all Pose2 transforms given as arguments from left to right."""
    return reduce((lambda x, y: x.compose(y)), poses)


def vee(M):
    """Pose2 vee operator."""
    return vector3(M[0, 2], M[1, 2], M[1, 0])


def delta(g0, g1):
    """Difference between x,y,,theta components of SE(2) poses."""
    return vector3(g1.x() - g0.x(), g1.y() - g0.y(), g1.theta() - g0.theta())


def trajectory(g0, g1, N=20):
    """ Create an interpolated trajectory in SE(2), treating x,y, and theta separately.
        g0 and g1 are the initial and final pose, respectively.
        N is the number of *intervals*
        Returns N+1 poses
    """
    e = delta(g0, g1)
    return [Pose2(g0.x()+e[0]*t, g0.y()+e[1]*t, g0.theta()+e[2]*t) for t in np.linspace(0, 1, N)]


class ThreeLinkArm(object):
    """Three-link arm class."""

    def __init__(self):
        self.L1 = 3.5
        self.L2 = 3.5
        self.L3 = 2.5
        self.xi1 = vector3(0, 0, 1)
        self.xi2 = vector3(self.L1, 0, 1)
        self.xi3 = vector3(self.L1+self.L2, 0, 1)
        self.sXt0 = Pose2(0, self.L1+self.L2 + self.L3, math.radians(90))

    def fk(self, q):
        """ Forward kinematics.
            Takes numpy array of joint angles, in radians.
        """
        sXl1 = Pose2(0, 0, math.radians(90))
        l1Zl1 = Pose2(0, 0, q[0])
        l1Xl2 = Pose2(self.L1, 0, 0)
        l2Zl2 = Pose2(0, 0, q[1])
        l2Xl3 = Pose2(self.L2, 0, 0)
        l3Zl3 = Pose2(0, 0, q[2])
        l3Xt = Pose2(self.L3, 0, 0)
        return compose(sXl1, l1Zl1, l1Xl2, l2Zl2, l2Xl3, l3Zl3, l3Xt)

    def jacobian(self, q):
        """ Calculate manipulator Jacobian.
            Takes numpy array of joint angles, in radians.
        """
        a = q[0]+q[1]
        b = a+q[2]
        return np.array([[-self.L1*math.cos(q[0]) - self.L2*math.cos(a)-self.L3*math.cos(b),
                          -self.L1*math.cos(a)-self.L3*math.cos(b),
                          - self.L3*math.cos(b)],
                         [-self.L1*math.sin(q[0]) - self.L2*math.sin(a)-self.L3*math.sin(b),
                          -self.L1*math.sin(a)-self.L3*math.sin(b),
                          - self.L3*math.sin(b)],
                         [1, 1, 1]], float)

    def poe(self, q):
        """ Forward kinematics.
            Takes numpy array of joint angles, in radians.
        """
        l1Zl1 = Pose2.Expmap(self.xi1 * q[0])
        l2Zl2 = Pose2.Expmap(self.xi2 * q[1])
        l3Zl3 = Pose2.Expmap(self.xi3 * q[2])
        return compose(l1Zl1, l2Zl2, l3Zl3, self.sXt0)

    def con(self, q):
        """ Forward kinematics, conjugation form.
            Takes numpy array of joint angles, in radians.
        """
        def expmap(x, y, theta):
            """Implement exponential map via conjugation with axis (x,y)."""
            return compose(Pose2(x, y, 0), Pose2(0, 0, theta), Pose2(-x, -y, 0))

        l1Zl1 = expmap(0.0, 0.0, q[0])
        l2Zl2 = expmap(0.0, self.L1, q[1])
        l3Zl3 = expmap(0.0, self.L1+self.L2, q[2])
        return compose(l1Zl1, l2Zl2, l3Zl3, self.sXt0)

    def ik(self, sTt_desired, e=1e-9):
        """ Inverse kinematics.
            Takes desired Pose2 of tool T with respect to base S.
            Optional: mu, gradient descent rate; e: error norm threshold
        """
        q = np.radians(vector3(30, -30, 45))  # well within workspace
        error = vector3(100, 100, 100)

        while np.linalg.norm(error) > e:
            error = delta(sTt_desired, self.fk(q))
            J = self.jacobian(q)
            q -= np.dot(np.linalg.pinv(J), error)

        # return result in interval [-pi,pi)
        return np.remainder(q+math.pi, 2*math.pi)-math.pi

    def manipulator_jacobian(self, q):
        """ Calculate manipulator Jacobian.
            Takes numpy array of joint angles, in radians.
            Returns the manipulator Jacobian of differential twists. When multiplied with
            a vector of joint velocities, will yield a single differential twist which is
            the spatial velocity d(sTt)/dt * inv(sTt) of the end-effector pose.
            Just like always, differential twists can be hatted and multiplied with spatial
            coordinates of a point to give the spatial velocity of the point.
        """
        l1Zl1 = Pose2.Expmap(self.xi1 * q[0])
        l2Zl2 = Pose2.Expmap(self.xi2 * q[1])
        # l3Zl3 = Pose2.Expmap(self.xi3 * q[2])

        p1 = self.xi1
        # p1 = Pose2().Adjoint(self.xi1)

        sTl1 = l1Zl1
        p2 = sTl1.Adjoint(self.xi2)

        sTl2 = compose(l1Zl1, l2Zl2)
        p3 = sTl2.Adjoint(self.xi3)

        differential_twists = [p1, p2, p3]
        return np.stack(differential_twists, axis=1)

    def plot(self, fignum, q):
        """ Plot arm.
            Takes figure number, and numpy array of joint angles, in radians.
        """
        fig = plt.figure(fignum)
        axes = fig.gca()

        sXl1 = Pose2(0, 0, math.radians(90))
        p1 = sXl1.translation()
        gtsam_plot.plot_pose2_on_axes(axes, sXl1)

        def plot_line(p, g, color):
            q = g.translation()
            line = np.append(p[np.newaxis], q[np.newaxis], axis=0)
            axes.plot(line[:, 0], line[:, 1], color)
            return q

        l1Zl1 = Pose2(0, 0, q[0])
        l1Xl2 = Pose2(self.L1, 0, 0)
        sTl2 = compose(sXl1, l1Zl1, l1Xl2)
        p2 = plot_line(p1, sTl2, 'r-')
        gtsam_plot.plot_pose2_on_axes(axes, sTl2)

        l2Zl2 = Pose2(0, 0, q[1])
        l2Xl3 = Pose2(self.L2, 0, 0)
        sTl3 = compose(sTl2, l2Zl2, l2Xl3)
        p3 = plot_line(p2, sTl3, 'g-')
        gtsam_plot.plot_pose2_on_axes(axes, sTl3)

        l3Zl3 = Pose2(0, 0, q[2])
        l3Xt = Pose2(self.L3, 0, 0)
        sTt = compose(sTl3, l3Zl3, l3Xt)
        plot_line(p3, sTt, 'b-')
        gtsam_plot.plot_pose2_on_axes(axes, sTt)


# Create common example configurations.
Q0 = vector3(0, 0, 0)
Q1 = np.radians(vector3(-30, -45, -90))
Q2 = np.radians(vector3(-90, 90, 0))


class TestPose2SLAMExample(GtsamTestCase):
    """Unit tests for functions used below."""

    def setUp(self):
        self.arm = ThreeLinkArm()

    def assertPose2Equals(self, actual, expected, tol=1e-2):
        """Helper function that prints out actual and expected if not equal."""
        equal = actual.equals(expected, tol)
        if not equal:
            raise self.failureException(
                "Poses are not equal:\n{}!={}".format(actual, expected))

    def test_fk_arm(self):
        """Make sure forward kinematics is correct for some known test configurations."""
        # at rest
        expected = Pose2(0, 2*3.5 + 2.5, math.radians(90))
        sTt = self.arm.fk(Q0)
        self.assertIsInstance(sTt, Pose2)
        self.assertPose2Equals(sTt, expected)

        # -30, -45, -90
        expected = Pose2(5.78, 1.52, math.radians(-75))
        sTt = self.arm.fk(Q1)
        self.assertPose2Equals(sTt, expected)

    def test_jacobian(self):
        """Test Jacobian calculation."""
        # at rest
        expected = np.array([[-9.5, -6, -2.5], [0, 0, 0], [1, 1, 1]], float)
        J = self.arm.jacobian(Q0)
        np.testing.assert_array_almost_equal(J, expected)

        # at -90, 90, 0
        expected = np.array([[-6, -6, -2.5], [3.5, 0, 0], [1, 1, 1]], float)
        J = self.arm.jacobian(Q2)
        np.testing.assert_array_almost_equal(J, expected)

    def test_con_arm(self):
        """Make sure POE is correct for some known test configurations."""
        # at rest
        expected = Pose2(0, 2*3.5 + 2.5, math.radians(90))
        sTt = self.arm.con(Q0)
        self.assertIsInstance(sTt, Pose2)
        self.assertPose2Equals(sTt, expected)

        # -30, -45, -90
        expected = Pose2(5.78, 1.52, math.radians(-75))
        sTt = self.arm.con(Q1)
        self.assertPose2Equals(sTt, expected)

    def test_poe_arm(self):
        """Make sure POE is correct for some known test configurations."""
        # at rest
        expected = Pose2(0, 2*3.5 + 2.5, math.radians(90))
        sTt = self.arm.poe(Q0)
        self.assertIsInstance(sTt, Pose2)
        self.assertPose2Equals(sTt, expected)

        # -30, -45, -90
        expected = Pose2(5.78, 1.52, math.radians(-75))
        sTt = self.arm.poe(Q1)
        self.assertPose2Equals(sTt, expected)

    def test_ik(self):
        """Check iterative inverse kinematics function."""
        # at rest
        actual = self.arm.ik(Pose2(0, 2*3.5 + 2.5, math.radians(90)))
        np.testing.assert_array_almost_equal(actual, Q0, decimal=2)

        # -30, -45, -90
        sTt_desired = Pose2(5.78, 1.52, math.radians(-75))
        actual = self.arm.ik(sTt_desired)
        self.assertPose2Equals(self.arm.fk(actual), sTt_desired)
        np.testing.assert_array_almost_equal(actual, Q1, decimal=2)

    def test_manipulator_jacobian(self):
        """Test Jacobian calculation."""
        # at rest
        expected = np.array([[0, 3.5, 7], [0, 0, 0], [1, 1, 1]], float)
        J = self.arm.manipulator_jacobian(Q0)
        np.testing.assert_array_almost_equal(J, expected)

        # at -90, 90, 0
        expected = np.array(
            [[0, 0, 3.5], [0, -3.5, -3.5], [1, 1, 1]], float)
        J = self.arm.manipulator_jacobian(Q2)
        np.testing.assert_array_almost_equal(J, expected)


def run_example():
    """ Use trajectory interpolation and then trajectory tracking a la Murray
        to move a 3-link arm on a straight line.
    """
    # Create arm
    arm = ThreeLinkArm()

    # Get initial pose using forward kinematics
    q = np.radians(vector3(30, -30, 45))
    sTt_initial = arm.fk(q)

    # Create interpolated trajectory in task space to desired goal pose
    sTt_goal = Pose2(2.4, 4.3, math.radians(0))
    poses = trajectory(sTt_initial, sTt_goal, 50)

    # Setup figure and plot initial pose
    fignum = 0
    fig = plt.figure(fignum)
    axes = fig.gca()
    axes.set_xlim(-5, 5)
    axes.set_ylim(0, 10)
    gtsam_plot.plot_pose2(fignum, arm.fk(q))

    # For all poses in interpolated trajectory, calculate dq to move to next pose.
    # We do this by calculating the local Jacobian J and doing dq = inv(J)*delta(sTt, pose).
    for pose in poses:
        sTt = arm.fk(q)
        error = delta(sTt, pose)
        J = arm.jacobian(q)
        q += np.dot(np.linalg.inv(J), error)
        arm.plot(fignum, q)
        plt.pause(0.01)

    plt.pause(10)


if __name__ == "__main__":
    run_example()
    unittest.main()
