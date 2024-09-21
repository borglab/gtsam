"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for IMU testing scenarios.
Author: Frank Dellaert & Joel Truher
"""
# pylint: disable=invalid-name, no-name-in-module

import unittest
import numpy as np

from gtsam import Pose3, Rot3, Point3
from gtsam.utils.numerical_derivative import numericalDerivative11, numericalDerivative21, numericalDerivative22, numericalDerivative33


class TestNumericalDerivatives(unittest.TestCase):
    def test_numericalDerivative11_scalar(self):
        # Test function of one variable
        def h(x):
            return x ** 2

        x = np.array([3.0])
        # Analytical derivative: dh/dx = 2x
        analytical_derivative = np.array([[2.0 * x[0]]])

        # Compute numerical derivative
        numerical_derivative = numericalDerivative11(h, x)

        # Check if numerical derivative is close to analytical derivative
        np.testing.assert_allclose(
            numerical_derivative, analytical_derivative, rtol=1e-5
        )

    def test_numericalDerivative11_vector(self):
        # Test function of one vector variable
        def h(x):
            return x ** 2

        x = np.array([1.0, 2.0, 3.0])
        # Analytical derivative: dh/dx = 2x
        analytical_derivative = np.diag(2.0 * x)

        numerical_derivative = numericalDerivative11(h, x)

        np.testing.assert_allclose(
            numerical_derivative, analytical_derivative, rtol=1e-5
        )

    def test_numericalDerivative21(self):
        # Test function of two variables, derivative with respect to first variable
        def h(x1, x2):
            return x1 * np.sin(x2)

        x1 = np.array([2.0])
        x2 = np.array([np.pi / 4])
        # Analytical derivative: dh/dx1 = sin(x2)
        analytical_derivative = np.array([[np.sin(x2[0])]])

        numerical_derivative = numericalDerivative21(h, x1, x2)

        np.testing.assert_allclose(
            numerical_derivative, analytical_derivative, rtol=1e-5
        )

    def test_numericalDerivative22(self):
        # Test function of two variables, derivative with respect to second variable
        def h(x1, x2):
            return x1 * np.sin(x2)

        x1 = np.array([2.0])
        x2 = np.array([np.pi / 4])
        # Analytical derivative: dh/dx2 = x1 * cos(x2)
        analytical_derivative = np.array([[x1[0] * np.cos(x2[0])]])

        numerical_derivative = numericalDerivative22(h, x1, x2)

        np.testing.assert_allclose(
            numerical_derivative, analytical_derivative, rtol=1e-5
        )

    def test_numericalDerivative33(self):
        # Test function of three variables, derivative with respect to third variable
        def h(x1, x2, x3):
            return x1 * x2 + np.exp(x3)

        x1 = np.array([1.0])
        x2 = np.array([2.0])
        x3 = np.array([0.5])
        # Analytical derivative: dh/dx3 = exp(x3)
        analytical_derivative = np.array([[np.exp(x3[0])]])

        numerical_derivative = numericalDerivative33(h, x1, x2, x3)

        np.testing.assert_allclose(
            numerical_derivative, analytical_derivative, rtol=1e-5
        )

    def test_numericalDerivative_with_pose(self):
        # Test function with manifold and vector inputs

        def h(pose:Pose3, point:np.ndarray):
            return pose.transformFrom(point)

        # Values from testPose3.cpp
        P = Point3(0.2,0.7,-2) 
        R = Rot3.Rodrigues(0.3,0,0)   
        P2 = Point3(3.5,-8.2,4.2)  
        T = Pose3(R,P2)    

        analytic_H1 = np.zeros((3,6), order='F', dtype=float)
        analytic_H2 = np.zeros((3,3), order='F', dtype=float)
        y = T.transformFrom(P, analytic_H1, analytic_H2)

        numerical_H1 = numericalDerivative21(h, T, P)
        numerical_H2 = numericalDerivative22(h, T, P)

        np.testing.assert_allclose(numerical_H1, analytic_H1, rtol=1e-5)
        np.testing.assert_allclose(numerical_H2, analytic_H2, rtol=1e-5)

if __name__ == "__main__":
    unittest.main()