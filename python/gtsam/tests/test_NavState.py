"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

NavState unit tests.
"""

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from gtsam.utils.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)

from gtsam import NavState, Point3, Rot3, Unit3


class TestNavState(GtsamTestCase):
    """Test selected NavState methods."""

    def test_range_point_derivatives(self):
        """Test NavState range to Point3 Jacobians."""
        state = NavState(
            Rot3.Rodrigues(0.3, 0.2, 0.1),
            Point3(3.5, -8.2, 4.2),
            np.array([0.4, 0.5, 0.6]),
        )
        point = Point3(1, 4, -4)

        jacobian_state = np.zeros((1, 9), order="F")
        jacobian_point = np.zeros((1, 3), order="F")
        state.range(point, jacobian_state, jacobian_point)

        jacobian_numerical_state = numericalDerivative21(NavState.range, state, point)
        jacobian_numerical_point = numericalDerivative22(NavState.range, state, point)
        self.gtsamAssertEquals(jacobian_state, jacobian_numerical_state)
        self.gtsamAssertEquals(jacobian_point, jacobian_numerical_point)

    def test_bearing_point_derivatives(self):
        """Test NavState bearing to Point3 Jacobians."""
        state = NavState(
            Rot3.Rodrigues(0.3, 0.2, 0.1),
            Point3(3.5, -8.2, 4.2),
            np.array([0.4, 0.5, 0.6]),
        )
        point = Point3(1, 4, -4)

        expected = Unit3(state.pose().transformTo(point))
        actual = state.bearing(point)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        jacobian_state = np.zeros((2, 9), order="F")
        jacobian_point = np.zeros((2, 3), order="F")
        state.bearing(point, jacobian_state, jacobian_point)

        jacobian_numerical_state = numericalDerivative21(NavState.bearing, state, point)
        jacobian_numerical_point = numericalDerivative22(NavState.bearing, state, point)
        self.gtsamAssertEquals(jacobian_state, jacobian_numerical_state)
        self.gtsamAssertEquals(jacobian_point, jacobian_numerical_point)


if __name__ == "__main__":
    unittest.main()
