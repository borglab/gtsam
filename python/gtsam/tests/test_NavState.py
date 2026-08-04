"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

NavState unit tests.
"""

import unittest

import numpy as np
import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam.utils.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)

from gtsam import NavState, Point3, Rot3, Unit3


class TestNavState(GtsamTestCase):
    """Test selected NavState methods."""

    @staticmethod
    def make_state(yaw: float, position, velocity) -> NavState:
        return NavState(Rot3.Rz(yaw), Point3(*position), np.array(velocity))

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

    def test_prior_and_between_factors(self):
        """Test wrapped PriorFactorNavState and BetweenFactorNavState."""
        state1 = self.make_state(0.1, (1.0, 2.0, 3.0), (0.4, 0.5, 0.6))
        state2 = self.make_state(-0.2, (1.5, 2.5, 4.0), (0.1, 0.3, 0.7))
        model = gtsam.noiseModel.Unit.Create(9)

        prior_factor = gtsam.PriorFactorNavState(0, state1, model)
        self.gtsamAssertEquals(prior_factor.prior(), state1, 1e-9)

        between_measurement = state1.between(state2)
        between_factor = gtsam.BetweenFactorNavState(0, 1, between_measurement, model)
        self.gtsamAssertEquals(between_factor.measured(), between_measurement, 1e-9)

        values = gtsam.Values()
        values.insert(0, state1)
        values.insert(1, state2)

        self.assertAlmostEqual(prior_factor.error(values), 0.0, places=9)
        self.assertAlmostEqual(between_factor.error(values), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
