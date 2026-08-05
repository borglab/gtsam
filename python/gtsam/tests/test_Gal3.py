"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Gal3 unit tests.
"""

import unittest

import numpy as np
from gtsam.utils.test_case import GtsamTestCase
from gtsam.utils.numerical_derivative import (
    numericalDerivative21,
    numericalDerivative22,
)

import gtsam
from gtsam import Gal3, Point3, Rot3, Unit3


class TestGal3(GtsamTestCase):
    """Test selected Gal3 methods."""

    @staticmethod
    def make_state(yaw: float, position, velocity, time: float) -> Gal3:
        return Gal3(Rot3.Rz(yaw), np.array(position), np.array(velocity), time)

    def test_range_point_derivatives(self):
        """Test Gal3 range to Point3 Jacobians."""
        state = Gal3(
            Rot3.Rz(0.1), np.array([0.2, 0.3, 0.4]), np.array([0.5, 0.6, 0.7]), 0.8
        )
        point = Point3(1, 4, -4)

        jacobian_state = np.zeros((1, 10), order="F")
        jacobian_point = np.zeros((1, 3), order="F")
        state.range(point, jacobian_state, jacobian_point)

        jacobian_numerical_state = numericalDerivative21(Gal3.range, state, point)
        jacobian_numerical_point = numericalDerivative22(Gal3.range, state, point)
        self.gtsamAssertEquals(jacobian_state, jacobian_numerical_state)
        self.gtsamAssertEquals(jacobian_point, jacobian_numerical_point)

    def test_bearing_point_derivatives(self):
        """Test Gal3 bearing to Point3 Jacobians."""
        state = Gal3(
            Rot3.Rz(0.1), np.array([0.2, 0.3, 0.4]), np.array([0.5, 0.6, 0.7]), 0.8
        )
        point = Point3(1, 4, -4)

        expected = Unit3(
            gtsam.Pose3(state.rotation(), state.translation()).transformTo(point)
        )
        actual = state.bearing(point)
        self.gtsamAssertEquals(actual, expected, 1e-6)

        jacobian_state = np.zeros((2, 10), order="F")
        jacobian_point = np.zeros((2, 3), order="F")
        state.bearing(point, jacobian_state, jacobian_point)

        jacobian_numerical_state = numericalDerivative21(Gal3.bearing, state, point)
        jacobian_numerical_point = numericalDerivative22(Gal3.bearing, state, point)
        self.gtsamAssertEquals(jacobian_state, jacobian_numerical_state)
        self.gtsamAssertEquals(jacobian_point, jacobian_numerical_point)

    def test_prior_and_between_factors(self):
        """Test wrapped PriorFactorGal3 and BetweenFactorGal3."""
        state1 = self.make_state(0.1, (0.2, 0.3, 0.4), (0.5, 0.6, 0.7), 0.8)
        state2 = self.make_state(-0.2, (0.7, -0.1, 0.9), (0.4, 0.2, 0.3), 1.1)
        model = gtsam.noiseModel.Unit.Create(10)

        prior_factor = gtsam.PriorFactorGal3(0, state1, model)
        self.gtsamAssertEquals(prior_factor.prior(), state1, 1e-9)

        between_measurement = state1.between(state2)
        between_factor = gtsam.BetweenFactorGal3(0, 1, between_measurement, model)
        self.gtsamAssertEquals(between_factor.measured(), between_measurement, 1e-9)

        values = gtsam.Values()
        values.insert(0, state1)
        values.insert(1, state2)

        self.assertAlmostEqual(prior_factor.error(values), 0.0, places=9)
        self.assertAlmostEqual(between_factor.error(values), 0.0, places=9)


if __name__ == "__main__":
    unittest.main()
