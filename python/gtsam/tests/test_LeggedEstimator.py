"""Wrapper tests for the legged estimator runtime API."""

import unittest

import numpy as np

import gtsam

from gtsam.examples.LeggedEstimatorExample import make_legged_estimator_params


def make_simple_estimator():
    params = make_legged_estimator_params()
    params.useFullContactInitialization = False

    nav_state0 = gtsam.NavState(
        gtsam.Rot3(), np.array([0.0, 0.0, 0.12]), np.array([0.0, 0.0, -0.12])
    )
    footholds0 = np.zeros((3, 4), dtype=float)
    covariance0 = np.zeros((21, 21), dtype=float)
    covariance0[:9, :9] = np.eye(9) * 1e-3
    covariance0[9:, 9:] = np.eye(12) * (params.footholdInitSigma**2)
    foot_names = ["lf", "rf", "lh", "rh"]
    return gtsam.LeggedInvariantEKF(
        nav_state0, footholds0, covariance0, params, foot_names
    )


def make_estimator(variant: str):
    params = make_legged_estimator_params()
    params.useFullContactInitialization = False

    nav_state0 = gtsam.NavState(
        gtsam.Rot3(), np.array([0.0, 0.0, 0.12]), np.array([0.0, 0.0, -0.12])
    )
    footholds0 = np.zeros((3, 4), dtype=float)
    foot_names = ["lf", "rf", "lh", "rh"]

    if variant in ("invariant_ekf", "invariant_graph"):
        covariance0 = np.zeros((21, 21), dtype=float)
        covariance0[:9, :9] = np.eye(9) * 1e-3
        covariance0[9:, 9:] = np.eye(12) * (params.footholdInitSigma**2)
        if variant == "invariant_ekf":
            return gtsam.LeggedInvariantEKF(
                nav_state0, footholds0, covariance0, params, foot_names
            )
        return gtsam.LeggedInvariantIEKF(
            nav_state0, footholds0, covariance0, params, foot_names
        )

    base_covariance0 = np.eye(9) * 1e-3
    if variant == "fixed_lag_single_bias":
        return gtsam.LeggedFixedLagSmoother(
            nav_state0, footholds0, base_covariance0, params, 1.0, foot_names
        )
    if variant == "fixed_lag_combined_bias":
        return gtsam.LeggedCombinedFixedLagSmoother(
            nav_state0, footholds0, base_covariance0, params, 1.0, foot_names
        )
    raise ValueError(f"unknown estimator variant: {variant}")


def single_contact_update(estimator, terrain_height):
    if terrain_height is None:
        estimator.turnHeightPriorOff()
    else:
        estimator.turnHeightPriorOn(terrain_height)

    measurement = gtsam.ContactMeasurement()
    measurement.foot = 1
    measurement.bodyPoint = np.zeros(3)
    measurement.touchdown = False
    estimator.processContacts([measurement])
    estimate = estimator.estimate()
    return float(estimate.x(3)[2])


class TestLeggedEstimator(unittest.TestCase):
    def test_wrapper_inheritance_uses_base_height_prior_methods(self):
        self.assertIs(gtsam.LeggedInvariantIEKF.__mro__[1], gtsam.LeggedInvariantEKF)

        variants = (
            "invariant_ekf",
            "invariant_graph",
            "fixed_lag_single_bias",
            "fixed_lag_combined_bias",
        )
        for variant in variants:
            cls = type(make_estimator(variant))
            self.assertTrue(callable(getattr(cls, "turnHeightPriorOn")))
            self.assertTrue(callable(getattr(cls, "turnHeightPriorOff")))
            no_prior_foot_z = single_contact_update(make_estimator(variant), None)
            high_prior_foot_z = single_contact_update(make_estimator(variant), 10.0)
            low_prior_foot_z = single_contact_update(make_estimator(variant), -10.0)
            self.assertNotAlmostEqual(no_prior_foot_z, high_prior_foot_z, places=6)
            self.assertNotAlmostEqual(high_prior_foot_z, low_prior_foot_z, places=6)

    def test_height_prior_api_changes_filter_estimate_in_python(self):
        no_prior_estimator = make_simple_estimator()
        high_prior_estimator = make_simple_estimator()
        low_prior_estimator = make_simple_estimator()

        no_prior_foot_z = single_contact_update(no_prior_estimator, None)
        high_prior_foot_z = single_contact_update(high_prior_estimator, 10.0)
        low_prior_foot_z = single_contact_update(low_prior_estimator, -10.0)

        self.assertNotAlmostEqual(no_prior_foot_z, high_prior_foot_z, places=6)
        self.assertNotAlmostEqual(high_prior_foot_z, low_prior_foot_z, places=6)

    def test_height_prior_can_be_turned_off_again(self):
        estimator = make_simple_estimator()

        estimator.turnHeightPriorOn(3.0)
        estimator.turnHeightPriorOff()

        disabled_foot_z = single_contact_update(estimator, None)
        fresh_disabled_foot_z = single_contact_update(make_simple_estimator(), None)
        self.assertAlmostEqual(disabled_foot_z, fresh_disabled_foot_z, places=9)


if __name__ == "__main__":
    unittest.main()
