"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information.

Unit tests for the TG-EqF biased-INS filter wrapper.
Author: Arihant Lunawat
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase
from gtsam_unstable import tgeqf

# Error-state block order: attitude, velocity, position, gyro bias,
# accelerometer bias, virtual bias.
VELOCITY = slice(3, 6)
POSITION = slice(6, 9)
Z_INDEX = 8


def physical_covariance():
    """15x15 initial covariance over [R, v, p, b_w, b_a]."""
    return np.diag([0.05**2] * 3 + [0.1**2] * 3 + [0.2**2] * 3 +
                   [1e-3**2] * 3 + [1e-2**2] * 3)


def imu_noise():
    """Small per-axis IMU noise densities."""
    noise = tgeqf.ImuNoise()
    noise.gyro = np.full(3, 1e-4)
    noise.accel = np.full(3, 1e-3)
    noise.gyro_rw = np.full(3, 1e-6)
    noise.accel_rw = np.full(3, 1e-5)
    return noise


class TestTGEqF(GtsamTestCase):
    """Construct, propagate and update the filter through the wrapper."""

    def setUp(self):
        self.xi_ref = tgeqf.State.identity()
        self.Sigma0 = tgeqf.TGEqF.initialCovariance(physical_covariance())
        self.filter = tgeqf.TGEqF(self.xi_ref, self.Sigma0)

    def test_initial_covariance(self):
        """initialCovariance appends the virtual-bias block to the 15x15."""
        self.assertEqual(self.Sigma0.shape, (18, 18))
        np.testing.assert_allclose(self.Sigma0[:15, :15], physical_covariance())
        np.testing.assert_allclose(self.Sigma0, self.Sigma0.T)

    def test_construction(self):
        """A new filter starts at the reference state with Sigma0."""
        self.assertIsInstance(self.filter.attitude(), gtsam.Rot3)
        self.gtsamAssertEquals(self.filter.attitude(), gtsam.Rot3())
        np.testing.assert_allclose(self.filter.position(), np.zeros(3))
        np.testing.assert_allclose(self.filter.velocity(), np.zeros(3))
        np.testing.assert_allclose(self.filter.biases(), np.zeros(9))
        np.testing.assert_allclose(self.filter.errorCovariance(), self.Sigma0)
        self.assertEqual(self.filter.covariance().shape, (18, 18))

    def test_state_round_trip(self):
        """State.retract and localCoordinates are inverse charts."""
        delta = np.linspace(-0.1, 0.1, 18)
        moved = self.xi_ref.retract(delta)
        np.testing.assert_allclose(self.xi_ref.localCoordinates(moved), delta,
                                   atol=1e-9)
        self.assertEqual(moved.navigationMatrix().shape, (5, 5))

    def test_propagate_at_rest(self):
        """A body at rest stays put while its covariance grows."""
        g_vec = np.array([0.0, 0.0, -9.81])
        for _ in range(100):
            self.filter.propagate(np.zeros(3), -g_vec, g_vec, imu_noise(), 0.01)

        np.testing.assert_allclose(self.filter.position(), np.zeros(3),
                                   atol=1e-9)
        np.testing.assert_allclose(self.filter.velocity(), np.zeros(3),
                                   atol=1e-9)
        P = self.filter.errorCovariance()
        self.assertGreater(P[Z_INDEX, Z_INDEX], self.Sigma0[Z_INDEX, Z_INDEX])
        np.testing.assert_allclose(P, P.T, atol=1e-12)

    def test_update_position(self):
        """A position fix pulls the estimate toward it and shrinks variance."""
        measured = np.array([0.1, -0.05, 0.2])
        # errorCovariance() is a live view, so keep a copy of the prior.
        before = np.diag(self.filter.errorCovariance())[POSITION].copy()
        self.filter.updatePosition(measured, 1e-4 * np.eye(3))

        np.testing.assert_allclose(self.filter.position(), measured, atol=0.01)
        after = np.diag(self.filter.errorCovariance())[POSITION]
        self.assertTrue(np.all(after < before))

    def test_update_dvl(self):
        """A DVL measurement shrinks the velocity variance."""
        before = np.diag(self.filter.errorCovariance())[VELOCITY].copy()
        self.filter.updateDvl(np.array([0.05, 0.0, 0.0]), 1e-4 * np.eye(3))

        self.assertGreater(self.filter.velocity()[0], 0.0)
        after = np.diag(self.filter.errorCovariance())[VELOCITY]
        self.assertTrue(np.all(after < before))

    def test_depth_updates(self):
        """Both depth models move z toward the measured depth."""
        R_depth = np.array([[1e-4]])
        direct = tgeqf.TGEqF(self.xi_ref, self.Sigma0)

        self.filter.updateDepth(0.3, R_depth, 1e6)
        direct.updateDepthDirect(0.3, R_depth)

        for filt in (self.filter, direct):
            self.assertAlmostEqual(filt.position()[2], 0.3, delta=0.01)
            self.assertLess(filt.errorCovariance()[Z_INDEX, Z_INDEX],
                            self.Sigma0[Z_INDEX, Z_INDEX])

    def test_repeated_retrieval(self):
        """const& accessors are views that track and keep alive the filter."""
        state = self.filter.state()
        covariance = self.filter.errorCovariance()
        snapshot = covariance.copy()
        self.filter.updatePosition(np.array([1.0, 0.0, 0.0]), np.eye(3))

        # The earlier views follow the update; the copy does not.
        np.testing.assert_allclose(state.navigationMatrix(),
                                   self.filter.state().navigationMatrix())
        np.testing.assert_allclose(covariance, self.filter.errorCovariance())
        self.assertFalse(np.allclose(covariance, snapshot))
        self.assertGreater(state.navigationMatrix()[0, 4], 0.0)

        # Value-returning accessors give the expected shapes.
        self.assertEqual(self.filter.extendedPose().matrix().shape, (5, 5))
        self.assertEqual(self.filter.groupEstimate().second.shape, (9,))

        # The views outlive the Python handle to the filter.
        del self.filter
        self.assertEqual(state.navigationMatrix().shape, (5, 5))
        self.assertEqual(covariance.shape, (18, 18))


if __name__ == "__main__":
    unittest.main()
