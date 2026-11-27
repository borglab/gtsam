"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the VioEqF (equivariant visual-inertial odometry) wrapper.
Author: Rohan Bansal
"""
import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase

eqvio = gtsam_unstable.eqvio

# Standard gravity used by VioEqF (GRAVITY_CONSTANT in VioEqFCommon.h); a
# stationary IMU measures this along world +Z.
GRAVITY = 9.80665


def stationary_imu() -> "eqvio.IMUInput":
    """IMU sample of a body at rest with its z-axis aligned to gravity."""
    imu = eqvio.IMUInput()
    imu.gyr = np.zeros(3)
    imu.acc = np.array([0.0, 0.0, GRAVITY])
    return imu


class TestVioEqFParams(GtsamTestCase):
    """VioEqFParams exposes its defaults and accepts assignment."""

    def test_defaults(self):
        params = eqvio.VioEqFParams()
        self.assertEqual(params.initialPointDepth, 10.0)
        self.assertEqual(params.initialPointVariance, 1.0)
        self.assertEqual(params.featureRetention, 0.3)
        np.testing.assert_allclose(params.inputNoise, 1e-3 * np.eye(12))

    def test_assignment(self):
        params = eqvio.VioEqFParams()
        params.initialPointDepth = 5.0
        params.pointProcessVariance = 0.02
        params.inputNoise = 2e-3 * np.eye(12)
        self.assertEqual(params.initialPointDepth, 5.0)
        self.assertEqual(params.pointProcessVariance, 0.02)
        np.testing.assert_allclose(params.inputNoise, 2e-3 * np.eye(12))


class TestVioEqF(GtsamTestCase):
    """VioEqF construction, propagation, and visual updates from Python."""

    def setUp(self):
        self.params = eqvio.VioEqFParams()
        # Identity intrinsics, so measurements are normalized image coordinates.
        self.camera = gtsam.PinholeCameraCal3_S2(gtsam.Pose3(), gtsam.Cal3_S2())
        # Symbol keys exceed 32 bits and exercise 64-bit Key conversion.
        self.measurement = {
            gtsam.symbol('l', 1): np.array([0.1, -0.05]),
            gtsam.symbol('l', 2): np.array([-0.2, 0.15]),
        }

    def initialized_filter(self) -> "eqvio.VioEqF":
        """Filter initialized from a stationary, gravity-aligned IMU sample."""
        vio = eqvio.VioEqF(self.params)
        vio.initializeFromIMU(stationary_imu())
        return vio

    def test_default_state(self):
        state = eqvio.State()
        self.assertEqual(state.n(), 0)
        self.assertEqual(state.dim(), 21)
        self.gtsamAssertEquals(state.pose(), gtsam.Pose3())

    def test_initialize_from_imu(self):
        vio = eqvio.VioEqF(self.params)
        self.assertFalse(vio.isInitialized())
        vio.initializeFromIMU(stationary_imu())
        self.assertTrue(vio.isInitialized())
        self.assertEqual(vio.landmarkCount(), 0)

    def test_stationary_predict(self):
        vio = self.initialized_filter()
        for _ in range(100):
            vio.predict(stationary_imu(), 0.01)
        np.testing.assert_allclose(vio.position(), np.zeros(3), atol=1e-9)
        np.testing.assert_allclose(vio.velocity(), np.zeros(3), atol=1e-9)

    def test_seeded_constructor(self):
        vio = eqvio.VioEqF(eqvio.State(), np.eye(21), [], self.params)
        self.assertTrue(vio.isInitialized())
        self.assertEqual(vio.landmarkCount(), 0)

    def test_seeded_constructor_rejects_extra_keys(self):
        with self.assertRaises(ValueError):
            eqvio.VioEqF(eqvio.State(), np.eye(21), [gtsam.symbol('l', 1)],
                         self.params)

    def test_update_adds_landmarks(self):
        vio = self.initialized_filter()
        R = 1e-4 * np.eye(2 * len(self.measurement))
        vio.update(self.measurement, self.camera, R)
        self.assertEqual(vio.landmarkCount(), 2)

        # Re-observing the same landmarks must not duplicate them.
        vio.update(self.measurement, self.camera, R)
        self.assertEqual(vio.landmarkCount(), 2)

    def test_update_rejects_wrong_sized_covariance(self):
        vio = self.initialized_filter()
        with self.assertRaises(ValueError):
            vio.update(self.measurement, self.camera, np.eye(3))


if __name__ == "__main__":
    unittest.main()
