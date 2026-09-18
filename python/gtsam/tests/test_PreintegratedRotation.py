"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for the PreintegratedRotation Python bindings.
"""

import unittest

import gtsam
import numpy as np
from gtsam.utils.test_case import GtsamTestCase


class TestPreintegratedRotation(GtsamTestCase):
    """Tests for wrapped rotation preintegration utilities."""

    def test_integrate_sequential_rotations_accepts_numpy_arrays(self):
        """Sequential rotations accept C- and F-contiguous arrays."""
        times = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
        omega = np.array([0.1, -0.2, 0.3])
        measured_omegas_c = np.tile(omega, (times.size, 1))
        measured_omegas_f = np.asfortranarray(measured_omegas_c)
        expected = gtsam.Rot3.Expmap(omega)

        for measured_omegas in (measured_omegas_c, measured_omegas_f):
            order = "F" if measured_omegas.flags["F_CONTIGUOUS"] else "C"
            with self.subTest(order=order):
                actual = gtsam.integrateSequentialRotations(
                    times, measured_omegas)
                self.gtsamAssertEquals(expected, actual, tol=1e-12)

        with self.assertRaises(TypeError):
            gtsam.integrateSequentialRotations(times,
                                               measured_omegas_c.tolist())

    def test_integrate_single_speed_coning_accepts_numpy_arrays(self):
        """Single-speed coning accepts C- and F-contiguous arrays."""
        times = np.array([0.0, 0.5, 1.0])
        omega = np.array([0.15, 0.0, -0.05])
        bias = np.array([0.3, -0.4, 0.5])
        measured_omegas_c = np.tile(omega + bias, (times.size, 1))
        measured_omegas_f = np.asfortranarray(measured_omegas_c)
        expected = gtsam.Rot3.Expmap(omega)

        for measured_omegas in (measured_omegas_c, measured_omegas_f):
            order = "F" if measured_omegas.flags["F_CONTIGUOUS"] else "C"
            with self.subTest(order=order):
                actual = gtsam.integrateSingleSpeedConing(
                    times, measured_omegas, bias)
                self.gtsamAssertEquals(expected, actual, tol=1e-12)

        with self.assertRaises(TypeError):
            gtsam.integrateSingleSpeedConing(times,
                                             measured_omegas_c.tolist(), bias)

    def test_integrators_apply_sensor_rotation(self):
        """Wrapped utilities rotate sensor samples into the body frame."""
        times = np.array([0.0, 0.5, 1.0])
        sensor_omega = np.array([0.2, 0.0, 0.0])
        measured_omegas = np.tile(sensor_omega, (times.size, 1))
        body_R_sensor = gtsam.Rot3.Yaw(np.pi / 2.0)
        expected = gtsam.Rot3.Expmap(body_R_sensor.rotate(sensor_omega))

        sequential = gtsam.integrateSequentialRotations(
            times, measured_omegas, np.zeros(3), body_R_sensor)
        single_speed = gtsam.integrateSingleSpeedConing(
            times, measured_omegas, np.zeros(3), body_R_sensor)

        self.gtsamAssertEquals(expected, sequential, tol=1e-12)
        self.gtsamAssertEquals(expected, single_speed, tol=1e-12)


if __name__ == "__main__":
    unittest.main()
