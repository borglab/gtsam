"""
GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

DopplerFactor / DopplerFactorArm python binding unit tests.
Author: inuex35
"""
import unittest

import numpy as np

import gtsam
from gtsam.utils.test_case import GtsamTestCase

# Physical constants, matching gtsam::gnss::C_LIGHT / OMGE and the L1
# wavelength used by the C++ unit tests (gnssTestHelpers.h).
C_LIGHT = 299792458.0
OMGE = 7.2921151467e-5
LAMBDA_L1 = 0.190293672798365

# Sample ECEF geometry shared with the C++ tests.
SAT_POS = np.array([-5824269.46342, -22935011.26952, -12195522.22428])
RCV_POS = np.array([-3961908.12, 3348995.59, 3698211.13])


def line_of_sight(sat_pos, rcv_pos):
    """Unit line-of-sight vector, receiver -> satellite (matches gnss::geodist)."""
    dr = sat_pos - rcv_pos
    return dr / np.linalg.norm(dr)


def sagnac_rate(sat_pos, sat_vel, rcv_pos, rcv_vel):
    """Earth-rotation (Sagnac) range-rate term.

    The time derivative of the Sagnac term gnss::geodist adds to the range,
    k * (sat.x * rcv.y - sat.y * rcv.x).
    """
    k = OMGE / C_LIGHT
    return k * (sat_vel[0] * rcv_pos[1] + sat_pos[0] * rcv_vel[1] -
                sat_vel[1] * rcv_pos[0] - sat_pos[1] * rcv_vel[0])


def expected_error(sat_vel, rcv_vel, meas_doppler, dt, bias_prev, bias_curr,
                   sat_clk_drift):
    """Independent reference for the Doppler range-rate error at a given state."""
    e = line_of_sight(SAT_POS, RCV_POS)
    drift = (bias_curr - bias_prev) / dt
    range_rate = (e.dot(sat_vel - rcv_vel) +
                  C_LIGHT * (drift - sat_clk_drift) +
                  sagnac_rate(SAT_POS, sat_vel, RCV_POS, rcv_vel))
    meas_range_rate = -LAMBDA_L1 * meas_doppler
    return range_rate - meas_range_rate


class TestDopplerFactor(GtsamTestCase):
    def setUp(self):
        self.model = gtsam.noiseModel.Unit.Create(1)

    def test_model(self):
        """evaluateError must match an independent range-rate reference."""
        sat_vel = np.array([-1200.0, 2400.0, 800.0])
        rcv_vel = np.array([0.3, -0.1, 0.05])
        meas_doppler = -1500.0
        sat_clk_drift = 1.2e-9
        rcv_clk_drift = 4.5e-9
        dt = 0.2
        bias_prev = 1.0e-6
        bias_curr = bias_prev + rcv_clk_drift * dt

        factor = gtsam.DopplerFactor(0, 1, 2, meas_doppler, LAMBDA_L1, SAT_POS,
                                     sat_vel, RCV_POS, dt, sat_clk_drift,
                                     self.model)

        error = factor.evaluateError(rcv_vel, bias_prev, bias_curr)
        expected = expected_error(sat_vel, rcv_vel, meas_doppler, dt, bias_prev,
                                  bias_curr, sat_clk_drift)
        self.assertAlmostEqual(error[0], expected, places=6)

        # Accessors.
        self.assertAlmostEqual(factor.measuredRangeRate(),
                               -LAMBDA_L1 * meas_doppler, places=9)
        self.assertAlmostEqual(factor.dt(), dt, places=12)
        np.testing.assert_allclose(factor.lineOfSight(),
                                   line_of_sight(SAT_POS, RCV_POS), atol=1e-12)

    def test_common_bias_offset_invariance(self):
        """Doppler observes clock drift, not absolute bias: a common offset on
        both clock-bias states must leave the error unchanged."""
        sat_vel = np.array([-1200.0, 2400.0, 800.0])
        rcv_vel = np.array([0.3, -0.1, 0.05])
        dt = 1.0
        bias_prev = 1.0e-6
        bias_curr = 1.0045e-6

        factor = gtsam.DopplerFactor(0, 1, 2, -1500.0, LAMBDA_L1, SAT_POS,
                                     sat_vel, RCV_POS, dt, 1.2e-9, self.model)
        e1 = factor.evaluateError(rcv_vel, bias_prev, bias_curr)[0]
        offset = 3.7e-4
        e2 = factor.evaluateError(rcv_vel, bias_prev + offset,
                                  bias_curr + offset)[0]
        self.assertAlmostEqual(e1, e2, places=6)

    def test_invalid_dt_throws(self):
        sat_vel = np.array([100.0, 200.0, 300.0])
        with self.assertRaises((ValueError, RuntimeError)):
            gtsam.DopplerFactor(0, 1, 2, 10.0, LAMBDA_L1, SAT_POS, sat_vel,
                                RCV_POS, 0.0, 0.0, self.model)
        with self.assertRaises((ValueError, RuntimeError)):
            gtsam.DopplerFactor(0, 1, 2, 10.0, LAMBDA_L1, SAT_POS, sat_vel,
                                RCV_POS, -1.0, 0.0, self.model)

    def test_equality(self):
        sat_vel = np.array([100.0, 200.0, 300.0])
        factor1 = gtsam.DopplerFactor(0, 1, 2, 10.0, LAMBDA_L1, SAT_POS,
                                      sat_vel, RCV_POS, 1.0, 0.0, self.model)
        factor2 = gtsam.DopplerFactor(0, 1, 2, 10.0, LAMBDA_L1, SAT_POS,
                                      sat_vel, RCV_POS, 1.0, 0.0, self.model)
        factor3 = gtsam.DopplerFactor(0, 1, 2, 99.0, LAMBDA_L1, SAT_POS,
                                      sat_vel, RCV_POS, 1.0, 0.0, self.model)
        factor4 = gtsam.DopplerFactor(0, 1, 2, 10.0, LAMBDA_L1, SAT_POS,
                                      sat_vel, RCV_POS, 0.5, 0.0, self.model)
        self.assertTrue(factor1.equals(factor2, 1e-9))
        self.assertFalse(factor1.equals(factor3, 1e-9))
        self.assertFalse(factor1.equals(factor4, 1e-9))  # differs only in dt
        factor1.print("doppler ")

    @unittest.skipUnless(hasattr(gtsam.DopplerFactor, "serialize"),
                         "Serialization not enabled")
    def test_serialization(self):
        sat_vel = np.array([100.0, 200.0, 300.0])
        factor = gtsam.DopplerFactor(0, 1, 2, 10.0, LAMBDA_L1, SAT_POS, sat_vel,
                                     RCV_POS, 1.0, 0.0, self.model)
        factor.serialize()


class TestDopplerFactorArm(GtsamTestCase):
    def setUp(self):
        self.model = gtsam.noiseModel.Unit.Create(1)

    def test_reduces_to_base_when_no_rotation_rate(self):
        """With omega = 0 the lever velocity vanishes, so the arm factor must
        match DopplerFactor at the same velocity, whatever the attitude is."""
        sat_vel = np.array([-1200.0, 2400.0, 800.0])
        rcv_vel = np.array([0.3, -0.1, 0.05])
        meas_doppler, sat_clk_drift = -1500.0, 1.2e-9
        dt, bias_prev = 0.2, 1.0e-6
        bias_curr = bias_prev + 4.5e-9 * dt
        lever = np.array([0.5, -0.3, 1.0])
        omega = np.array([0.0, 0.0, 0.0])
        pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.3, -0.2, 0.5), RCV_POS)

        arm = gtsam.DopplerFactorArm(0, 1, 2, 3, meas_doppler, LAMBDA_L1,
                                     SAT_POS, sat_vel, RCV_POS, lever, omega, dt,
                                     sat_clk_drift, self.model)
        base = gtsam.DopplerFactor(1, 2, 3, meas_doppler, LAMBDA_L1, SAT_POS,
                                   sat_vel, RCV_POS, dt, sat_clk_drift,
                                   self.model)
        self.assertAlmostEqual(
            base.evaluateError(rcv_vel, bias_prev, bias_curr)[0],
            arm.evaluateError(pose, rcv_vel, bias_prev, bias_curr)[0],
            places=9)

    def test_model(self):
        """Arm factor error must match a reference using the lever-arm antenna
        velocity v_ant = v + R * (omega x lever)."""
        sat_vel = np.array([-1200.0, 2400.0, 800.0])
        rcv_vel = np.array([0.3, -0.1, 0.05])
        meas_doppler, sat_clk_drift, rcv_clk_drift = -1500.0, 1.2e-9, 4.5e-9
        dt, bias_prev = 0.2, 1.0e-6
        bias_curr = bias_prev + rcv_clk_drift * dt
        lever = np.array([0.5, -0.3, 1.0])
        omega = np.array([0.02, -0.05, 0.1])
        pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.3, -0.2, 0.5), RCV_POS)

        factor = gtsam.DopplerFactorArm(0, 1, 2, 3, meas_doppler, LAMBDA_L1,
                                        SAT_POS, sat_vel, RCV_POS, lever, omega,
                                        dt, sat_clk_drift, self.model)

        v_ant = rcv_vel + pose.rotation().rotate(np.cross(omega, lever))
        e = line_of_sight(SAT_POS, RCV_POS)
        range_rate = (e.dot(sat_vel - v_ant) +
                      C_LIGHT * (rcv_clk_drift - sat_clk_drift) +
                      sagnac_rate(SAT_POS, sat_vel, RCV_POS, v_ant))
        expected = range_rate - (-LAMBDA_L1 * meas_doppler)
        error = factor.evaluateError(pose, rcv_vel, bias_prev, bias_curr)
        self.assertAlmostEqual(error[0], expected, places=6)
        np.testing.assert_allclose(factor.leverArm(), lever, atol=1e-12)

    def test_nav_frame_overload(self):
        """The ecef_T_nav overload takes a local nav-frame pose, and the
        velocity is then a nav-frame velocity rotated to ECEF by ecef_R_nav."""
        sat_vel = np.array([-1200.0, 2400.0, 800.0])
        nav_vel = np.array([0.3, -0.1, 0.05])  # receiver velocity in nav frame
        meas_doppler, sat_clk_drift, rcv_clk_drift = -1500.0, 1.2e-9, 4.5e-9
        dt, bias_prev = 0.2, 1.0e-6
        bias_curr = bias_prev + rcv_clk_drift * dt
        lever = np.array([0.5, -0.3, 1.0])
        omega = np.array([0.02, -0.05, 0.1])
        ecef_T_nav = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.1, 0.4, -0.7), RCV_POS)
        nav_pose = gtsam.Pose3(gtsam.Rot3.RzRyRx(0.3, -0.2, 0.5), np.zeros(3))

        factor = gtsam.DopplerFactorArm(0, 1, 2, 3, meas_doppler, LAMBDA_L1,
                                        SAT_POS, sat_vel, RCV_POS, lever,
                                        ecef_T_nav, omega, dt, sat_clk_drift,
                                        self.model)

        # Antenna velocity in the nav frame, rotated to ECEF by ecef_R_nav.
        v_ant_nav = nav_vel + nav_pose.rotation().rotate(np.cross(omega, lever))
        v_ant = ecef_T_nav.rotation().rotate(v_ant_nav)
        e = line_of_sight(SAT_POS, RCV_POS)
        range_rate = (e.dot(sat_vel - v_ant) +
                      C_LIGHT * (rcv_clk_drift - sat_clk_drift) +
                      sagnac_rate(SAT_POS, sat_vel, RCV_POS, v_ant))
        expected = range_rate - (-LAMBDA_L1 * meas_doppler)
        error = factor.evaluateError(nav_pose, nav_vel, bias_prev, bias_curr)
        self.assertAlmostEqual(error[0], expected, places=6)

    def test_invalid_dt_throws(self):
        sat_vel = np.array([100.0, 200.0, 300.0])
        lever = np.array([0.5, -0.3, 1.0])
        omega = np.array([0.02, 0.0, 0.1])
        with self.assertRaises((ValueError, RuntimeError)):
            gtsam.DopplerFactorArm(0, 1, 2, 3, 10.0, LAMBDA_L1, SAT_POS,
                                   sat_vel, RCV_POS, lever, omega, 0.0, 0.0,
                                   self.model)

    def test_equality(self):
        sat_vel = np.array([100.0, 200.0, 300.0])
        lever = np.array([0.5, -0.3, 1.0])
        omega = np.array([0.02, 0.0, 0.1])
        factor1 = gtsam.DopplerFactorArm(0, 1, 2, 3, 10.0, LAMBDA_L1, SAT_POS,
                                         sat_vel, RCV_POS, lever, omega, 1.0,
                                         0.0, self.model)
        factor2 = gtsam.DopplerFactorArm(0, 1, 2, 3, 10.0, LAMBDA_L1, SAT_POS,
                                         sat_vel, RCV_POS, lever, omega, 1.0,
                                         0.0, self.model)
        factor3 = gtsam.DopplerFactorArm(0, 1, 2, 3, 10.0, LAMBDA_L1, SAT_POS,
                                         sat_vel, RCV_POS,
                                         np.array([1.0, 0.0, 0.0]), omega, 1.0,
                                         0.0, self.model)
        self.assertTrue(factor1.equals(factor2, 1e-9))
        self.assertFalse(factor1.equals(factor3, 1e-9))  # differs in lever arm
        factor1.print("dopplerArm ")

    @unittest.skipUnless(hasattr(gtsam.DopplerFactorArm, "serialize"),
                         "Serialization not enabled")
    def test_serialization(self):
        sat_vel = np.array([100.0, 200.0, 300.0])
        lever = np.array([0.5, -0.3, 1.0])
        omega = np.array([0.02, 0.0, 0.1])
        factor = gtsam.DopplerFactorArm(0, 1, 2, 3, 10.0, LAMBDA_L1, SAT_POS,
                                        sat_vel, RCV_POS, lever, omega, 1.0, 0.0,
                                        self.model)
        factor.serialize()


if __name__ == "__main__":
    unittest.main()
