"""
GTSAM Copyright 2010-2025, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for our implementation of DCSAM.
Original code: https://github.com/MarineRoboticsGroup/dcsam

Author: Varun Agrawal
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

from gtsam.symbol_shorthand import D, X
from gtsam.utils.test_case import GtsamTestCase

from gtsam import (
    DCSAM,
    BetweenFactorPose2,
    DiscreteValues,
    HybridNonlinearFactor,
    HybridNonlinearFactorGraph,
    HybridValues,
    ISAM2Params,
    Pose2,
    PriorFactorDouble,
    PriorFactorPose2,
    Values,
    VectorValues,
    noiseModel,
)


class TestDCSAM(GtsamTestCase):
    """Unit tests for DCSAM."""

    def setUp(self):
        """Set up fixtures."""
        ## For SLAM tests.
        # Noise models
        self.prior_noise = noiseModel.Isotropic.Sigma(3, 0.1)
        self.meas_noise = noiseModel.Isotropic.Sigma(3, 1.0)

        # Prior pose
        self.pose0 = Pose2(0, 0, 0)
        # The pose delta to create an octagonal trajectory
        self.dx = Pose2(1, 0, 0.78539816)
        # The noise to add to each odometry measurement
        self.noise_pose = Pose2(0.01, 0.01, 0.01)

    def test_constructor(self):
        """Test DCSAM construction with default and custom ISAM2 params."""
        dcsam = DCSAM()
        self.assertIsInstance(dcsam, DCSAM)

        params = ISAM2Params()
        dcsam_with_params = DCSAM(params)
        self.assertIsInstance(dcsam_with_params, DCSAM)

    def test_simple_mixture_factor(self):
        """Test DCSAM with a HybridNonlinearFactor (1D mixture model).

        Creates a hybrid factor with two components:
        - Mode 0: tight prior (sigma=1) → more informative
        - Mode 1: loose prior (sigma=8) → less informative

        The log-normalization constants ensure DCSAM selects the tighter
        mode (mode 0) as the most probable explanation.
        """
        mode_key = D(1)
        dk = (mode_key, 2)

        # Both priors have mean 0; mode 0 has tighter noise.
        sigma_tight = 1.0
        noise_tight = noiseModel.Isotropic.Sigma(1, sigma_tight)
        f0 = PriorFactorDouble(X(1), 0.0, noise_tight)

        sigma_loose = 8.0
        noise_loose = noiseModel.Isotropic.Sigma(1, sigma_loose)
        f1 = PriorFactorDouble(X(1), 0.0, noise_loose)

        mixture_factor = HybridNonlinearFactor(
            dk,
            [(f0, noise_tight.negLogConstant()), (f1, noise_loose.negLogConstant())],
        )

        hfg = HybridNonlinearFactorGraph()
        hfg.push_back(mixture_factor)

        # Initial continuous guess away from the prior mean
        initial_continuous = Values()
        initial_continuous.insert(X(1), -2.5)

        initial_guess_discrete = DiscreteValues()
        initial_guess_discrete[mode_key] = 0

        initial_guess = HybridValues(initial_guess_discrete, initial_continuous)

        dcsam = DCSAM()
        dcsam.update(hfg, initial_guess)
        dc_values = dcsam.calculateEstimate()

        # Run one more iteration to refine the estimate
        dcsam.update()
        dc_values = dcsam.calculateEstimate()

        # Mode 0 (tighter prior, lower neg-log-constant) should be preferred
        self.assertEqual(dc_values.atDiscrete(mode_key), 0)

    def test_simple_slam_batch(self):
        """Test DCSAM on a Pose2 SLAM problem in batch mode.

        This is a basic octagonal pose graph SLAM test
        to verify that DCSAM works on standard SLAM examples.
        """

        graph = HybridNonlinearFactorGraph()
        initial_guess_continuous = Values()

        graph.push_back(PriorFactorPose2(X(0), self.pose0, self.prior_noise))
        initial_guess_continuous.insert(X(0), self.pose0)

        odom = Pose2(self.pose0)
        for i in range(7):
            meas = self.dx.compose(self.noise_pose)
            graph.push_back(BetweenFactorPose2(X(i), X(i + 1), meas, self.meas_noise))
            odom = odom.compose(meas)
            initial_guess_continuous.insert(X(i + 1), odom)

        dcsam = DCSAM()
        initial_guess = HybridValues(DiscreteValues(), initial_guess_continuous)

        dcsam.update(graph, initial_guess)
        dc_values = dcsam.calculateEstimate()

        # Expected values from the reference DCSAM C++ implementation
        expected = Values()
        expected.insert(X(0), Pose2(-3.53708e-33, -8.97559e-34, 4.1955e-33))
        expected.insert(X(1), Pose2(1, 0.0141421, 0.795398))
        expected.insert(X(2), Pose2(1.6899, 0.738184, 1.5908))
        expected.insert(X(3), Pose2(1.65576, 1.7377, 2.38619))
        expected.insert(X(4), Pose2(0.918069, 2.41298, -3.10159))
        expected.insert(X(5), Pose2(-0.0805657, 2.35886, -2.30619))
        expected.insert(X(6), Pose2(-0.740961, 1.60781, -1.5108))
        expected.insert(X(7), Pose2(-0.66688, 0.61046, -0.715398))

        self.gtsamAssertEquals(dc_values.nonlinear(), expected, 1e-5)

    def test_simple_slam_incremental(self):
        """Test DCSAM on a Pose2 SLAM problem in incremental mode.

        Basic qualitative test on an octagonal pose graph SLAM example similar to the batch setting above, but in *incremental* mode.
        """

        # --- Step 0: prior on the first pose ---
        graph = HybridNonlinearFactorGraph()
        initial_continuous = Values()

        graph.push_back(PriorFactorPose2(X(0), self.pose0, self.prior_noise))
        initial_continuous.insert(X(0), self.pose0)

        dcsam = DCSAM()
        initial_guess = HybridValues(DiscreteValues(), initial_continuous)

        dcsam.update(graph, initial_guess)

        # --- Steps 1-7: add one odometry factor at a time ---
        odom = Pose2(self.pose0)

        for i in range(7):
            graph = HybridNonlinearFactorGraph()
            initial_continuous = Values()

            meas = self.dx.compose(self.noise_pose)
            graph.push_back(BetweenFactorPose2(X(i), X(i + 1), meas, self.meas_noise))

            odom = odom.compose(meas)
            initial_continuous.insert(X(i + 1), odom)

            initial_guess = HybridValues(
                VectorValues(), DiscreteValues(), initial_continuous
            )
            dcsam.update(graph, initial_guess)

        dc_values = dcsam.calculateEstimate()

        # Expected values from the reference DCSAM C++ implementation
        expected = Values()
        expected.insert(X(0), Pose2(-3.53708e-33, -8.97559e-34, 4.1955e-33))
        expected.insert(X(1), Pose2(1, 0.0141421, 0.795398))
        expected.insert(X(2), Pose2(1.6899, 0.738184, 1.5908))
        expected.insert(X(3), Pose2(1.65576, 1.7377, 2.38619))
        expected.insert(X(4), Pose2(0.918069, 2.41298, -3.10159))
        expected.insert(X(5), Pose2(-0.0805657, 2.35886, -2.30619))
        expected.insert(X(6), Pose2(-0.740961, 1.60781, -1.5108))
        expected.insert(X(7), Pose2(-0.66688, 0.61046, -0.715398))

        self.gtsamAssertEquals(dc_values.nonlinear(), expected, 1e-5)


if __name__ == "__main__":
    unittest.main()
