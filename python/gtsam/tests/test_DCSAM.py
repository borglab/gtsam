"""
GTSAM Copyright 2010-2025, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Unit tests for DCSAM (Discrete-Continuous Smoothing And Mapping).
Author: Varun Agrawal
"""

# pylint: disable=invalid-name, no-name-in-module, no-member

import unittest

import numpy as np

from gtsam import (
    DCSAM,
    BetweenFactorPose2,
    ISAM2Params,
    DecisionTreeFactor,
    DiscreteValues,
    HybridNonlinearFactor,
    HybridNonlinearFactorGraph,
    HybridValues,
    Pose2,
    PriorFactorDouble,
    PriorFactorPose2,
    Values,
    VectorValues,
    noiseModel,
)
from gtsam.symbol_shorthand import D, X
from gtsam.utils.test_case import GtsamTestCase


class TestDCSAM(GtsamTestCase):
    """Unit tests for DCSAM."""

    def test_constructor(self):
        """Test DCSAM construction with default and custom ISAM2 params."""
        dcsam = DCSAM()
        self.assertIsInstance(dcsam, DCSAM)

        params = ISAM2Params()
        dcsam_with_params = DCSAM(params)
        self.assertIsInstance(dcsam_with_params, DCSAM)

    def test_simple_discrete(self):
        """Test DCSAM with only discrete variables.

        Creates a single discrete variable with two modes, adds a
        DecisionTreeFactor that strongly prefers mode 1, and verifies
        that DCSAM correctly infers mode 1 as the MPE.
        """
        dcsam = DCSAM()

        mode_key = D(1)
        mode = (mode_key, 2)

        # Discrete prior: weight 0.1 for mode 0, weight 0.9 for mode 1
        dpf = DecisionTreeFactor(mode, "0.1 0.9")
        hfg = HybridNonlinearFactorGraph()
        hfg.push_back(dpf)

        # Provide an initial discrete guess (mode 1 is preferred)
        initial_discrete = DiscreteValues()
        initial_discrete[mode_key] = 1

        dcsam.update(hfg, initial_discrete)
        dc_values = dcsam.calculateEstimate()

        # Mode 1 has a higher weight and should be the MPE
        self.assertEqual(dc_values.atDiscrete(mode_key), 1)

    def test_simple_mixture_factor(self):
        """Test DCSAM with a HybridNonlinearFactor (1D mixture model).

        Creates a hybrid factor with two components:
        - Mode 0: tight prior (sigma=1) → more informative
        - Mode 1: loose prior (sigma=8) → less informative

        The log-normalization constants ensure DCSAM selects the tighter
        mode (mode 0) as the most probable explanation.
        """
        mode_key = D(1)
        mode = (mode_key, 2)

        sigma_tight = 1.0
        sigma_loose = 8.0
        noise_tight = noiseModel.Isotropic.Sigma(1, sigma_tight)
        noise_loose = noiseModel.Isotropic.Sigma(1, sigma_loose)

        # Both priors have mean 0; mode 0 has tighter noise.
        f0 = PriorFactorDouble(X(1), 0.0, noise_tight)
        f1 = PriorFactorDouble(X(1), 0.0, noise_loose)

        # Include log-normalization constants so the discrete selection
        # accounts for the normalization of each Gaussian component.
        scalar0 = 0.5 * np.log(2 * np.pi * sigma_tight**2)
        scalar1 = 0.5 * np.log(2 * np.pi * sigma_loose**2)

        hybrid_factor = HybridNonlinearFactor(mode, [(f0, scalar0), (f1, scalar1)])

        hfg = HybridNonlinearFactorGraph()
        hfg.push_back(hybrid_factor)

        # Initial continuous guess away from the prior mean
        initial_continuous = Values()
        initial_continuous.insert(X(1), -2.5)

        initial_discrete = DiscreteValues()
        initial_discrete[mode_key] = 0

        initial_guess = HybridValues(
            VectorValues(), initial_discrete, initial_continuous
        )

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

        Constructs an 8-pose octagonal trajectory with odometry factors and
        verifies the optimized poses match the expected values from the
        reference DCSAM implementation.
        """
        prior_noise = noiseModel.Isotropic.Sigma(3, 0.1)
        meas_noise = noiseModel.Isotropic.Sigma(3, 1.0)

        pose0 = Pose2(0, 0, 0)
        dx = Pose2(1, 0, 0.78539816)
        noise_pose = Pose2(0.01, 0.01, 0.01)

        graph = HybridNonlinearFactorGraph()
        initial_continuous = Values()

        graph.push_back(PriorFactorPose2(X(0), pose0, prior_noise))
        initial_continuous.insert(X(0), pose0)

        odom = Pose2(pose0)
        for i in range(7):
            meas = dx.compose(noise_pose)
            graph.push_back(BetweenFactorPose2(X(i), X(i + 1), meas, meas_noise))
            odom = odom.compose(meas)
            initial_continuous.insert(X(i + 1), odom)

        dcsam = DCSAM()
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

    def test_simple_slam_incremental(self):
        """Test DCSAM on a Pose2 SLAM problem in incremental mode.

        Adds one pose at a time and verifies the final optimized trajectory
        matches the expected values from the reference DCSAM implementation.
        """
        prior_noise = noiseModel.Isotropic.Sigma(3, 0.1)
        meas_noise = noiseModel.Isotropic.Sigma(3, 1.0)

        pose0 = Pose2(0, 0, 0)
        dx = Pose2(1, 0, 0.78539816)
        noise_pose = Pose2(0.01, 0.01, 0.01)

        # --- Step 0: prior on the first pose ---
        graph = HybridNonlinearFactorGraph()
        initial_continuous = Values()

        graph.push_back(PriorFactorPose2(X(0), pose0, prior_noise))
        initial_continuous.insert(X(0), pose0)

        dcsam = DCSAM()
        initial_guess = HybridValues(
            VectorValues(), DiscreteValues(), initial_continuous
        )
        dcsam.update(graph, initial_guess)

        odom = Pose2(pose0)

        # --- Steps 1-7: add one odometry factor at a time ---
        for i in range(7):
            graph = HybridNonlinearFactorGraph()
            initial_continuous = Values()

            meas = dx.compose(noise_pose)
            graph.push_back(BetweenFactorPose2(X(i), X(i + 1), meas, meas_noise))

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
