"""
GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved

See LICENSE for the license information

Cal3Unified unit tests.
Author: Frank Dellaert & Duy Nguyen Ta (Python)
"""
import unittest

import numpy as np

import gtsam
import gtsam_unstable
from gtsam.utils.test_case import GtsamTestCase

class TestFixedLagSmootherExample(GtsamTestCase):
    '''
    Tests the fixed lag smoother wrapper
    '''

    def test_FixedLagSmootherExample(self):
        '''
        Simple test that checks for equality between C++ example
        file and the Python implementation. See
        gtsam_unstable/examples/FixedLagSmootherExample.cpp
        '''
        # Define a batch fixed lag smoother, which uses
        # Levenberg-Marquardt to perform the nonlinear optimization
        lag = 2.0
        smoother_batch = gtsam.BatchFixedLagSmoother(lag)

        # Create containers to store the factors and linearization points
        # that will be sent to the smoothers
        new_factors = gtsam.NonlinearFactorGraph()
        new_values = gtsam.Values()
        new_timestamps = gtsam.FixedLagSmootherKeyTimestampMap()

        # Create  a prior on the first pose, placing it at the origin
        prior_mean = gtsam.Pose2(0, 0, 0)
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([0.3, 0.3, 0.1]))
        X1 = 0
        new_factors.push_back(
            gtsam.PriorFactorPose2(
                X1, prior_mean, prior_noise))
        new_values.insert(X1, prior_mean)
        new_timestamps.insert((X1, 0.0))

        delta_time = 0.25
        time = 0.25

        i = 0

        ground_truth = [
            gtsam.Pose2(0.995821, 0.0231012, 0.0300001),
            gtsam.Pose2(1.49284, 0.0457247, 0.045),
            gtsam.Pose2(1.98981, 0.0758879, 0.06),
            gtsam.Pose2(2.48627, 0.113502, 0.075),
            gtsam.Pose2(2.98211, 0.158558, 0.09),
            gtsam.Pose2(3.47722, 0.211047, 0.105),
            gtsam.Pose2(3.97149, 0.270956, 0.12),
            gtsam.Pose2(4.4648, 0.338272, 0.135),
            gtsam.Pose2(4.95705, 0.41298, 0.15),
            gtsam.Pose2(5.44812, 0.495063, 0.165),
            gtsam.Pose2(5.9379, 0.584503, 0.18),
        ]

        # Iterates from 0.25s to 3.0s, adding 0.25s each loop
        # In each iteration, the agent moves at a constant speed
        # and its two odometers measure the change. The smoothed
        # result is then compared to the ground truth
        while time <= 3.0:
            previous_key = int(1000 * (time - delta_time))
            current_key = int(1000 * time)

            # assign current key to the current timestamp
            new_timestamps.insert((current_key, time))

            # Add a guess for this pose to the new values
            # Assume that the robot moves at 2 m/s. Position is time[s] *
            # 2[m/s]
            current_pose = gtsam.Pose2(time * 2, 0, 0)
            new_values.insert(current_key, current_pose)

            # Add odometry factors from two different sources with different
            # error stats
            odometry_measurement_1 = gtsam.Pose2(0.61, -0.08, 0.02)
            odometry_noise_1 = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.1, 0.1, 0.05]))
            new_factors.push_back(
                gtsam.BetweenFactorPose2(
                    previous_key,
                    current_key,
                    odometry_measurement_1,
                    odometry_noise_1))

            odometry_measurement_2 = gtsam.Pose2(0.47, 0.03, 0.01)
            odometry_noise_2 = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.05, 0.05, 0.05]))
            new_factors.push_back(
                gtsam.BetweenFactorPose2(
                    previous_key,
                    current_key,
                    odometry_measurement_2,
                    odometry_noise_2))

            # Update the smoothers with the new factors. In this case,
            # one iteration must pass for Levenberg-Marquardt to accurately
            # estimate
            if time >= 0.50:
                smoother_batch.update(new_factors, new_values, new_timestamps)

                estimate = smoother_batch.calculateEstimatePose2(current_key)
                self.assertTrue(estimate.equals(ground_truth[i], 1e-4))
                i += 1

                new_timestamps.clear()
                new_values.clear()
                new_factors.resize(0)

            time += delta_time


if __name__ == "__main__":
    unittest.main()
