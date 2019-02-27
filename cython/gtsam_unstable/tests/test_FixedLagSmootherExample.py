import unittest
import gtsam
import gtsam_unstable
import numpy as np

def _timestamp_key_value(key, value):
    return gtsam_unstable.FixedLagSmootherKeyTimestampMapValue(
        key, value
    )
class TestFixedLagSmootherExample(unittest.TestCase):
    # Simple test that checks for equality between C++ example
    # file and the Python implementation
    def test_FixedLagSmootherExample(self):
        # Define a batch fixed lag smoother, which uses
        # Levenberg-Marquardt to perform the nonlinear optimization
        lag = 2.0
        smoother_batch = gtsam_unstable.BatchFixedLagSmoother(lag)


        # Create containers to store the factors and linearization points
        # that will be sent to the smoothers
        new_factors = gtsam.NonlinearFactorGraph()
        new_values = gtsam.Values()
        new_timestamps = gtsam_unstable.FixedLagSmootherKeyTimestampMap()


        # Create  a prior on the first pose, placing it at the origin
        prior_mean = gtsam.Pose2(0, 0, 0)
        prior_noise = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.3, 0.3, 0.1]))
        X1 = 0
        new_factors.push_back(gtsam.PriorFactorPose2(X1, prior_mean, prior_noise))
        new_values.insert(X1, prior_mean)
        new_timestamps.insert(_timestamp_key_value(X1, 0.0))

        delta_time = 0.25
        time = 0.25

        i = 0

        ground_truth = [
            gtsam.Pose2(0.49792, 0.007802, 0.015),
            gtsam.Pose2(0.99547, 0.023019, 0.03),
            gtsam.Pose2(1.4928, 0.045725, 0.045),
            gtsam.Pose2(1.9898, 0.075888, 0.06),
            gtsam.Pose2(2.4863, 0.1135, 0.075),
            gtsam.Pose2(2.9821, 0.15856, 0.09),
            gtsam.Pose2(3.4772, 0.21105, 0.105),
            gtsam.Pose2(3.9715, 0.27096, 0.12),
            gtsam.Pose2(4.4648, 0.33827, 0.135),
            gtsam.Pose2(4.957, 0.41298, 0.15),
            gtsam.Pose2(5.4481, 0.49506, 0.165),
            gtsam.Pose2(5.9379, 0.5845, 0.18),
        ]
        while time <= 3.0:
            previous_key = 1000 * (time - delta_time)
            current_key = 1000 * time

            # assign current key to the current timestamp
            new_timestamps.insert(_timestamp_key_value(current_key, time))

            # Add a guess for this pose to the new values
            # Assume that the robot moves at 2 m/s. Position is time[s] * 2[m/s]
            current_pose = gtsam.Pose2(time * 2, 0, 0)
            new_values.insert(current_key, current_pose)

            # Add odometry factors from two different sources with different error stats
            odometry_measurement_1 = gtsam.Pose2(0.61, -0.08, 0.02)
            odometry_noise_1 = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
            new_factors.push_back(gtsam.BetweenFactorPose2(
                previous_key, current_key, odometry_measurement_1, odometry_noise_1
            ))

            odometry_measurement_2 = gtsam.Pose2(0.47, 0.03, 0.01)
            odometry_noise_2 = gtsam.noiseModel_Diagonal.Sigmas(np.array([0.05, 0.05, 0.05]))
            new_factors.push_back(gtsam.BetweenFactorPose2(
                previous_key, current_key, odometry_measurement_2, odometry_noise_2
            ))

            # Update the smoothers with the new factors
            smoother_batch.update(new_factors, new_values, new_timestamps)

            estimate = smoother_batch.calculateEstimatePose2(current_key)
            self.assertTrue(estimate.equals(ground_truth[i], 1e-4))
            print("PASS")

            new_timestamps.clear()
            new_values.clear()
            new_factors.resize(0)

            time += delta_time
            i += 1

if __name__ == "__main__":
    unittest.main()
