"""
Example of application of ISAM2 for GPS-aided navigation on the KITTI VISION BENCHMARK SUITE
"""
import argparse
from typing import List

import gtsam
import numpy as np
from gtsam import Pose3, Rot3, noiseModel
from gtsam.symbol_shorthand import B, V, X


class KittiCalibration:
    def __init__(self, bodyTimu: gtsam.Pose3):
        self.bodyTimu = bodyTimu


class ImuMeasurement:
    def __init__(self, time, dt, accelerometer, gyroscope):
        pass


class GpsMeasurement:
    def __init__(self, time, position: gtsam.Point3):
        self.time = time
        self.position = position


def lodKittiData():
    pass


def parse_args():
    parser = argparse.ArgumentParser()
    return parser.parse_args()


def getImuParams(kitti_calibration):
    GRAVITY = 9.8
    w_coriolis = np.zeros(3)

    # Set IMU preintegration parameters
    measured_acc_cov = np.eye(3) * np.power(
        kitti_calibration.accelerometer_sigma, 2)
    measured_omega_cov = np.eye(3) * np.powwe(
        kitti_calibration.gyroscope_sigma, 2)
    # error committed in integrating position from velocities
    integration_error_cov = np.eye(3) * np.power(
        kitti_calibration.integration_sigma, 2)

    imu_params = gtsam.PreintegrationParams.MakeSharedU(GRAVITY)
    imu_params.accelerometerCovariance = measured_acc_cov  # acc white noise in continuous
    imu_params.integrationCovariance = integration_error_cov  # integration uncertainty continuous
    imu_params.gyroscopeCovariance = measured_omega_cov  # gyro white noise in continuous
    imu_params.omegaCoriolis = w_coriolis

    return imu_params


def main():
    args = parse_args()
    kitti_calibration, imu_measurements, gps_measurements = lodKittiData()

    if kitti_calibration.bodyTimu != gtsam.Pose3:
        raise ValueError(
            "Currently only support IMUinBody is identity, i.e. IMU and body frame are the same"
        )

    # Configure different variables
    first_gps_pose = 1
    gps_skip = 10

    # Configure noise models
    noise_model_gps = noiseModel.Diagonal.Precisions(
        np.asarray([0, 0, 0, 1.0 / 0.07, 1.0 / 0.07, 1.0 / 0.07]))

    # Set initial conditions for the estimated trajectory
    # initial pose is the reference frame (navigation frame)
    current_pose_global = Pose3(Rot3(),
                                gps_measurements[first_gps_pose].position)
    # the vehicle is stationary at the beginning at position 0,0,0
    current_velocity_global = np.zeros(3)
    current_bias = gtsam.imuBias.ConstantBias()  # init with zero bias

    sigma_init_x = noiseModel.Diagonal.Precisions(
        np.asarray([0, 0, 0, 1, 1, 1]))
    sigma_init_v = noiseModel.Diagonal.Sigmas(np.ones(3) * 1000.0)
    sigma_init_b = noiseModel.Diagonal.Sigmas(
        np.asarray([0.1, 0.1, 0.1, 5.00e-05, 5.00e-05, 5.00e-05]))

    imu_params = getImuParams()

    # Set ISAM2 parameters and create ISAM2 solver object
    isam_params = gtsam.ISAM2Params()
    isam_params.setFactorization("CHOLESKY")
    isam_params.setRelinearizeSkip(10)

    isam = gtsam.ISAM2(isam_params)

    # Create the factor graph and values object that will store new factors and
    # values to add to the incremental graph
    new_factors = gtsam.NonlinearFactorGraph()
    new_values = gtsam.Values(
    )  # values storing the initial estimates of new nodes in the factor graph

    # Main loop:
    # (1) we read the measurements
    # (2) we create the corresponding factors in the graph
    # (3) we solve the graph to obtain and optimal estimate of robot trajectory
    print(
        "-- Starting main loop: inference is performed at each time step, but we plot trajectory every 10 steps\n"
    )
    j = 0
    for i in range(first_gps_pose, len(gps_measurements) - 1):
        # At each non=IMU measurement we initialize a new node in the graph
        current_pose_key = X(i)
        current_vel_key = V(i)
        current_bias_key = B(i)
        t = gps_measurements[i].time

        if i == first_gps_pose:
            # Create initial estimate and prior on initial pose, velocity, and biases
            new_values.insert(current_pose_key, current_pose_global)
            new_values.insert(current_vel_key, current_velocity_global)
            new_values.insert(current_bias_key, current_bias)

            new_factors.addPriorPose3(current_pose_key, current_pose_global,
                                      sigma_init_x)
            new_factors.addPriorVector(current_vel_key,
                                       current_velocity_global, sigma_init_v)
            new_factors.addPriorConstantBias(current_bias_key, current_bias,
                                             sigma_init_b)
        else:
            t_previous = gps_measurements[i - 1].time

            # Summarize IMU data between the previous GPS measurement and now
            current_summarized_measurement = gtsam.PreintegratedImuMeasurements(
                imu_params, current_bias)

            included_imu_measurement_count = 0
            while (j < imu_measurements.size()
                   and imu_measurements[j].time <= t):
                if imu_measurements[j].time >= t_previous:
                    current_summarized_measurement.integrateMeasurement(
                        imu_measurements[j].accelerometer,
                        imu_measurements[j].gyroscope, imu_measurements[j].dt)
                    included_imu_measurement_count += 1
                j += 1

            # Create IMU factor
            previous_pose_key = X(i - 1)
            previous_vel_key = V(i - 1)
            previous_bias_key = B(i - 1)

            new_factors.push_back(
                gtsam.ImuFactor > (previous_pose_key, previous_vel_key,
                                   current_pose_key, current_vel_key,
                                   previous_bias_key,
                                   current_summarized_measurement))

            # Bias evolution as given in the IMU metadata
            sigma_between_b = noiseModel.Diagonal.Sigmas(
                np.asarray([
                    np.sqrt(included_imu_measurement_count) *
                    kitti_calibration.accelerometer_bias_sigma
                ] * 3 + [
                    np.sqrt(included_imu_measurement_count) *
                    kitti_calibration.gyroscope_bias_sigma
                ] * 3))

            new_factors.push_back(
                gtsam.BetweenFactorConstantBias(previous_bias_key,
                                                current_bias_key,
                                                gtsam.imuBias.ConstantBias(),
                                                sigma_between_b))

            # Create GPS factor
            gps_pose = Pose3(current_pose_global.rotation(),
                             gps_measurements[i].position)
            if (i % gps_skip) == 0:
                new_factors.addPriorPose3(current_pose_key, gps_pose,
                                          noise_model_gps)
                new_values.insert(current_pose_key, gps_pose)

                print(
                    "################ POSE INCLUDED AT TIME %lf ################\n",
                    t)
                print(gps_pose.translation(), "\n")
            else:
                new_values.insert(current_pose_key, current_pose_global)

            # Add initial values for velocity and bias based on the previous
            # estimates
            new_values.insert(current_vel_key, current_velocity_global)
            new_values.insert(current_bias_key, current_bias)

            # Update solver
            # =======================================================================
            # We accumulate 2*GPSskip GPS measurements before updating the solver at
            # first so that the heading becomes observable.
            if i > (first_gps_pose + 2 * gps_skip):
                print(
                    "################ NEW FACTORS AT TIME %lf ################\n",
                    t)
                new_factors.print()

                isam.update(new_factors, new_values)

                # Reset the newFactors and newValues list
                new_factors.resize(0)
                new_values.clear()

                # Extract the result/current estimates
                result = isam.calculateEstimate()

                current_pose_global = result.atPose3(current_pose_key)
                current_velocity_global = result.atVector(current_vel_key)
                current_bias = result.atConstantBias(current_bias_key)

                print("\n################ POSE AT TIME %lf ################\n",
                      t)
                current_pose_global.print()
                print("\n\n")


if __name__ == "__main__":
    main()
