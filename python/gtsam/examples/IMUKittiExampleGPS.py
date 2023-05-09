"""
Example of application of ISAM2 for GPS-aided navigation on the KITTI VISION BENCHMARK SUITE

Author: Varun Agrawal
"""
import argparse
from typing import List, Tuple

import gtsam
import numpy as np
from gtsam import ISAM2, Pose3, noiseModel
from gtsam.symbol_shorthand import B, V, X

GRAVITY = 9.8


class KittiCalibration:
    """Class to hold KITTI calibration info."""
    def __init__(self, body_ptx: float, body_pty: float, body_ptz: float,
                 body_prx: float, body_pry: float, body_prz: float,
                 accelerometer_sigma: float, gyroscope_sigma: float,
                 integration_sigma: float, accelerometer_bias_sigma: float,
                 gyroscope_bias_sigma: float, average_delta_t: float):
        self.bodyTimu = Pose3(gtsam.Rot3.RzRyRx(body_prx, body_pry, body_prz),
                              gtsam.Point3(body_ptx, body_pty, body_ptz))
        self.accelerometer_sigma = accelerometer_sigma
        self.gyroscope_sigma = gyroscope_sigma
        self.integration_sigma = integration_sigma
        self.accelerometer_bias_sigma = accelerometer_bias_sigma
        self.gyroscope_bias_sigma = gyroscope_bias_sigma
        self.average_delta_t = average_delta_t


class ImuMeasurement:
    """An instance of an IMU measurement."""
    def __init__(self, time: float, dt: float, accelerometer: gtsam.Point3,
                 gyroscope: gtsam.Point3):
        self.time = time
        self.dt = dt
        self.accelerometer = accelerometer
        self.gyroscope = gyroscope


class GpsMeasurement:
    """An instance of a GPS measurement."""
    def __init__(self, time: float, position: gtsam.Point3):
        self.time = time
        self.position = position


def loadImuData(imu_data_file: str) -> List[ImuMeasurement]:
    """Helper to load the IMU data."""
    # Read IMU data
    # Time dt accelX accelY accelZ omegaX omegaY omegaZ
    imu_data_file = gtsam.findExampleDataFile(imu_data_file)
    imu_measurements = []

    print("-- Reading IMU measurements from file")
    with open(imu_data_file, encoding='UTF-8') as imu_data:
        data = imu_data.readlines()
        for i in range(1, len(data)):  # ignore the first line
            time, dt, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = map(
                float, data[i].split(' '))
            imu_measurement = ImuMeasurement(
                time, dt, np.asarray([acc_x, acc_y, acc_z]),
                np.asarray([gyro_x, gyro_y, gyro_z]))
            imu_measurements.append(imu_measurement)

    return imu_measurements


def loadGpsData(gps_data_file: str) -> List[GpsMeasurement]:
    """Helper to load the GPS data."""
    # Read GPS data
    # Time,X,Y,Z
    gps_data_file = gtsam.findExampleDataFile(gps_data_file)
    gps_measurements = []

    print("-- Reading GPS measurements from file")
    with open(gps_data_file, encoding='UTF-8') as gps_data:
        data = gps_data.readlines()
        for i in range(1, len(data)):
            time, x, y, z = map(float, data[i].split(','))
            gps_measurement = GpsMeasurement(time, np.asarray([x, y, z]))
            gps_measurements.append(gps_measurement)

    return gps_measurements


def loadKittiData(
    imu_data_file: str = "KittiEquivBiasedImu.txt",
    gps_data_file: str = "KittiGps_converted.txt",
    imu_metadata_file: str = "KittiEquivBiasedImu_metadata.txt"
) -> Tuple[KittiCalibration, List[ImuMeasurement], List[GpsMeasurement]]:
    """
    Load the KITTI Dataset.
    """
    # Read IMU metadata and compute relative sensor pose transforms
    # BodyPtx BodyPty BodyPtz BodyPrx BodyPry BodyPrz AccelerometerSigma
    # GyroscopeSigma IntegrationSigma AccelerometerBiasSigma GyroscopeBiasSigma
    # AverageDeltaT
    imu_metadata_file = gtsam.findExampleDataFile(imu_metadata_file)
    with open(imu_metadata_file, encoding='UTF-8') as imu_metadata:
        print("-- Reading sensor metadata")
        line = imu_metadata.readline()  # Ignore the first line
        line = imu_metadata.readline().strip()
        data = list(map(float, line.split(' ')))
        kitti_calibration = KittiCalibration(*data)
        print("IMU metadata:", data)

    imu_measurements = loadImuData(imu_data_file)
    gps_measurements = loadGpsData(gps_data_file)

    return kitti_calibration, imu_measurements, gps_measurements


def getImuParams(kitti_calibration: KittiCalibration):
    """Get the IMU parameters from the KITTI calibration data."""
    w_coriolis = np.zeros(3)

    # Set IMU preintegration parameters
    measured_acc_cov = np.eye(3) * np.power(
        kitti_calibration.accelerometer_sigma, 2)
    measured_omega_cov = np.eye(3) * np.power(
        kitti_calibration.gyroscope_sigma, 2)
    # error committed in integrating position from velocities
    integration_error_cov = np.eye(3) * np.power(
        kitti_calibration.integration_sigma, 2)

    imu_params = gtsam.PreintegrationParams.MakeSharedU(GRAVITY)
    # acc white noise in continuous
    imu_params.setAccelerometerCovariance(measured_acc_cov)
    # integration uncertainty continuous
    imu_params.setIntegrationCovariance(integration_error_cov)
    # gyro white noise in continuous
    imu_params.setGyroscopeCovariance(measured_omega_cov)
    imu_params.setOmegaCoriolis(w_coriolis)

    return imu_params


def save_results(isam: gtsam.ISAM2, output_filename: str, first_gps_pose: int,
                 gps_measurements: List[GpsMeasurement]):
    """Write the results from `isam` to `output_filename`."""
    # Save results to file
    print("Writing results to file...")
    with open(output_filename, 'w', encoding='UTF-8') as fp_out:
        fp_out.write(
            "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m)\n")

        result = isam.calculateEstimate()
        for i in range(first_gps_pose, len(gps_measurements)):
            pose_key = X(i)
            vel_key = V(i)
            bias_key = B(i)

            pose = result.atPose3(pose_key)
            velocity = result.atVector(vel_key)
            bias = result.atConstantBias(bias_key)

            pose_quat = pose.rotation().toQuaternion()
            gps = gps_measurements[i].position

            print(f"State at #{i}")
            print(f"Pose:\n{pose}")
            print(f"Velocity:\n{velocity}")
            print(f"Bias:\n{bias}")

            fp_out.write("{},{},{},{},{},{},{},{},{},{},{}\n".format(
                gps_measurements[i].time, pose.x(), pose.y(), pose.z(),
                pose_quat.x(), pose_quat.y(), pose_quat.z(), pose_quat.w(),
                gps[0], gps[1], gps[2]))


def parse_args() -> argparse.Namespace:
    """Parse command line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_filename",
                        default="IMUKittiExampleGPSResults.csv")
    return parser.parse_args()


def optimize(gps_measurements: List[GpsMeasurement],
             imu_measurements: List[ImuMeasurement],
             sigma_init_x: gtsam.noiseModel.Diagonal,
             sigma_init_v: gtsam.noiseModel.Diagonal,
             sigma_init_b: gtsam.noiseModel.Diagonal,
             noise_model_gps: gtsam.noiseModel.Diagonal,
             kitti_calibration: KittiCalibration, first_gps_pose: int,
             gps_skip: int) -> gtsam.ISAM2:
    """Run ISAM2 optimization on the measurements."""
    # Set initial conditions for the estimated trajectory
    # initial pose is the reference frame (navigation frame)
    current_pose_global = Pose3(gtsam.Rot3(),
                                gps_measurements[first_gps_pose].position)

    # the vehicle is stationary at the beginning at position 0,0,0
    current_velocity_global = np.zeros(3)
    current_bias = gtsam.imuBias.ConstantBias()  # init with zero bias

    imu_params = getImuParams(kitti_calibration)

    # Set ISAM2 parameters and create ISAM2 solver object
    isam_params = gtsam.ISAM2Params()
    isam_params.setFactorization("CHOLESKY")
    isam_params.setRelinearizeSkip(10)

    isam = gtsam.ISAM2(isam_params)

    # Create the factor graph and values object that will store new factors and
    # values to add to the incremental graph
    new_factors = gtsam.NonlinearFactorGraph()
    # values storing the initial estimates of new nodes in the factor graph
    new_values = gtsam.Values()

    # Main loop:
    # (1) we read the measurements
    # (2) we create the corresponding factors in the graph
    # (3) we solve the graph to obtain and optimal estimate of robot trajectory
    print("-- Starting main loop: inference is performed at each time step, "
          "but we plot trajectory every 10 steps")

    j = 0
    included_imu_measurement_count = 0

    for i in range(first_gps_pose, len(gps_measurements)):
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

            while (j < len(imu_measurements)
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
                gtsam.ImuFactor(previous_pose_key, previous_vel_key,
                                current_pose_key, current_vel_key,
                                previous_bias_key,
                                current_summarized_measurement))

            # Bias evolution as given in the IMU metadata
            sigma_between_b = gtsam.noiseModel.Diagonal.Sigmas(
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

                print(f"############ POSE INCLUDED AT TIME {t} ############")
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
                print(f"############ NEW FACTORS AT TIME {t:.6f} ############")
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

                print(f"############ POSE AT TIME {t} ############")
                current_pose_global.print()
                print("\n")

    return isam


def main():
    """Main runner."""
    args = parse_args()
    kitti_calibration, imu_measurements, gps_measurements = loadKittiData()

    if not kitti_calibration.bodyTimu.equals(Pose3(), 1e-8):
        raise ValueError(
            "Currently only support IMUinBody is identity, i.e. IMU and body frame are the same"
        )

    # Configure different variables
    first_gps_pose = 1
    gps_skip = 10

    # Configure noise models
    noise_model_gps = noiseModel.Diagonal.Precisions(
        np.asarray([0, 0, 0] + [1.0 / 0.07] * 3))

    sigma_init_x = noiseModel.Diagonal.Precisions(
        np.asarray([0, 0, 0, 1, 1, 1]))
    sigma_init_v = noiseModel.Diagonal.Sigmas(np.ones(3) * 1000.0)
    sigma_init_b = noiseModel.Diagonal.Sigmas(
        np.asarray([0.1] * 3 + [5.00e-05] * 3))

    isam = optimize(gps_measurements, imu_measurements, sigma_init_x,
                    sigma_init_v, sigma_init_b, noise_model_gps,
                    kitti_calibration, first_gps_pose, gps_skip)

    save_results(isam, args.output_filename, first_gps_pose, gps_measurements)


if __name__ == "__main__":
    main()
