/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IMUKittiExampleGPS
 * @brief Example of application of ISAM2 for GPS-aided navigation on the KITTI
 * VISION BENCHMARK SUITE
 * @author Ported by Thomas Jespersen (thomasj@tkjelectronics.dk), TKJ
 * Electronics
 */

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

struct KittiCalibration {
  double body_ptx;
  double body_pty;
  double body_ptz;
  double body_prx;
  double body_pry;
  double body_prz;
  double accelerometer_sigma;
  double gyroscope_sigma;
  double integration_sigma;
  double accelerometer_bias_sigma;
  double gyroscope_bias_sigma;
  double average_delta_t;
};

struct ImuMeasurement {
  double time;
  double dt;
  Vector3 accelerometer;
  Vector3 gyroscope;  // omega
};

struct GpsMeasurement {
  double time;
  Vector3 position;  // x,y,z
};

const string output_filename = "IMUKittiExampleGPSResults.csv";

void loadKittiData(KittiCalibration& kitti_calibration,
                   vector<ImuMeasurement>& imu_measurements,
                   vector<GpsMeasurement>& gps_measurements) {
  string line;

  // Read IMU metadata and compute relative sensor pose transforms
  // BodyPtx BodyPty BodyPtz BodyPrx BodyPry BodyPrz AccelerometerSigma
  // GyroscopeSigma IntegrationSigma AccelerometerBiasSigma GyroscopeBiasSigma
  // AverageDeltaT
  string imu_metadata_file =
      findExampleDataFile("KittiEquivBiasedImu_metadata.txt");
  ifstream imu_metadata(imu_metadata_file.c_str());

  printf("-- Reading sensor metadata\n");

  getline(imu_metadata, line, '\n');  // ignore the first line

  // Load Kitti calibration
  getline(imu_metadata, line, '\n');
  sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
         &kitti_calibration.body_ptx, &kitti_calibration.body_pty,
         &kitti_calibration.body_ptz, &kitti_calibration.body_prx,
         &kitti_calibration.body_pry, &kitti_calibration.body_prz,
         &kitti_calibration.accelerometer_sigma,
         &kitti_calibration.gyroscope_sigma,
         &kitti_calibration.integration_sigma,
         &kitti_calibration.accelerometer_bias_sigma,
         &kitti_calibration.gyroscope_bias_sigma,
         &kitti_calibration.average_delta_t);
  printf("IMU metadata: %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
         kitti_calibration.body_ptx, kitti_calibration.body_pty,
         kitti_calibration.body_ptz, kitti_calibration.body_prx,
         kitti_calibration.body_pry, kitti_calibration.body_prz,
         kitti_calibration.accelerometer_sigma,
         kitti_calibration.gyroscope_sigma, kitti_calibration.integration_sigma,
         kitti_calibration.accelerometer_bias_sigma,
         kitti_calibration.gyroscope_bias_sigma,
         kitti_calibration.average_delta_t);

  // Read IMU data
  // Time dt accelX accelY accelZ omegaX omegaY omegaZ
  string imu_data_file = findExampleDataFile("KittiEquivBiasedImu.txt");
  printf("-- Reading IMU measurements from file\n");
  {
    ifstream imu_data(imu_data_file.c_str());
    getline(imu_data, line, '\n');  // ignore the first line

    double time = 0, dt = 0, acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0,
           gyro_y = 0, gyro_z = 0;
    while (!imu_data.eof()) {
      getline(imu_data, line, '\n');
      sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf", &time, &dt,
             &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);

      ImuMeasurement measurement;
      measurement.time = time;
      measurement.dt = dt;
      measurement.accelerometer = Vector3(acc_x, acc_y, acc_z);
      measurement.gyroscope = Vector3(gyro_x, gyro_y, gyro_z);
      imu_measurements.push_back(measurement);
    }
  }

  // Read GPS data
  // Time,X,Y,Z
  string gps_data_file = findExampleDataFile("KittiGps_converted.txt");
  printf("-- Reading GPS measurements from file\n");
  {
    ifstream gps_data(gps_data_file.c_str());
    getline(gps_data, line, '\n');  // ignore the first line

    double time = 0, gps_x = 0, gps_y = 0, gps_z = 0;
    while (!gps_data.eof()) {
      getline(gps_data, line, '\n');
      sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &time, &gps_x, &gps_y, &gps_z);

      GpsMeasurement measurement;
      measurement.time = time;
      measurement.position = Vector3(gps_x, gps_y, gps_z);
      gps_measurements.push_back(measurement);
    }
  }
}

int main(int argc, char* argv[]) {
  KittiCalibration kitti_calibration;
  vector<ImuMeasurement> imu_measurements;
  vector<GpsMeasurement> gps_measurements;
  loadKittiData(kitti_calibration, imu_measurements, gps_measurements);

  Vector6 BodyP =
      (Vector6() << kitti_calibration.body_ptx, kitti_calibration.body_pty,
       kitti_calibration.body_ptz, kitti_calibration.body_prx,
       kitti_calibration.body_pry, kitti_calibration.body_prz)
          .finished();
  auto body_T_imu = Pose3::Expmap(BodyP);
  if (!body_T_imu.equals(Pose3(), 1e-5)) {
    printf(
        "Currently only support IMUinBody is identity, i.e. IMU and body frame "
        "are the same");
    exit(-1);
  }

  // Configure different variables
  // double t_offset = gps_measurements[0].time;
  size_t first_gps_pose = 1;
  size_t gps_skip = 10;  // Skip this many GPS measurements each time
  double g = 9.8;
  auto w_coriolis = Vector3::Zero();  // zero vector

  // Configure noise models
  auto noise_model_gps = noiseModel::Diagonal::Precisions(
      (Vector6() << Vector3::Constant(0), Vector3::Constant(1.0 / 0.07))
          .finished());

  // Set initial conditions for the estimated trajectory
  // initial pose is the reference frame (navigation frame)
  auto current_pose_global =
      Pose3(Rot3(), gps_measurements[first_gps_pose].position);
  // the vehicle is stationary at the beginning at position 0,0,0
  Vector3 current_velocity_global = Vector3::Zero();
  auto current_bias = imuBias::ConstantBias();  // init with zero bias

  auto sigma_init_x = noiseModel::Diagonal::Precisions(
      (Vector6() << Vector3::Constant(0), Vector3::Constant(1.0)).finished());
  auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000.0));
  auto sigma_init_b = noiseModel::Diagonal::Sigmas(
      (Vector6() << Vector3::Constant(0.100), Vector3::Constant(5.00e-05))
          .finished());

  // Set IMU preintegration parameters
  Matrix33 measured_acc_cov =
      I_3x3 * pow(kitti_calibration.accelerometer_sigma, 2);
  Matrix33 measured_omega_cov =
      I_3x3 * pow(kitti_calibration.gyroscope_sigma, 2);
  // error committed in integrating position from velocities
  Matrix33 integration_error_cov =
      I_3x3 * pow(kitti_calibration.integration_sigma, 2);

  auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
  imu_params->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  imu_params->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  imu_params->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  imu_params->omegaCoriolis = w_coriolis;

  std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement =
      nullptr;

  // Set ISAM2 parameters and create ISAM2 solver object
  ISAM2Params isam_params;
  isam_params.factorization = ISAM2Params::CHOLESKY;
  isam_params.relinearizeSkip = 10;

  ISAM2 isam(isam_params);

  // Create the factor graph and values object that will store new factors and
  // values to add to the incremental graph
  NonlinearFactorGraph new_factors;
  Values new_values;  // values storing the initial estimates of new nodes in
                      // the factor graph

  /// Main loop:
  /// (1) we read the measurements
  /// (2) we create the corresponding factors in the graph
  /// (3) we solve the graph to obtain and optimal estimate of robot trajectory
  printf(
      "-- Starting main loop: inference is performed at each time step, but we "
      "plot trajectory every 10 steps\n");
  size_t j = 0;

  for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; i++) {
    // At each non=IMU measurement we initialize a new node in the graph
    auto current_pose_key = X(i);
    auto current_vel_key = V(i);
    auto current_bias_key = B(i);
    double t = gps_measurements[i].time;
    size_t included_imu_measurement_count = 0;

    if (i == first_gps_pose) {
      // Create initial estimate and prior on initial pose, velocity, and biases
      new_values.insert(current_pose_key, current_pose_global);
      new_values.insert(current_vel_key, current_velocity_global);
      new_values.insert(current_bias_key, current_bias);
      new_factors.emplace_shared<PriorFactor<Pose3>>(
          current_pose_key, current_pose_global, sigma_init_x);
      new_factors.emplace_shared<PriorFactor<Vector3>>(
          current_vel_key, current_velocity_global, sigma_init_v);
      new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
          current_bias_key, current_bias, sigma_init_b);
    } else {
      double t_previous = gps_measurements[i - 1].time;

      // Summarize IMU data between the previous GPS measurement and now
      current_summarized_measurement =
          std::make_shared<PreintegratedImuMeasurements>(imu_params,
                                                         current_bias);

      while (j < imu_measurements.size() && imu_measurements[j].time <= t) {
        if (imu_measurements[j].time >= t_previous) {
          current_summarized_measurement->integrateMeasurement(
              imu_measurements[j].accelerometer, imu_measurements[j].gyroscope,
              imu_measurements[j].dt);
          included_imu_measurement_count++;
        }
        j++;
      }

      // Create IMU factor
      auto previous_pose_key = X(i - 1);
      auto previous_vel_key = V(i - 1);
      auto previous_bias_key = B(i - 1);

      new_factors.emplace_shared<ImuFactor>(
          previous_pose_key, previous_vel_key, current_pose_key,
          current_vel_key, previous_bias_key, *current_summarized_measurement);

      // Bias evolution as given in the IMU metadata
      auto sigma_between_b = noiseModel::Diagonal::Sigmas(
          (Vector6() << Vector3::Constant(
               sqrt(included_imu_measurement_count) *
               kitti_calibration.accelerometer_bias_sigma),
           Vector3::Constant(sqrt(included_imu_measurement_count) *
                             kitti_calibration.gyroscope_bias_sigma))
              .finished());
      new_factors.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
          previous_bias_key, current_bias_key, imuBias::ConstantBias(),
          sigma_between_b);

      // Create GPS factor
      auto gps_pose =
          Pose3(current_pose_global.rotation(), gps_measurements[i].position);
      if ((i % gps_skip) == 0) {
        new_factors.emplace_shared<PriorFactor<Pose3>>(
            current_pose_key, gps_pose, noise_model_gps);
        new_values.insert(current_pose_key, gps_pose);

        printf("############ POSE INCLUDED AT TIME %.6lf ############\n",
               t);
        cout << gps_pose.translation();
        printf("\n\n");
      } else {
        new_values.insert(current_pose_key, current_pose_global);
      }

      // Add initial values for velocity and bias based on the previous
      // estimates
      new_values.insert(current_vel_key, current_velocity_global);
      new_values.insert(current_bias_key, current_bias);

      // Update solver
      // =======================================================================
      // We accumulate 2*GPSskip GPS measurements before updating the solver at
      // first so that the heading becomes observable.
      if (i > (first_gps_pose + 2 * gps_skip)) {
        printf("############ NEW FACTORS AT TIME %.6lf ############\n",
               t);
        new_factors.print();

        isam.update(new_factors, new_values);

        // Reset the newFactors and newValues list
        new_factors.resize(0);
        new_values.clear();

        // Extract the result/current estimates
        Values result = isam.calculateEstimate();

        current_pose_global = result.at<Pose3>(current_pose_key);
        current_velocity_global = result.at<Vector3>(current_vel_key);
        current_bias = result.at<imuBias::ConstantBias>(current_bias_key);

        printf("\n############ POSE AT TIME %lf ############\n", t);
        current_pose_global.print();
        printf("\n\n");
      }
    }
  }

  // Save results to file
  printf("\nWriting results to file...\n");
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  fprintf(fp_out,
          "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m)\n");

  Values result = isam.calculateEstimate();
  for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; i++) {
    auto pose_key = X(i);
    auto vel_key = V(i);
    auto bias_key = B(i);

    auto pose = result.at<Pose3>(pose_key);
    auto velocity = result.at<Vector3>(vel_key);
    auto bias = result.at<imuBias::ConstantBias>(bias_key);

    auto pose_quat = pose.rotation().toQuaternion();
    auto gps = gps_measurements[i].position;

    cout << "State at #" << i << endl;
    cout << "Pose:" << endl << pose << endl;
    cout << "Velocity:" << endl << velocity << endl;
    cout << "Bias:" << endl << bias << endl;

    fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
            gps_measurements[i].time, pose.x(), pose.y(), pose.z(),
            pose_quat.x(), pose_quat.y(), pose_quat.z(), pose_quat.w(), gps(0),
            gps(1), gps(2));
  }

  fclose(fp_out);
}
