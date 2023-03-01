/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CombinedImuFactorsExample
 * @brief Test example for using GTSAM ImuCombinedFactor
 * navigation code.
 * @author Varun Agrawal
 */

/**
 * Example of use of the CombinedImuFactor in
 * conjunction with GPS
 *  - we read IMU and GPS data from a CSV file, with the following format:
 *  A row starting with "i" is the first initial position formatted with
 *  N, E, D, qx, qY, qZ, qW, velN, velE, velD
 *  A row starting with "0" is an imu measurement
 *  (body frame - Forward, Right, Down)
 *  linAccX, linAccY, linAccZ, angVelX, angVelY, angVelX
 *  A row starting with "1" is a gps correction formatted with
 *  N, E, D, qX, qY, qZ, qW
 * Note that for GPS correction, we're only using the position not the
 * rotation. The rotation is provided in the file for ground truth comparison.
 *
 *  See usage: ./CombinedImuFactorsExample --help
 */

#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <cstring>
#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace po = boost::program_options;

po::variables_map parseOptions(int argc, char* argv[]) {
  po::options_description desc;
  desc.add_options()("help,h", "produce help message")  // help message
      ("data_csv_path", po::value<string>()->default_value("imuAndGPSdata.csv"),
       "path to the CSV file with the IMU data")  // path to the data file
      ("output_filename",
       po::value<string>()->default_value("imuFactorExampleResults.csv"),
       "path to the result file to use")  // filename to save results to
      ("use_isam", po::bool_switch(),
       "use ISAM as the optimizer");  // flag for ISAM optimizer

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
    cout << desc << "\n";
    exit(1);
  }

  return vm;
}

Vector10 readInitialState(ifstream& file) {
  string value;
  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Vector10 initial_state;
  getline(file, value, ',');  // i
  for (int i = 0; i < 9; i++) {
    getline(file, value, ',');
    initial_state(i) = stof(value.c_str());
  }
  getline(file, value, '\n');
  initial_state(9) = stof(value.c_str());

  return initial_state;
}

std::shared_ptr<PreintegratedCombinedMeasurements::Params> imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * 1e-8;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_init =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}

int main(int argc, char* argv[]) {
  string data_filename, output_filename;
  po::variables_map var_map = parseOptions(argc, argv);

  data_filename = findExampleDataFile(var_map["data_csv_path"].as<string>());
  output_filename = var_map["output_filename"].as<string>();

  // Set up output file for plotting errors
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  fprintf(fp_out,
          "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,"
          "gt_qy,gt_qz,gt_qw\n");

  // Begin parsing the CSV file.  Input the first line for initialization.
  // From there, we'll iterate through the file and we'll preintegrate the IMU
  // or add in the GPS given the input.
  ifstream file(data_filename.c_str());

  Vector10 initial_state = readInitialState(file);
  cout << "initial state:\n" << initial_state.transpose() << "\n\n";

  // Assemble initial quaternion through GTSAM constructor
  // ::Quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
                                         initial_state(4), initial_state(5));
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());

  imuBias::ConstantBias prior_imu_bias;  // assume zero initial bias

  int index = 0;

  Values initial_values;

  // insert pose at initialization
  initial_values.insert(X(index), prior_pose);
  initial_values.insert(V(index), prior_velocity);
  initial_values.insert(B(index), prior_imu_bias);

  // Assemble prior noise model and add it the graph.`
  auto pose_noise_model = noiseModel::Diagonal::Sigmas(
      (Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5)
          .finished());  // rad,rad,rad,m, m, m
  auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
  auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph graph;
  graph.addPrior<Pose3>(X(index), prior_pose, pose_noise_model);
  graph.addPrior<Vector3>(V(index), prior_velocity, velocity_noise_model);
  graph.addPrior<imuBias::ConstantBias>(B(index), prior_imu_bias,
                                        bias_noise_model);

  auto p = imuParams();

  std::shared_ptr<PreintegrationType> preintegrated =
      std::make_shared<PreintegratedCombinedMeasurements>(p, prior_imu_bias);

  assert(preintegrated);

  // Store previous state for imu integration and latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Keep track of total error over the entire run as simple performance metric.
  double current_position_error = 0.0, current_orientation_error = 0.0;

  double output_time = 0.0;
  double dt = 0.005;  // The real system has noise, but here, results are nearly
                      // exactly the same, so keeping this for simplicity.

  // All priors have been set up, now iterate through the data file.
  while (file.good()) {
    // Parse out first value
    string value;
    getline(file, value, ',');
    int type = stoi(value.c_str());

    if (type == 0) {  // IMU measurement
      Vector6 imu;
      for (int i = 0; i < 5; ++i) {
        getline(file, value, ',');
        imu(i) = stof(value.c_str());
      }
      getline(file, value, '\n');
      imu(5) = stof(value.c_str());

      // Adding the IMU preintegration.
      preintegrated->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

    } else if (type == 1) {  // GPS measurement
      Vector7 gps;
      for (int i = 0; i < 6; ++i) {
        getline(file, value, ',');
        gps(i) = stof(value.c_str());
      }
      getline(file, value, '\n');
      gps(6) = stof(value.c_str());

      index++;

      // Adding IMU factor and GPS factor and optimizing.
      auto preint_imu_combined =
          dynamic_cast<const PreintegratedCombinedMeasurements&>(
              *preintegrated);
      CombinedImuFactor imu_factor(X(index - 1), V(index - 1), X(index),
                                   V(index), B(index - 1), B(index),
                                   preint_imu_combined);
      graph.add(imu_factor);

      auto correction_noise = noiseModel::Isotropic::Sigma(3, 1.0);
      GPSFactor gps_factor(X(index),
                           Point3(gps(0),   // N,
                                  gps(1),   // E,
                                  gps(2)),  // D,
                           correction_noise);
      graph.add(gps_factor);

      // Now optimize and compare results.
      prop_state = preintegrated->predict(prev_state, prev_bias);
      initial_values.insert(X(index), prop_state.pose());
      initial_values.insert(V(index), prop_state.v());
      initial_values.insert(B(index), prev_bias);

      LevenbergMarquardtParams params;
      params.setVerbosityLM("SUMMARY");
      LevenbergMarquardtOptimizer optimizer(graph, initial_values, params);
      Values result = optimizer.optimize();

      // Overwrite the beginning of the preintegration for the next step.
      prev_state =
          NavState(result.at<Pose3>(X(index)), result.at<Vector3>(V(index)));
      prev_bias = result.at<imuBias::ConstantBias>(B(index));

      // Reset the preintegration object.
      preintegrated->resetIntegrationAndSetBias(prev_bias);

      // Print out the position and orientation error for comparison.
      Vector3 result_position = prev_state.pose().translation();
      Vector3 position_error = result_position - gps.head<3>();
      current_position_error = position_error.norm();

      Quaternion result_quat = prev_state.pose().rotation().toQuaternion();
      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
      Quaternion quat_error = result_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3 euler_angle_error(quat_error.x() * 2, quat_error.y() * 2,
                                quat_error.z() * 2);
      current_orientation_error = euler_angle_error.norm();

      // display statistics
      cout << "Position error:" << current_position_error << "\t "
           << "Angular error:" << current_orientation_error << "\n"
           << endl;

      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
              output_time, result_position(0), result_position(1),
              result_position(2), result_quat.x(), result_quat.y(),
              result_quat.z(), result_quat.w(), gps(0), gps(1), gps(2),
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

      output_time += 1.0;

    } else {
      cerr << "ERROR parsing file\n";
      return 1;
    }
  }
  fclose(fp_out);
  cout << "Complete, results written to " << output_filename << "\n\n";

  return 0;
}
