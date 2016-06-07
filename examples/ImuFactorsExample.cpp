/*
   @author Garrett (ghemann@gmail.com), Luca Carlone
   @date   08.13.15
   @brief  Test example for using GTSAM ImuFactor and ImuCombinedFactor navigation code.
*/

// Standard includes.
#include <fstream>
#include <iostream>

// GTSAM related includes.
#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// A row starting with i is the first inital position formatted with
//   N, E, D, qx, qY, qZ, qW, velN, velE, velD
// A row starting with a 0 is an imu measurement
//   linAccN, linAccE, linAccD, angVelN, angVelE, angVelD
// A row starting with a 1 is a gps correction formatted with
//   N, E, D, qX, qY, qZ, qW
// Note that for correction, we're only using the point not the rotation. The
// rotation is provided for ground truth comparison.

// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
#define USE_COMBINED 

using namespace gtsam;
using namespace Eigen;
using namespace std;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

const string output_filename = "imuFactorExampleResults.csv";

// This will either be PreintegratedImuMeasurements (for ImuFactor) or
// PreintegratedCombinedMeasurements (for CombinedImuFactor).
PreintegrationType *imu_preintegrated_;

int main(int argc, char* argv[])
{
  string data_filename;
  if (argc < 2) {
    printf("using default CSV file\n");
    data_filename = findExampleDataFile("imuAndGPSdata.csv");
  } else {
    data_filename = argv[1];
  }
  printf("1\n");
  // Set up output file for plotting errors
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  printf("2\n");
  fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw,gt_x(m),gt_y(m),gt_z(m),gt_qx,gt_qy,gt_qz,gt_qw\n");

  // Begin parsing the CSV file.  Input the first line for initialization.
  // From there, we'll iterate through the file and we'll preintegrate the IMU
  // or add in the GPS given the input.
  std::ifstream file(data_filename.c_str());
  std::string value;

  // Format is (N,E,D,qX,qY,qZ,qW,velN,velE,velD)
  Eigen::Matrix<double,10,1> initial_state = Eigen::Matrix<double,10,1>::Zero();

  getline(file, value, ','); // i
  for (int i=0; i<9; i++) {
    getline(file, value, ',');
    initial_state(i) = std::atof(value.c_str());
  }
  getline(file, value, '\n');
  initial_state(9) = std::atof(value.c_str());

  std::cout << "initial state:\n" << initial_state << "\n\n";

  // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
  Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3), 
                                         initial_state(4), initial_state(5));
  Point3 prior_point(initial_state.head<3>());
  Pose3 prior_pose(prior_rotation, prior_point);
  Vector3 prior_velocity(initial_state.tail<3>());
  imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

  Values initial_values;
  int correction_count = 0;
  initial_values.insert(X(correction_count), prior_pose);
  initial_values.insert(V(correction_count), prior_velocity);
  initial_values.insert(B(correction_count), prior_imu_bias);  

  // Assemble prior noise model and add it the graph.
  noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // m, m, m, deg, deg, deg
  noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.1, 0.1, 0.1).finished()); // m/s
  noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished());

  // Add all prior factors (pose, velocity, bias) to the graph.
  NonlinearFactorGraph *graph = new NonlinearFactorGraph();
  graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
  graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity, 
                                  velocity_noise_model));
  graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias, 
                                                bias_noise_model));

  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 0.0003924;
  double gyro_noise_sigma = 0.000205689024915;
  double accel_bias_rw_sigma = 0.004905;
  double gyro_bias_rw_sigma = 0.000001454441043;
  Matrix3d measured_acc_cov = Matrix3d::Identity(3,3) * std::pow(accel_noise_sigma,2);
  Matrix3d measured_omega_cov = Matrix3d::Identity(3,3) * std::pow(gyro_noise_sigma,2);
  Matrix3d integration_error_cov = Matrix3d::Identity(3,3)*100000; // ?
  Matrix3d bias_acc_cov = Matrix3d::Identity(3,3) * std::pow(accel_bias_rw_sigma,2);
  Matrix3d bias_omega_cov = Matrix3d::Identity(3,3) * std::pow(gyro_bias_rw_sigma,2);
  Eigen::Matrix<double,6,6> bias_acc_omega_int = MatrixXd::Identity(6,6)*0.001; // ?

  //#ifdef USE_COMBINED
  boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
  // PreintegrationBase params:
  p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;
  
#ifdef USE_COMBINED
  imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prior_imu_bias);
#else
  imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
#endif

  // Store previous state for the imu integration and the latest predicted outcome.
  NavState prev_state(prior_pose, prior_velocity);
  NavState prop_state = prev_state;
  imuBias::ConstantBias prev_bias = prior_imu_bias;

  // Keep track of the total error over the entire run for a simple performance metric.
  double pose_error_sum = 0.0, orientation_error_sum = 0.0;

  double output_time = 0.0;
  double dt = 0.005;  // The real system has noise, but here, results are nearly 
                      // exactly the same, so keeping this for simplicity.

  // All priors have been set up, now iterate through the data file.
  while (file.good()) {

    // Parse out first value
    getline(file, value, ',');
    int type = std::atoi(value.c_str());

    if (type == 0) { // IMU measurement
      Eigen::Matrix<double,6,1> imu = Eigen::Matrix<double,6,1>::Zero();
      for (int i=0; i<5; ++i) {
        getline(file, value, ',');
        imu(i) = std::atof(value.c_str());
      }
      getline(file, value, '\n');
      imu(5) = std::atof(value.c_str());

      // Adding the IMU preintegration.
      imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);
      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);

    } else if (type == 1) { // GPS measurement
      Eigen::Matrix<double,7,1> gps = Eigen::Matrix<double,7,1>::Zero();
      for (int i=0; i<6; ++i) {
        getline(file, value, ',');
        gps(i) = std::atof(value.c_str());
      }
      getline(file, value, '\n');
      gps(6) = std::atof(value.c_str());

      correction_count++;

      // Adding IMU factor and GPS factor and optimizing.
#ifdef USE_COMBINED
      PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
      CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                                   X(correction_count  ), V(correction_count  ),
                                   B(correction_count-1), B(correction_count  ),
                                   *preint_imu_combined);
      graph->add(imu_factor);
#else
      PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
      ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
                           X(correction_count  ), V(correction_count  ),
                           B(correction_count-1),
                           *preint_imu);
      graph->add(imu_factor);
      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
      graph->add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1), 
                                                      B(correction_count  ), 
                                                      zero_bias, bias_noise_model));

#endif


      noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Diagonal::Variances((Vector(3) << 1, 1, 1).finished());
      GPSFactor gps_factor(X(correction_count),
                           Point3(gps(0),  // N,
                                  gps(1),  // E,
                                  gps(2)), // D,
                           correction_noise);
      graph->add(gps_factor);
      
      // Now optimize and compare results.
      initial_values.insert(X(correction_count), prop_state.pose());
      initial_values.insert(V(correction_count), prop_state.v());
      initial_values.insert(B(correction_count), prev_bias);

      LevenbergMarquardtOptimizer optimizer(*graph, initial_values);
      Values result = optimizer.optimize();

      // Overwrite the beginning of the preintegration for the next step.
      prev_state = NavState(result.at<Pose3>(X(correction_count)),
                            result.at<Vector3>(V(correction_count)));
      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));

      // Reset the preintegration object.
      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
      //prev_bias.print();

      // Print out the position and orientation error for comparison.
      Vector3d gtsam_pose = prev_state.pose().translation().vector();
      Quaterniond gtsam_quat = prev_state.pose().rotation().toQuaternion();
      Quaterniond gps_quat(gps(6), gps(3), gps(4), gps(5));

      Vector3d position_error = gtsam_pose - gps.head<3>();
      Quaterniond quat_error = gtsam_quat * gps_quat.inverse();
      quat_error.normalize();
      Vector3d euler_angle_error(quat_error.x()*2,
                                 quat_error.y()*2,
                                 quat_error.z()*2);

      pose_error_sum += position_error.norm();
      orientation_error_sum += euler_angle_error.norm();

      std::cout << pose_error_sum << "\t " << orientation_error_sum << "\n";
      //std::cout << "For correction " << correction_count-1 << ", pose error is:\n" << position_error << "\n(in meters, NED), and quaternion error is:\n" << euler_angle_error*(180/M_PI) << "\n(in degrees)\n\n";


      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
              output_time, gtsam_pose(0), gtsam_pose(1), gtsam_pose(2),
              gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
              gps(0), gps(1), gps(2), 
              gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());

      output_time += 1.0;

    } else {
      std::cerr << "ERROR parsing file\n";
      return 1;
    }
    
  }

  fclose(fp_out);

  std::cout << "Complete, results written to " << output_filename << "\n\n";;
  return 0;
}
