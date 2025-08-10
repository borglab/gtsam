/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Scenario.cpp
 * @brief   Classes for testing navigation scenarios.
 * @author  Porter Zach
 */

#include <gtsam/navigation/Scenario.h>

#include <map>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

namespace gtsam {

Rot3 Scenario::rotation(double t) const { 
  return pose(t).rotation(); 
}
NavState Scenario::navState(double t) const { 
  return NavState(pose(t), velocity_n(t));
}

Vector3 Scenario::velocity_b(double t) const {
  const Rot3 nRb = rotation(t);
  return nRb.transpose() * velocity_n(t);
}

Vector3 Scenario::acceleration_b(double t) const {
  const Rot3 nRb = rotation(t);
  return nRb.transpose() * acceleration_n(t);
}

Pose3 ConstantTwistScenario::pose(double t) const {
  return nTb0_ * Pose3::Expmap(twist_ * t);
}

Vector3 ConstantTwistScenario::omega_b(double t) const { 
  return twist_.head<3>(); 
}

Vector3 ConstantTwistScenario::velocity_n(double t) const {
  return rotation(t).matrix() * twist_.tail<3>();
}

Vector3 ConstantTwistScenario::acceleration_n(double t) const { 
  return rotation(t) * a_b_; 
}

Pose3 AcceleratingScenario::pose(double t) const {
  return Pose3(nRb_.expmap(omega_b_ * t), p0_ + v0_ * t + a_n_ * t * t / 2.0);
}

Vector3 AcceleratingScenario::omega_b(double t) const { 
  return omega_b_; 
}

Vector3 AcceleratingScenario::velocity_n(double t) const { 
  return v0_ + a_n_ * t; 
}

Vector3 AcceleratingScenario::acceleration_n(double t) const { 
  return a_n_; 
}

Pose3 DiscreteScenario::pose(double t) const { 
  return interpolate(poses_, t); 
}

Vector3 DiscreteScenario::omega_b(double t) const {
  return interpolate(angularVelocities_b_, t);
}

Vector3 DiscreteScenario::velocity_n(double t) const {
  return interpolate(velocities_n_, t);
}

Vector3 DiscreteScenario::acceleration_n(double t) const {
  return interpolate(accelerations_n_, t);
}

DiscreteScenario DiscreteScenario::FromCSV(const string& csv_filepath) {
  ifstream file(csv_filepath);
  if (!file.is_open()) {
    throw runtime_error(
        "DiscreteScenario::FromCSV: Could not open file " + csv_filepath);
  }

  // Temporary storage for data before timestamp normalization
  struct DataPoint {
    double t;
    Pose3 pose;
    Vector3 omega_b, velocity_n, acceleration_n;
  };
  vector<DataPoint> data_points;

  string line;
  // Skip header line
  if (!getline(file, line)) {
    throw runtime_error(
        "DiscreteScenario::FromCSV: CSV file is empty or contains no data.");
  }

  // Read data rows
  int line_number = 1;
  while (getline(file, line)) {
    line_number++;
    stringstream ss(line);
    char comma; // To consume the commas
    DataPoint dp;
    double px, py, pz, qw, qx, qy, qz;

    // clang-format off
    if (!(ss >> dp.t >> comma >> px >> comma >> py >> comma >> pz >> comma >>
          qw >> comma >> qx >> comma >> qy >> comma >> qz >> comma >>
          dp.velocity_n[0] >> comma >> dp.velocity_n[1] >> comma >> 
          dp.velocity_n[2] >> comma >> dp.omega_b[0] >> comma >> 
          dp.omega_b[1] >> comma >> dp.omega_b[2] >> comma >> 
          dp.acceleration_n[0] >> comma >> dp.acceleration_n[1] >> comma >> 
          dp.acceleration_n[2])) {
      throw runtime_error(
          "DiscreteScenario::FromCSV: Malformed data at line " +
          to_string(line_number));
    }
    // clang-format on
    
    dp.pose = Pose3(Rot3::Quaternion(qw, qx, qy, qz), Point3(px, py, pz));
    data_points.push_back(dp);
  }

  if (data_points.empty()) {
    throw runtime_error(
        "DiscreteScenario::FromCSV: No data points loaded from file.");
  }

  // Normalize timestamps and populate maps
  const double t0 = data_points.front().t;
  map<double, Pose3> poses;
  map<double, Vector3> angularVelocities_b, velocities_n, accelerations_n;

  for (const auto& dp : data_points) {
    const double normalized_t = dp.t - t0;
    poses[normalized_t] = dp.pose;
    angularVelocities_b[normalized_t] = dp.omega_b;
    velocities_n[normalized_t] = dp.velocity_n;
    accelerations_n[normalized_t] = dp.acceleration_n;
  }

  return DiscreteScenario(poses, angularVelocities_b, velocities_n,
                          accelerations_n);
}

} // namespace gtsam
