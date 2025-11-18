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
#include <algorithm>

using namespace std;

namespace gtsam {

Rot3 Scenario::rotation(double t) const { 
  return pose(t).rotation(); 
}
NavState Scenario::navState(double t) const { 
  return NavState(pose(t), velocity_n(t));
}
Gal3 Scenario::gal3(double t) const {
  return Gal3::FromPoseVelocityTime(pose(t), velocity_n(t), t);
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

double DiscreteScenario::duration() const {
  return t_;
}

DiscreteScenario DiscreteScenario::FromCSV(const string& csv_filepath) {
  ifstream file(csv_filepath);
  if (!file.is_open()) {
    throw runtime_error("DiscreteScenario::FromCSV: Could not open file " +
                        csv_filepath);
  }

  // Header parsing
  string header_line;
  if (!getline(file, header_line)) {
    throw runtime_error(
        "DiscreteScenario::FromCSV: CSV file is empty or contains no header.");
  }

  map<string, int> columnIndex;
  stringstream header_ss(header_line);
  string column_name;
  int index = 0;
  while (getline(header_ss, column_name, ',')) {
    // Basic trim for whitespace, in case of " col1, col2 "
    column_name.erase(0, column_name.find_first_not_of(" \t\r\n"));
    column_name.erase(column_name.find_last_not_of(" \t\r\n") + 1);
    columnIndex[column_name] = index++;
  }

  // Define and validate required columns
  // These are the header names we expect to find in the CSV. Based on EuRoC MAV
  // dataset format.
  const vector<string> required_columns = {
      "t", "p_x", "p_y", "p_z", "q_w", "q_x", "q_y", "q_z",
      "v_x", "v_y", "v_z", "w_x", "w_y", "w_z", "a_x", "a_y",
      "a_z"};

  for (const auto& col : required_columns) {
    if (columnIndex.find(col) == columnIndex.end()) {
      throw runtime_error(
          "DiscreteScenario::FromCSV: Missing required column header '" + col +
          "' in file " + csv_filepath);
    }
  }

  // Temporary storage for data
  struct DataPoint {
    double t;
    Pose3 pose;
    Vector3 omega_b, velocity_n, acceleration_n;
  };
  vector<DataPoint> data_points;

  // Read data rows
  string data_line;
  int line_number = 1; // Header was line 1
  while (getline(file, data_line)) {
    line_number++;
    stringstream data_ss(data_line);
    vector<string> values;
    string value;
    while (getline(data_ss, value, ',')) {
      values.push_back(value);
    }

    if (values.size() != columnIndex.size()) {
       throw runtime_error(
          "DiscreteScenario::FromCSV: Malformed data at line " +
          to_string(line_number) + ". Expected " + to_string(columnIndex.size()) +
          " columns, but found " + to_string(values.size()) + ".");
    }
    
    try {
      DataPoint dp;
      // Use the columnIndex map to get data by name, not by fixed position
      dp.t = stod(values[columnIndex.at("t")]);
      
      double px = stod(values[columnIndex.at("p_x")]);
      double py = stod(values[columnIndex.at("p_y")]);
      double pz = stod(values[columnIndex.at("p_z")]);
      
      double qw = stod(values[columnIndex.at("q_w")]);
      double qx = stod(values[columnIndex.at("q_x")]);
      double qy = stod(values[columnIndex.at("q_y")]);
      double qz = stod(values[columnIndex.at("q_z")]);

      dp.velocity_n << stod(values[columnIndex.at("v_x")]),
                       stod(values[columnIndex.at("v_y")]),
                       stod(values[columnIndex.at("v_z")]);

      dp.omega_b << stod(values[columnIndex.at("w_x")]),
                    stod(values[columnIndex.at("w_y")]),
                    stod(values[columnIndex.at("w_z")]);

      dp.acceleration_n << stod(values[columnIndex.at("a_x")]),
                           stod(values[columnIndex.at("a_y")]),
                           stod(values[columnIndex.at("a_z")]);

      dp.pose = Pose3(Rot3::Quaternion(qw, qx, qy, qz), Point3(px, py, pz));
      data_points.push_back(dp);

    } catch (const std::invalid_argument& e) {
        throw runtime_error(
          "DiscreteScenario::FromCSV: Non-numeric data at line " +
          to_string(line_number) + ". " + e.what());
    } catch (const std::out_of_range& e) {
        // This can be triggered by stod or by map::at if something is wrong
        throw runtime_error(
          "DiscreteScenario::FromCSV: Data conversion error at line " +
          to_string(line_number) + ". " + e.what());
    }
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