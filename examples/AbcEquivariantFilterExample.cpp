/**
 * @file AbcEquivariantFilterExample.cpp
 * @brief Demonstration of the Attitude-Bias-Calibration Equivariant Filter
 *
 * This demo shows the Equivariant Filter (EqF) for attitude estimation
 * with both gyroscope bias and sensor extrinsic calibration, based on the
 * paper: "Overcoming Bias: Equivariant Filter Design for Biased Attitude
 * Estimation with Online Calibration" by Fornasier et al.
 *
 * This example uses the simplified AbcEquivariantFilter class which
 * provides a clean interface for predict/update operations without requiring
 * manual computation of Jacobian matrices and innovation functions.
 *
 * @author Darshan Rajasekaran
 * @author Jennifer Oum
 * @author Rohan Bansal
 * @author Frank Dellaert
 * @date 2025
 */

#include <gtsam/slam/dataset.h>
#include <gtsam_unstable/geometry/ABC.h>
#include <gtsam_unstable/geometry/ABCEquivariantFilter.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Use namespace for convenience
using namespace gtsam;
constexpr size_t n = 1;  // Number of calibration states
using M = abc::State<n>;
using AbcFilter = abc::AbcEquivariantFilter<n>;

/// Measurement struct
struct Measurement {
  Unit3 y;           /// Measurement direction in sensor frame
  Unit3 d;           /// Known direction in global frame
  Matrix3 R;         /// Covariance matrix of the measurement
  int cal_idx = -1;  /// Calibration index (-1 for calibrated sensor)
};

/// Data structure for ground-truth, input and output data
struct Data {
  M xi;                     /// Ground-truth state
  Vector3 omega;            /// Angular velocity measurement
  Matrix6 inputCovariance;  /// Input noise covariance (6x6 matrix)
  std::vector<Measurement> measurements;  /// Output measurements
  int numMeasurements;                    /// Number of measurements
  double t;                               /// Time
  double dt;                              /// Time step
};

//========================================================================
// Data Processing Functions
//========================================================================

/**
 * Load data from CSV file into a vector of Data objects for the EqF
 *
 * CSV format:
 * - t: Time
 * - q_w, q_x, q_y, q_z: True attitude quaternion
 * - b_x, b_y, b_z: True bias
 * - cq_w_0, cq_x_0, cq_y_0, cq_z_0: True calibration quaternion
 * - w_x, w_y, w_z: Angular velocity measurements
 * - std_w_x, std_w_y, std_w_z: Angular velocity measurement standard deviations
 * - std_b_x, std_b_y, std_b_z: Bias process noise standard deviations
 * - y_x_0, y_y_0, y_z_0, y_x_1, y_y_1, y_z_1: Direction measurements
 * - std_y_x_0, std_y_y_0, std_y_z_0, std_y_x_1, std_y_y_1, std_y_z_1: Direction
 * measurement standard deviations
 * - d_x_0, d_y_0, d_z_0, d_x_1, d_y_1, d_z_1: Reference directions
 *
 */
std::vector<Data> loadDataFromCSV(const std::string& filename, int startRow = 0,
                                  int maxRows = -1, int downsample = 1);

/// Process data with EqF and print summary results
void processDataWithEqF(AbcFilter& filter, const std::vector<Data>& data_list,
                        int printInterval = 10);

//========================================================================
// Data Processing Functions Implementation
//========================================================================

/*
 * Loads the test data from the csv file
 * startRow First row to load based on csv, 0 by default
 * maxRows maximum rows to load, defaults to all rows
 * downsample Downsample factor, default 1
 * A list of data objects
 */

std::vector<Data> loadDataFromCSV(const std::string& filename, int startRow,
                                  int maxRows, int downsample) {
  std::vector<Data> data_list;
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  std::cout << "Loading data from " << filename << "..." << std::flush;

  std::string line;
  int lineNumber = 0;
  int rowCount = 0;
  int errorCount = 0;
  double prevTime = 0.0;

  // Skip header
  std::getline(file, line);
  lineNumber++;

  // Skip to startRow
  while (lineNumber < startRow && std::getline(file, line)) {
    lineNumber++;
  }

  // Read data
  while (std::getline(file, line) && (maxRows == -1 || rowCount < maxRows)) {
    lineNumber++;

    // Apply downsampling
    if ((lineNumber - startRow - 1) % downsample != 0) {
      continue;
    }

    std::istringstream ss(line);
    std::string token;
    std::vector<double> values;

    // Parse line into values
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (const std::exception& e) {
        errorCount++;
        values.push_back(0.0);  // Use default value
      }
    }

    // Check if we have enough values
    if (values.size() < 39) {
      errorCount++;
      continue;
    }

    // Extract values
    double t = values[0];
    double dt = (rowCount == 0) ? 0.0 : t - prevTime;
    prevTime = t;

    // Create ground truth state
    Quaternion quat(values[1], values[2], values[3], values[4]);  // w, x, y, z
    Rot3 R = Rot3(quat);

    Vector3 b(values[5], values[6], values[7]);

    Quaternion calQuat(values[8], values[9], values[10],
                       values[11]);  // w, x, y, z
    std::array<Rot3, n> S = {Rot3(calQuat)};

    M xi(R, b, S);

    // Create input
    Vector3 omega(values[12], values[13], values[14]);

    // Create input covariance matrix (6x6)
    // First 3x3 block for angular velocity, second 3x3 block for bias process
    // noise
    Matrix6 inputCovariance = Matrix6::Zero();
    inputCovariance(0, 0) = values[15] * values[15];  // std_w_x^2
    inputCovariance(1, 1) = values[16] * values[16];  // std_w_y^2
    inputCovariance(2, 2) = values[17] * values[17];  // std_w_z^2
    inputCovariance(3, 3) = values[18] * values[18];  // std_b_x^2
    inputCovariance(4, 4) = values[19] * values[19];  // std_b_y^2
    inputCovariance(5, 5) = values[20] * values[20];  // std_b_z^2

    // Create measurements
    std::vector<Measurement> measurements;

    // First measurement (calibrated sensor, cal_idx = 0)
    Vector3 y0(values[21], values[22], values[23]);
    Vector3 d0(values[33], values[34], values[35]);

    // Normalize vectors if needed
    if (abs(y0.norm() - 1.0) > 1e-5) y0.normalize();
    if (abs(d0.norm() - 1.0) > 1e-5) d0.normalize();

    // Measurement covariance
    Matrix3 covY0 = Matrix3::Zero();
    covY0(0, 0) = values[27] * values[27];  // std_y_x_0^2
    covY0(1, 1) = values[28] * values[28];  // std_y_y_0^2
    covY0(2, 2) = values[29] * values[29];  // std_y_z_0^2

    // Create measurement
    measurements.push_back(Measurement{Unit3(y0), Unit3(d0), covY0, 0});

    // Second measurement (calibrated sensor, cal_idx = -1)
    Vector3 y1(values[24], values[25], values[26]);
    Vector3 d1(values[36], values[37], values[38]);

    // Normalize vectors if needed
    if (abs(y1.norm() - 1.0) > 1e-5) y1.normalize();
    if (abs(d1.norm() - 1.0) > 1e-5) d1.normalize();

    // Measurement covariance
    Matrix3 covY1 = Matrix3::Zero();
    covY1(0, 0) = values[30] * values[30];  // std_y_x_1^2
    covY1(1, 1) = values[31] * values[31];  // std_y_y_1^2
    covY1(2, 2) = values[32] * values[32];  // std_y_z_1^2

    // Create measurement
    measurements.push_back(Measurement{Unit3(y1), Unit3(d1), covY1, -1});

    // Create Data object and add to list
    data_list.push_back(
        Data{xi, omega, inputCovariance, measurements, 2, t, dt});

    rowCount++;

    // Show loading progress every 1000 rows
    if (rowCount % 1000 == 0) {
      std::cout << "." << std::flush;
    }
  }

  std::cout << " Done!" << std::endl;
  std::cout << "Loaded " << data_list.size() << " data points";

  if (errorCount > 0) {
    std::cout << " (" << errorCount << " errors encountered)";
  }

  std::cout << std::endl;

  return data_list;
}

/// Takes in the data and runs an EqF on it and reports the results
void processDataWithEqF(AbcFilter& filter, const std::vector<Data>& data_list,
                        int printInterval) {
  if (data_list.empty()) {
    std::cerr << "No data to process" << std::endl;
    return;
  }

  std::cout << "Processing " << data_list.size() << " data points with EqF..."
            << std::endl;

  // Track performance metrics
  std::vector<double> att_errors;
  std::vector<double> bias_errors;
  std::vector<double> cal_errors;

  // Track time for performance measurement
  auto start = std::chrono::high_resolution_clock::now();

  int totalMeasurements = 0;
  int validMeasurements = 0;

  // Define constant for converting radians to degrees
  const double RAD_TO_DEG = 180.0 / M_PI;

  // Print a progress indicator
  int progressStep = data_list.size() / 10;  // 10 progress updates
  if (progressStep < 1) progressStep = 1;

  std::cout << "Progress: ";

  for (size_t i = 0; i < data_list.size(); i++) {
    const Data& data = data_list[i];
    filter.predict(data.omega, data.inputCovariance, data.dt);

    // Process all measurements
    for (const auto& measurement : data.measurements) {
      totalMeasurements++;

      // Skip invalid measurements
      Vector3 y_vec = measurement.y.unitVector();
      Vector3 d_vec = measurement.d.unitVector();
      if (std::isnan(y_vec[0]) || std::isnan(y_vec[1]) ||
          std::isnan(y_vec[2]) || std::isnan(d_vec[0]) ||
          std::isnan(d_vec[1]) || std::isnan(d_vec[2])) {
        continue;
      }

      try {
        filter.update(measurement.y, measurement.d, measurement.R,
                      measurement.cal_idx);
        validMeasurements++;
      } catch (const std::exception& e) {
        std::cerr << "Error updating at t=" << data.t << ": " << e.what()
                  << std::endl;
      }
    }

    // Get current estimates using accessor methods
    Rot3 att_est = filter.attitude();
    Vector3 bias_est = filter.bias();
    Rot3 cal_est = filter.calibration(0);

    // Calculate errors
    Vector3 att_error = Rot3::Logmap(data.xi.R.between(att_est));
    Vector3 bias_error = bias_est - data.xi.b;
    Vector3 cal_error = Z_3x1;
    if (!data.xi.S.empty()) {
      cal_error = Rot3::Logmap(data.xi.S[0].between(cal_est));
    }

    // Store errors
    att_errors.push_back(att_error.norm());
    bias_errors.push_back(bias_error.norm());
    cal_errors.push_back(cal_error.norm());

    // Show progress dots
    if (i % progressStep == 0) {
      std::cout << "." << std::flush;
    }
  }

  std::cout << " Done!" << std::endl;

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  // Calculate average errors
  double avg_att_error = 0.0;
  double avg_bias_error = 0.0;
  double avg_cal_error = 0.0;

  if (!att_errors.empty()) {
    avg_att_error = std::accumulate(att_errors.begin(), att_errors.end(), 0.0) /
                    att_errors.size();
    avg_bias_error =
        std::accumulate(bias_errors.begin(), bias_errors.end(), 0.0) /
        bias_errors.size();
    avg_cal_error = std::accumulate(cal_errors.begin(), cal_errors.end(), 0.0) /
                    cal_errors.size();
  }

  // Calculate final errors from last data point
  const Data& final_data = data_list.back();
  Rot3 final_att_est = filter.attitude();
  Vector3 final_bias_est = filter.bias();
  Rot3 final_cal_est = filter.calibration(0);

  Vector3 final_att_error =
      Rot3::Logmap(final_data.xi.R.between(final_att_est));
  Vector3 final_bias_error = final_bias_est - final_data.xi.b;
  Vector3 final_cal_error = Z_3x1;
  if (!final_data.xi.S.empty()) {
    final_cal_error = Rot3::Logmap(final_data.xi.S[0].between(final_cal_est));
  }

  // Print summary statistics
  std::cout << "\n=== Filter Performance Summary ===" << std::endl;
  std::cout << "Processing time: " << elapsed.count() << " seconds"
            << std::endl;
  std::cout << "Processed measurements: " << totalMeasurements
            << " (valid: " << validMeasurements << ")" << std::endl;

  // Average errors
  std::cout << "\n-- Average Errors --" << std::endl;
  std::cout << "Attitude: " << (avg_att_error * RAD_TO_DEG) << "°" << std::endl;
  std::cout << "Bias: " << avg_bias_error << std::endl;
  std::cout << "Calibration: " << (avg_cal_error * RAD_TO_DEG) << "°"
            << std::endl;

  // Final errors
  std::cout << "\n-- Final Errors --" << std::endl;
  std::cout << "Attitude: " << (final_att_error.norm() * RAD_TO_DEG) << "°"
            << std::endl;
  std::cout << "Bias: " << final_bias_error.norm() << std::endl;
  std::cout << "Calibration: " << (final_cal_error.norm() * RAD_TO_DEG) << "°"
            << std::endl;

  // Print a brief comparison of final estimate vs ground truth
  std::cout << "\n-- Final State vs Ground Truth --" << std::endl;
  std::cout << "Attitude (RPY) - Estimate: "
            << (final_att_est.rpy() * RAD_TO_DEG).transpose()
            << "° | Truth: " << (final_data.xi.R.rpy() * RAD_TO_DEG).transpose()
            << "°" << std::endl;
  std::cout << "Bias - Estimate: " << final_bias_est.transpose()
            << " | Truth: " << final_data.xi.b.transpose() << std::endl;

  if (!final_data.xi.S.empty()) {
    std::cout << "Calibration (RPY) - Estimate: "
              << (final_cal_est.rpy() * RAD_TO_DEG).transpose() << "° | Truth: "
              << (final_data.xi.S[0].rpy() * RAD_TO_DEG).transpose() << "°"
              << std::endl;
  }
}

int main(int argc, char* argv[]) {
  std::cout << "ABC-EqF: Attitude-Bias-Calibration Equivariant Filter Demo"
            << std::endl;
  std::cout << "=============================================================="
            << std::endl;

  try {
    // Parse command line options
    std::string csvFilePath;
    int maxRows = -1;    // Process all rows by default
    int downsample = 1;  // No downsampling by default

    if (argc > 1) {
      csvFilePath = argv[1];
    } else {
      // Try to find the EQFdata file in the GTSAM examples directory
      try {
        csvFilePath = findExampleDataFile("EqFdata.csv");
      } catch (const std::exception& e) {
        std::cerr << "Error: Could not find EqFdata.csv" << std::endl;
        std::cerr << "Usage: " << argv[0]
                  << " [csv_file_path] [max_rows] [downsample]" << std::endl;
        return 1;
      }
    }

    // Optional command line parameters
    if (argc > 2) {
      maxRows = std::stoi(argv[2]);
    }

    if (argc > 3) {
      downsample = std::stoi(argv[3]);
    }

    // Load data from CSV file
    std::vector<Data> data =
        loadDataFromCSV(csvFilePath, 0, maxRows, downsample);

    if (data.empty()) {
      std::cerr << "No data available to process. Exiting." << std::endl;
      return 1;
    }

    // Initial covariance - larger values allow faster convergence
    Matrix initialSigma = Matrix::Identity(6 + 3 * n, 6 + 3 * n);
    initialSigma.diagonal().head<3>() =
        Vector3::Constant(0.1);  // Attitude uncertainty
    initialSigma.diagonal().segment<3>(3) =
        Vector3::Constant(0.01);  // Bias uncertainty
    initialSigma.diagonal().tail<3>() =
        Vector3::Constant(0.1);  // Calibration uncertainty

    // Create filter with initial covariance (starts at identity state)
    AbcFilter filter(initialSigma);

    // Process data
    processDataWithEqF(filter, data);

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "\nEqF demonstration completed successfully." << std::endl;
  return 0;
}
