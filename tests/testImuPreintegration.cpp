/**
 * @file   testImuPreintegration.cpp
 * @brief  Unit tests for IMU Preintegration
 * @author Russell Buchanan
 **/

#include <tests/ImuMeasurement.h>

#include <fstream>
#include <iostream>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/CombinedImuFactor.h>

namespace drs {

Measurement::Measurement() : dt(0), time(0), type("UNDEFINED") {}

Measurement::Measurement(std::string _type) : dt(0), time(0), type(_type) {}

ImuMeasurement::ImuMeasurement() : I_a_WI{0, 0, 0}, I_w_WI{0, 0, 0} { type = "ImuMeasurement"; }

std::ostream& operator<<(std::ostream& stream, const ImuMeasurement& meas) {
  stream << "IMU Measurement at time = " << meas.time << " : \n"
         << "dt    : " << meas.dt << "\n"
         << "I_a_WI: " << meas.I_a_WI.transpose() << "\n"
         << "I_w_WI: " << meas.I_w_WI.transpose() << "\n";
  return stream;
}

}  // namespace drs

using namespace gtsam;

/* ************************************************************************* */
/// \brief Uses the GTSAM library to perform IMU preintegration on an acceleration input.
///
TEST(GtsamLibraryTests, LoadedSimulationData) {
  Eigen::Vector3d finalPos;

  std::vector<drs::ImuMeasurement> imuMeasurements;

  double accNoiseSigma = 0.001249;
  double accBiasRwSigma = 0.000106;
  double gyrNoiseSigma = 0.000208;
  double gyrBiasRwSigma = 0.000004;
  double integrationCovariance = 1e-8;
  double biasAccOmegaInt = 1e-5;

  double gravity = 9.81;
  double rate = 400.0;  // Hz

  /// @todo Update directory to correct location
  std::string inFileString = "/home/russell/imu_data.csv";
  std::ofstream outputFile;
  outputFile.open("/home/russell/gtsam_output.csv", std::ofstream::out);
  std::ifstream inputFile(inFileString);
  std::string line;
  while (std::getline(inputFile, line)) {
    std::stringstream ss(line);
    std::string str;
    std::vector<double> results;
    while (getline(ss, str, ',')) {
      results.push_back(std::atof(str.c_str()));
    }
    drs::ImuMeasurement measurement;
    measurement.dt = static_cast<uint64_t>(1e9 * (1 / rate));
    measurement.time = results[2];
    measurement.I_a_WI = {results[29], results[30], results[31]};
    measurement.I_w_WI = {results[17], results[18], results[19]};
    imuMeasurements.push_back(measurement);

    // std::cout << "IMU measurement " << measurement << std::endl;
  }

  // Assume a Z-up navigation (assuming we are performing optimization in the IMU frame).
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuPreintegratedParams =
      gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(gravity);
  imuPreintegratedParams->accelerometerCovariance = I_3x3 * pow(accNoiseSigma, 2);
  imuPreintegratedParams->biasAccCovariance = I_3x3 * pow(accBiasRwSigma, 2);
  imuPreintegratedParams->gyroscopeCovariance = I_3x3 * pow(gyrNoiseSigma, 2);
  imuPreintegratedParams->biasOmegaCovariance = I_3x3 * pow(gyrBiasRwSigma, 2);
  imuPreintegratedParams->integrationCovariance = I_3x3 * integrationCovariance;
  imuPreintegratedParams->biasAccOmegaInt = I_6x6 * biasAccOmegaInt;

  // Initial state
  gtsam::Pose3 priorPose;
  gtsam::Vector3 priorVelocity;
  gtsam::imuBias::ConstantBias priorImuBias;
  gtsam::PreintegratedCombinedMeasurements imuPreintegrated;
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  gtsam::NavState propState;

  gtsam::NavState initialNavState(priorPose, priorVelocity);

  // Bias estimated by my Algorithm
  priorImuBias =
      gtsam::imuBias::ConstantBias(Eigen::Vector3d(-0.0314648, 0.0219921, 6.95945e-05),
                                   Eigen::Vector3d(4.88581e-08, -1.04971e-09, -0.000122868));

  // zero bias
  // priorImuBias = gtsam::imuBias::ConstantBias(Eigen::Vector3d(0,0,0),
  //                                             Eigen::Vector3d(0,0,0));

  imuPreintegrated = gtsam::PreintegratedCombinedMeasurements(imuPreintegratedParams, priorImuBias);

  // Put header row in output csv
  outputFile << "X Position,"
             << "Y Position,"
             << "Z Position,"
             << "X Velocity,"
             << "Y Velocity,"
             << "Z Velocity,"
             << "\n";

  for (int n = 1; n < imuMeasurements.size(); n++) { //start at 1 to skip header
    // integrate
    imuPreintegrated.integrateMeasurement(imuMeasurements[n].I_a_WI, imuMeasurements[n].I_w_WI,
                                          1 / rate);
    // predict
    propState = imuPreintegrated.predict(initialNavState, priorImuBias);
    position = propState.pose().translation();
    velocity = propState.velocity();
    // std::cout << "IMU Position " << position.transpose() << std::endl;
    // std::cout << "IMU Velocity " << velocity.transpose() << std::endl;

    // Write to csv
    outputFile << std::to_string(position.x()) << "," << std::to_string(position.y()) << ","
               << std::to_string(position.z()) << "," << std::to_string(velocity.x()) << ","
               << std::to_string(velocity.y()) << "," << std::to_string(velocity.z()) << ","
               << "\n";
  }

  outputFile.close();

  gtsam::Vector3 rotation = propState.pose().rotation().rpy();

  // Dont have ground truth for x and y position yet
  // DOUBLES_EQUAL(0.1, position[0], 1e-2);
  // DOUBLES_EQUAL(0.1, position[1], 1e-2);
  DOUBLES_EQUAL(0.0, position[2], 1e-2);

  DOUBLES_EQUAL(0.0, rotation[0], 1e-2);
  DOUBLES_EQUAL(0.0, rotation[1], 1e-2);
  DOUBLES_EQUAL(0.0, rotation[2], 1e-2);
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
