/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/// @file EqVIOFilterCsvReplay.cpp
/// @brief Replay preprocessed EqVIO CSV data without feature tracking.

#include <gtsam_unstable/navigation/EqVIOCsv.h>
#include <gtsam_unstable/navigation/EqVIOFilter.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

using namespace gtsam;
using namespace gtsam::eqvio;

namespace {

struct HardcodedGroundTruth {
  static constexpr double t = 1403715283.3121430874;
  static Vector3 position() { return Vector3(-0.954631, -0.101702, 0.179862); }
  static Vector3 velocity() { return Vector3(-0.120739, -0.314283, 0.119599); }
};

constexpr double kTimeTolerance = 1e-1;
constexpr double kPositionTolerance = 5e-2;
constexpr double kVelocityTolerance = 5e-2;

class NormalizedCamera final : public VIOCameraModel {
 public:
  Point2 projectPoint(const Point3& p) const override {
    if (std::abs(p.z()) < 1e-12) {
      throw std::invalid_argument("NormalizedCamera: z near zero");
    }
    return Point2(p.x() / p.z(), p.y() / p.z());
  }

  Vector3 undistortPoint(const Point2& y) const override {
    return Vector3(y.x(), y.y(), 1.0);
  }

  Matrix23 projectionJacobian(const Vector3& y) const override {
    if (std::abs(y.z()) < 1e-12) {
      throw std::invalid_argument("NormalizedCamera: z near zero");
    }
    Matrix23 J;
    const double z2 = y.z() * y.z();
    J << 1.0 / y.z(), 0.0, -y.x() / z2, 0.0, 1.0 / y.z(), -y.y() / z2;
    return J;
  }
};

bool metadataBool(const EqVIOCsvLog& log, const std::string& key, bool fallback) {
  const auto it = log.metadata.find(key);
  if (it == log.metadata.end()) return fallback;
  const std::string& v = it->second;
  if (v == "1" || v == "true" || v == "TRUE" || v == "True") return true;
  if (v == "0" || v == "false" || v == "FALSE" || v == "False") return false;
  return fallback;
}

double metadataFiniteDouble(const EqVIOCsvLog& log, const std::string& key,
                            double fallback) {
  const double v = metadataDouble(log, key, fallback);
  if (!std::isfinite(v)) return fallback;
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage:\n  " << argv[0] << " <processed_eqvio_stream.csv>\n";
    return 1;
  }

  const std::string csvPath = argv[1];

  try {
    const EqVIOCsvLog log = readEqVIOCsv(csvPath);

    EqVIOFilterParams params;
    params.coordinateChoice = CoordinateChoice::InvDepth;
    params.fastRiccati =
        metadataBool(log, "eqf.fast_riccati", params.fastRiccati);
    params.useDiscreteStateMatrix = metadataBool(
        log, "eqf.use_discrete_state_matrix", params.useDiscreteStateMatrix);
    params.useDiscreteVelocityLift = metadataBool(
        log, "eqf.use_discrete_velocity_lift", params.useDiscreteVelocityLift);
    params.useDiscreteInnovationLift = metadataBool(
        log, "eqf.use_discrete_innovation_lift",
        params.useDiscreteInnovationLift);
    params.useEquivariantOutput =
        metadataBool(log, "eqf.use_equivariant_output", params.useEquivariantOutput);
    params.removeLostLandmarks =
        metadataBool(log, "eqf.remove_lost_landmarks", params.removeLostLandmarks);
    params.removeInvalidLandmarks = true;
    params.useMedianDepth =
        metadataBool(log, "eqf.use_median_depth", params.useMedianDepth);

    params.initialPointDepth = metadataFiniteDouble(
        log, "eqf.initial_point_depth", params.initialPointDepth);
    params.initialPointVariance = metadataFiniteDouble(
        log, "eqf.initial_point_variance", params.initialPointVariance);
    params.measurementNoiseVariance = metadataFiniteDouble(
        log, "eqf.measurement_noise_variance_norm",
        params.measurementNoiseVariance);
    params.outlierThresholdAbs = metadataFiniteDouble(
        log, "eqf.outlier_threshold_abs", params.outlierThresholdAbs);
    params.outlierThresholdAbs = metadataFiniteDouble(
        log, "eqf.outlier_threshold_abs_norm", params.outlierThresholdAbs);
    params.outlierThresholdProb = metadataFiniteDouble(
        log, "eqf.outlier_threshold_prob", params.outlierThresholdProb);
    params.featureRetention = metadataFiniteDouble(
        log, "eqf.feature_retention", params.featureRetention);

    params.biasOmegaProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_bias_omega", params.biasOmegaProcessVariance);
    params.biasAccelProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_bias_accel", params.biasAccelProcessVariance);
    params.attitudeProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_attitude", params.attitudeProcessVariance);
    params.positionProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_position", params.positionProcessVariance);
    params.velocityProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_velocity", params.velocityProcessVariance);
    params.cameraAttitudeProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_cam_attitude", params.cameraAttitudeProcessVariance);
    params.cameraPositionProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_cam_position", params.cameraPositionProcessVariance);
    params.pointProcessVariance = metadataFiniteDouble(
        log, "eqf.process_var_point", params.pointProcessVariance);

    params.inputNoise.setZero();
    params.inputNoise.block<3, 3>(0, 0).setIdentity();
    params.inputNoise.block<3, 3>(3, 3).setIdentity();
    params.inputNoise.block<3, 3>(6, 6).setIdentity();
    params.inputNoise.block<3, 3>(9, 9).setIdentity();
    params.inputNoise.block<3, 3>(0, 0) *=
        metadataFiniteDouble(log, "eqf.input_var_gyr",
                             params.inputNoise(0, 0));
    params.inputNoise.block<3, 3>(3, 3) *=
        metadataFiniteDouble(log, "eqf.input_var_acc",
                             params.inputNoise(3, 3));
    params.inputNoise.block<3, 3>(6, 6) *=
        metadataFiniteDouble(log, "eqf.input_var_gyr_bias_walk",
                             params.inputNoise(6, 6));
    params.inputNoise.block<3, 3>(9, 9) *=
        metadataFiniteDouble(log, "eqf.input_var_acc_bias_walk",
                             params.inputNoise(9, 9));

    VIOSensorState sensor;
    sensor.inputBias = VIOBias::Identity();
    sensor.pose = Pose3::Identity();
    sensor.velocity.setZero();
    sensor.cameraOffset = cameraExtrinsicsFromMetadata(log).value_or(Pose3::Identity());
    VIOState xi0(sensor, {});
    Matrix Sigma0 = Matrix::Identity(xi0.dim(), xi0.dim());
    Sigma0.block<3, 3>(0, 0) *= metadataFiniteDouble(
        log, "eqf.initial_var_bias_omega", 0.1);
    Sigma0.block<3, 3>(3, 3) *= metadataFiniteDouble(
        log, "eqf.initial_var_bias_accel", 0.1);
    Sigma0.block<3, 3>(6, 6) *= metadataFiniteDouble(
        log, "eqf.initial_var_attitude", 1e-4);
    Sigma0.block<3, 3>(9, 9) *= metadataFiniteDouble(
        log, "eqf.initial_var_position", 1e-4);
    Sigma0.block<3, 3>(12, 12) *= metadataFiniteDouble(
        log, "eqf.initial_var_velocity", 1e-2);
    Sigma0.block<3, 3>(15, 15) *= metadataFiniteDouble(
        log, "eqf.initial_var_cam_attitude", 1e-5);
    Sigma0.block<3, 3>(18, 18) *= metadataFiniteDouble(
        log, "eqf.initial_var_cam_position", 1e-4);

    EqVIOFilter filter(params);
    filter.setReferenceState(xi0, Sigma0);
    auto camera = std::make_shared<NormalizedCamera>();

    size_t imuCount = 0;
    size_t visionFrameCount = 0;
    size_t visionFeatureCount = 0;
    for (const EqVIOCsvEvent& event : log.events) {
      if (event.type == EqVIOCsvEvent::Type::Imu) {
        filter.processIMUData(event.imu);
        ++imuCount;
      } else {
        const Matrix R =
            Matrix::Identity(static_cast<int>(2 * event.vision.size()),
                             static_cast<int>(2 * event.vision.size())) *
            params.measurementNoiseVariance;
        filter.processVisionData(event.tAbs, event.vision, camera, R);
        ++visionFrameCount;
        visionFeatureCount += event.vision.size();
      }
    }

    const VIOState estimate = filter.stateEstimate();
    std::cout << "CSV replay complete.\n";
    std::cout << "Events: " << log.events.size() << ", IMU: " << imuCount
              << ", vision frames: " << visionFrameCount
              << ", vision features: " << visionFeatureCount << "\n";
    std::cout << "Measurement noise variance (normalized): "
              << params.measurementNoiseVariance << "\n";
    std::cout << std::setprecision(17);
    std::cout << "Filter time: " << filter.currentTime() << "\n";
    std::cout << std::setprecision(10);
    std::cout << "Landmarks: " << estimate.n() << "\n";
    std::cout << "Pose translation: "
              << estimate.sensor.pose.translation().transpose() << "\n";
    std::cout << "GT pose translation: "
              << HardcodedGroundTruth::position().transpose() << "\n";
    std::cout << "Velocity: " << estimate.sensor.velocity.transpose() << "\n";
    std::cout << "GT velocity: "
              << HardcodedGroundTruth::velocity().transpose() << "\n";

  } catch (const std::exception& e) {
    std::cerr << "EqVIOFilterCsvReplay failed: " << e.what() << "\n";
    return 2;
  }

  return 0;
}
