/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file MatrixWeightedLocalizationExample.cpp
 * @brief Locally solve a noiseless matrix-weighted localization problem.
 * @author Avinash Subramanian
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/KnownLandmarkFactor.h>

#include <iostream>
#include <vector>

using namespace gtsam;

int main() {
  constexpr size_t kNumPoses = 3;
  constexpr double kLateralSigma = 0.01;
  constexpr double kAnisotropicity = 10.0;
  constexpr double kOdometrySigma = 0.1;

  const Pose3 iTw;
  const Pose3 kTkp1(Rot3::RzRyRx(0.03, -0.05, 0.08), Point3(0.3, -0.1, 0.2));
  std::vector<Pose3> kTws{iTw};
  kTws.reserve(kNumPoses);
  for (size_t i = 1; i < kNumPoses; ++i) {
    kTws.push_back(kTws.back().compose(kTkp1));
  }

  const std::vector<Point3> wLs{Point3(4.0, 1.0, 2.0), Point3(-1.0, 3.0, 5.0),
                                Point3(2.0, -4.0, 3.0), Point3(5.0, 2.0, -1.0)};

  NonlinearFactorGraph graph;
  Values groundTruth;

  const double lateralPrecision = 1.0 / (kLateralSigma * kLateralSigma);
  const double radialPrecision =
      lateralPrecision / (kAnisotropicity * kAnisotropicity);

  // kTw maps world coordinates into sensor frame k, so transformFrom predicts
  // the sensor-frame point kP corresponding to world landmark wL.
  for (size_t k = 0; k < kTws.size(); ++k) {
    groundTruth.insert(k, kTws[k]);
    for (const Point3& wL : wLs) {
      const Point3 measured_kP = kTws[k].transformFrom(wL);
      // This simplified stereo-like model aligns the largest covariance axis
      // with the observation ray and sets sqrt(cond(Sigma)) to anisotropicity.
      const Vector3 kRay = measured_kP.normalized();
      const Matrix3 information =
          lateralPrecision * Matrix3::Identity() +
          (radialPrecision - lateralPrecision) * kRay * kRay.transpose();
      graph.emplace_shared<KnownLandmarkFactor<Pose3>>(
          k, wL, measured_kP, noiseModel::Gaussian::Information(information));
    }
  }

  const auto odometryNoiseModel =
      noiseModel::Isotropic::Sigma(Pose3::dimension, kOdometrySigma);
  for (size_t i = 0; i + 1 < kTws.size(); ++i) {
    const size_t j = i + 1;
    const Pose3 iTj = kTws[i].compose(kTws[j].inverse());
    graph.emplace_shared<FrobeniusLeftBetweenFactor<Pose3>>(i, j, iTj,
                                                            odometryNoiseModel);
  }

  Values initial;
  Vector6 perturbation;
  perturbation << 0.02, -0.015, 0.01, 0.08, -0.05, 0.06;
  for (size_t i = 0; i < kTws.size(); ++i) {
    initial.insert(i, kTws[i].retract((i + 1) * perturbation));
  }

  GaussNewtonParams parameters;
  parameters.setMaxIterations(100);
  parameters.setRelativeErrorTol(1e-12);
  const Values result =
      GaussNewtonOptimizer(graph, initial, parameters).optimize();

  const double groundTruthError = graph.error(groundTruth);
  const double initialError = graph.error(initial);
  const double finalError = graph.error(result);
  double maximumPoseError = 0.0;
  double totalPoseError = 0.0;
  for (size_t i = 0; i < kTws.size(); ++i) {
    const double poseError =
        kTws[i].localCoordinates(result.at<Pose3>(i)).norm();
    totalPoseError += poseError;
    if (poseError > maximumPoseError) maximumPoseError = poseError;
  }
  const double meanPoseError =
      totalPoseError / static_cast<double>(kTws.size());

  std::cout << "Ground-truth error: " << groundTruthError << '\n'
            << "Initial error: " << initialError << '\n'
            << "Final error: " << finalError << '\n'
            << "Mean pose error: " << meanPoseError << '\n'
            << "Maximum pose error: " << maximumPoseError << std::endl;

  constexpr double kErrorTolerance = 1e-8;
  if (groundTruthError > kErrorTolerance || finalError > kErrorTolerance ||
      maximumPoseError > kErrorTolerance) {
    std::cerr << "Matrix-weighted localization did not recover ground truth."
              << std::endl;
    return 1;
  }

  return 0;
}
