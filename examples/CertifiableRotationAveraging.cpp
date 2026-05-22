/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    CertifiableRotationAveraging.cpp
 * @brief   Certifiable SO(d) synchronization (d = 2 or 3) via the Riemannian
 *          Staircase (Burer–Monteiro low-rank SDP + sparse-eigensolver
 *          certificate). Minimizes Σ_ij κ_ij ‖R_i R_ij − R_j‖²_F over
 *          Rot2 / Rot3.
 * @author  Zhexin Xu
 * @author  David M. Rosen
 *
 * Usage:
 *   ./CertifiableRotationAveraging --data=<g2o file> --dim=<2|3>
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/dataset.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <type_traits>

using namespace gtsam;

namespace {

/// Random initialization of Rot2 / Rot3.
template <typename RotT, int IntrinsicDim>
Values RandomInitial(const std::set<Key>& keys, unsigned int seed) {
  Values v;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni(-M_PI, M_PI);
  for (Key key : keys) {
    if constexpr (std::is_same_v<RotT, Rot2>) {
      InsertQcqpValue<Rot2, IntrinsicDim>(key, Rot2::fromAngle(uni(rng)), &v);
    } else {
      const Vector3 rpy(uni(rng), uni(rng), uni(rng));
      InsertQcqpValue<Rot3, IntrinsicDim>(
          key, Rot3::RzRyRx(rpy(0), rpy(1), rpy(2)), &v);
    }
  }
  return v;
}

template <typename PoseT, typename RotT, int IntrinsicDim>
int RunCertifiableRA(const std::string& dataPath, unsigned int seed) {
  constexpr bool kIs3D = std::is_same_v<PoseT, Pose3>;

  // `readG2o` returns a pose graph of BetweenFactor<PoseT> edges. We rebuild
  // it as a rotation-averaging graph: drop the translation part, keep the
  // rotation measurement, and wrap it in a FrobeniusBetweenFactor<RotT>
  // with isotropic noise. Skipping `readG2o` and constructing the
  // FrobeniusBetweenFactor graph directly is equally fine.
  //
  // Rotation precision κ follows the SE-Sync convention:
  //   2D:  κ = I(θ,θ), read off Gaussian::information() directly.
  //   3D:  κ = d / (2·trace(RotInfo⁻¹)), with RotInfo the rotation block.
  auto [poseGraph, _] = readG2o(dataPath, /*is3D=*/kIs3D);
  NonlinearFactorGraph graph;
  std::set<Key> keys;
  for (const auto& f : *poseGraph) {
    const auto bf = std::dynamic_pointer_cast<BetweenFactor<PoseT>>(f);
    if (!bf) continue;

    const auto gaussian =
        std::dynamic_pointer_cast<noiseModel::Gaussian>(bf->noiseModel());

    double kappa;
    if constexpr (!kIs3D) {
      const Matrix3 info =
          gaussian ? gaussian->information() : Matrix3::Identity();
      kappa = info(2, 2);
    } else {
      const Matrix6 info =
          gaussian ? gaussian->information() : Matrix6::Identity();
      kappa = 3.0 / (2.0 * info.topLeftCorner<3, 3>().inverse().trace());
    }

    auto rotNoise =
        noiseModel::Isotropic::Variance(traits<RotT>::dimension, 1.0 / kappa);
    graph.emplace_shared<FrobeniusBetweenFactor<RotT>>(
        bf->key1(), bf->key2(), bf->measured().rotation(), rotNoise);
    keys.insert(bf->key1());
    keys.insert(bf->key2());
  }
  std::cout << "Loaded " << keys.size() << " rotations, " << graph.size()
            << " edges from " << dataPath << " (d=" << IntrinsicDim << ")\n\n";

  RiemannianStaircaseParams params;
  params.pMin = IntrinsicDim;  // starting rank should be ≥ IntrinsicDim.
  params.verificationMethod =
      RiemannianStaircaseParams::VerificationMethod::LOBPCG;
  params.verbose = true;
  // Certifiable methods need a high-precision solution to verify, so we
  // tighten the Augmented Lagrangian tolerances below the defaults.
  params.almParams->initialMuEq = 1e3;
  params.almParams->maxIterations = 200;
  params.almParams->absoluteViolationTolerance = 1e-10;
  params.almParams->relativeViolationTolerance = 1e-10;
  params.almParams->absoluteCostTolerance = 1e-10;
  params.almParams->relativeCostTolerance = 1e-10;

  // Convert the (QCQP-representable) factor graph into a QCQP problem.
  QcqpProblem qcqp(graph, params.pMin);

  // Riemannian Staircase with the Augmented Lagrangian as the inner local
  // solver for the low-rank-factorized problem.
  RiemannianStaircaseOptimizer rso(
      qcqp, RandomInitial<RotT, IntrinsicDim>(keys, seed), params);

  const auto t0 = std::chrono::steady_clock::now();
  const auto result = rso.optimize();
  const double wallSeconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();

  Values rounded;
  if (result.rounded) {
    for (auto& [key, R] : RiemannianStaircaseOptimizer::extractRotations<RotT>(
             *result.rounded, result.layout)) {
      rounded.insert(key, R);
    }
  }
  // ×2 to match the paper convention `Σ κ_ij ‖R_i R_ij − R_j‖²_F` that the
  // per-level `cost=...` line above prints (GTSAM's `graph.error()` returns
  // ½·Σ‖σ⁻¹·residual‖², so an explicit ×2 cancels the half).
  const double roundedObj =
      rounded.empty() ? std::nan("") : 2.0 * graph.error(rounded);

  std::cout << "\n========== Result ==========\n"
            << "Certified:        " << (result.certified ? "yes" : "no") << "\n"
            << "Final rank:       " << result.finalRank << "\n"
            << "Rounded objective: " << roundedObj << "\n"
            << "Wall time:        " << std::fixed << std::setprecision(3)
            << wallSeconds << " s\n";

  return result.certified ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  const auto usage = [&]() {
    std::cerr << "Usage: " << argv[0]
              << " --data=<g2o file> --dim=<2|3>\n";
  };
  std::string dataPath;
  int dim = 0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg.rfind("--data=", 0) == 0) {
      dataPath = arg.substr(7);
    } else if (arg.rfind("--dim=", 0) == 0) {
      dim = std::stoi(arg.substr(6));
    }
  }
  if (dataPath.empty() || (dim != 2 && dim != 3)) {
    usage();
    return 2;
  }

  if (dim == 3) {
    return RunCertifiableRA<Pose3, Rot3, 3>(dataPath, /*seed=*/30);
  } else {
    return RunCertifiableRA<Pose2, Rot2, 2>(dataPath, /*seed=*/42);
  }
}
