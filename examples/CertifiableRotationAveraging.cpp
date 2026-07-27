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
 *          certificate). Minimizes Σ_ij κ_ij ‖R_i R_ij − R_j‖²_F over Rot2 / Rot3.
 * @author  Zhexin Xu
 * @author  David M. Rosen
 *
 * Usage:
 *   ./CertifiableRotationAveraging --data=<g2o file> --dim=<2|3>
 */

#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/dataset.h>

#include <chrono>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <type_traits>

using namespace gtsam;

namespace {

constexpr unsigned int kSeed = 60;
constexpr double kPi = 3.14159265358979323846;

/// If less than half of the `D × D` entries have positive determinant, negate
/// every entry's last column. Required before per-block `Rot::ClosestTo`
/// projects each block to SO(d) — otherwise its sign-correction fires only on
/// the det < 0 blocks and the rounded rotations end up with inconsistent
/// signs across keys.
template <int D>
void AlignBlockDetSigns(Values& qcqpValues) {
  size_t numNeg = 0, numBlocks = 0;
  for (const auto& [key, M] : qcqpValues.extract<Matrix>()) {
    if (M.rows() != D) continue;
    ++numBlocks;
    if (M.determinant() < 0) ++numNeg;
  }
  if (numBlocks == 0 || numNeg <= numBlocks / 2) return;
  for (const auto& [key, M] : qcqpValues.extract<Matrix>()) {
    if (M.rows() != D) continue;
    Matrix flipped = M;
    flipped.col(M.cols() - 1) *= -1.0;
    qcqpValues.update(key, flipped);
  }
}

/// Random Rot2 / Rot3 init at column count `IntrinsicDim` (D ≥ ambient dim).
template <typename RotT, int IntrinsicDim>
Values RandomInitial(const std::set<Key>& keys) {
  Values v;
  std::mt19937 rng(kSeed);
  std::uniform_real_distribution<double> uni(-kPi, kPi);
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
int RunCertifiableRA(const std::string& dataPath) {
  constexpr bool kIs3D = std::is_same_v<PoseT, Pose3>;

  // Drop translations, wrap each BetweenFactor<PoseT>'s rotation in a
  // FrobeniusBetweenFactor<RotT> with isotropic precision kappa (SE-Sync):
  //   2D:  kappa = I(theta, theta)
  //   3D:  kappa = d / (2 * trace(RotInfo^-1))
  // g2o initial values are discarded; we use random init below.
  auto [poseGraph, _] = readG2o(dataPath, /*is3D=*/kIs3D);
  NonlinearFactorGraph graph;
  std::set<Key> keys;
  size_t skipped = 0;
  for (const auto& f : *poseGraph) {
    const auto bf = std::dynamic_pointer_cast<BetweenFactor<PoseT>>(f);
    if (!bf) {
      ++skipped;
      continue;
    }

    const auto gaussian =
        std::dynamic_pointer_cast<noiseModel::Gaussian>(bf->noiseModel());

    double kappa;
    if constexpr (kIs3D) {
      const Matrix6 info =
          gaussian ? gaussian->information() : Matrix6::Identity();
      kappa = 3.0 / (2.0 * info.topLeftCorner<3, 3>().inverse().trace());
    } else {
      const Matrix3 info =
          gaussian ? gaussian->information() : Matrix3::Identity();
      kappa = info(2, 2);
    }

    auto rotNoise =
        noiseModel::Isotropic::Variance(traits<RotT>::dimension, 1.0 / kappa);
    graph.emplace_shared<FrobeniusBetweenFactor<RotT>>(
        bf->key1(), bf->key2(), bf->measured().rotation(), rotNoise);
    keys.insert(bf->key1());
    keys.insert(bf->key2());
  }

  if (graph.empty()) {
    std::cerr << "No BetweenFactor<" << (kIs3D ? "Pose3" : "Pose2")
              << "> found in " << dataPath
              << ". Wrong --dim, or wrong file format?\n";
    return 1;
  }
  std::cout << "Loaded " << keys.size() << " rotations, " << graph.size()
            << " edges from " << dataPath << " (d=" << IntrinsicDim << ")";
  if (skipped > 0) std::cout << "  (" << skipped << " non-Between skipped)";
  std::cout << "\n\n";

  // Low initial mu + slow growth lets LM make cost progress before the
  // constraints tighten. Spectra verifier + ALM solver are the defaults.
  RiemannianStaircaseParams params;
  params.pMin = IntrinsicDim;
  params.verbose = true;
  params.almParams->initialMuEq = 1.0;
  params.almParams->muEqIncreaseRate = 2.0;
  params.almParams->maxIterations = 50;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->relativeViolationTolerance = 1e-8;
  params.almParams->absoluteCostTolerance = 1e-10;
  params.almParams->relativeCostTolerance = 1e-10;
  params.almParams->verbose = false;

  // graph must be QCQP-representable — constructor probes once and throws.
  RiemannianStaircaseOptimizer rso(
      graph, RandomInitial<RotT, IntrinsicDim>(keys), params);

  const auto t0 = std::chrono::steady_clock::now();
  const auto result = rso.optimize();
  const double wallSeconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();

  Values rounded;
  if (result.rounded) {
    // unstack → gauge-align → per-block project to RotT.
    Values atRankD = result.layout.unstack(result.rounded->Yd);
    AlignBlockDetSigns<IntrinsicDim>(atRankD);
    for (auto& [key, R] :
         ExtractQcqpValues<RotT, IntrinsicDim>(atRankD)) {
      rounded.insert(key, R);
    }
  }
  // graph.error is GTSAM's 0.5-NLL form; multiply by 2 for SE-Sync's paper F.
  std::cout << "\n========== Result ==========\n"
            << "Certified:         " << (result.certified ? "yes" : "no") << "\n"
            << "Final rank:        " << result.finalRank << "\n";
  if (rounded.empty()) {
    std::cout << "Rounded objective: n/a\n";
  } else {
    std::cout << "Rounded objective: " << graph.error(rounded) << "\n";
  }
  std::cout << "Wall time:         " << wallSeconds << " s\n";

  return result.certified ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
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
    std::cerr << "Usage: " << argv[0] << " --data=<g2o file> --dim=<2|3>\n";
    return 2;
  }

  if (dim == 3) {
    return RunCertifiableRA<Pose3, Rot3, 3>(dataPath);
  } else {
    return RunCertifiableRA<Pose2, Rot2, 2>(dataPath);
  }
}
