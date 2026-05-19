/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Rot2SynchronizationExample.cpp
 * @brief   Small angular synchronization example demonstrating the
 *          Riemannian Staircase certifiable optimizer on Rot2.
 *
 * Problem: recover absolute orientations of N=4 robot poses arranged on a
 * ring, given noisy relative rotation measurements between consecutive poses
 * (including the loop-closing edge x3 -> x0).
 *
 * The optimizer returns a Burer-Monteiro solution (matrix-valued Values) plus
 * a certificate of (near-)optimality for the underlying SDP relaxation.
 *
 * Usage:
 *   ./Rot2SynchronizationExample
 *       [--init=<perturbed|random|wrong>]  (default: random)
 *       [--verify=<lobpcg|dense>]          (default: lobpcg)
 *       [--seed=<N>]                       (default: 42; init RNG seed)
 *       [--noiseSeed=<N>]                  (default: 42; measurement-noise RNG)
 *
 *   Initialization strategies:
 *     --init=perturbed   ground-truth angles + small per-pose perturbation.
 *     --init=random      uniform random in [-pi, pi]; seeded by --seed.
 *     --init=wrong       -i * deltaAngle (opposite winding).
 *
 *   Verification methods:
 *     --verify=lobpcg    scalable fast_verification (LOBPCG + Cholmod + ILDL).
 *     --verify=dense     dense Eigen::SelfAdjointEigenSolver.
 */

#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>

#include <Eigen/SVD>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace gtsam;

namespace {

enum class InitKind { Perturbed, Random, Wrong };

struct CliOptions {
  InitKind init = InitKind::Random;
  RiemannianStaircaseParams::VerificationMethod verify =
      RiemannianStaircaseParams::VerificationMethod::LOBPCG;
  unsigned int seed = 42;       // init RNG (used by random)
  unsigned int noiseSeed = 42;  // measurement-noise RNG
};

bool startsWith(const std::string& s, const std::string& prefix) {
  return s.size() >= prefix.size() &&
         s.compare(0, prefix.size(), prefix) == 0;
}

CliOptions parseCli(int argc, char** argv) {
  CliOptions opts;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (startsWith(a, "--init=")) {
      const std::string v = a.substr(7);
      if (v == "perturbed") opts.init = InitKind::Perturbed;
      else if (v == "random") opts.init = InitKind::Random;
      else if (v == "wrong") opts.init = InitKind::Wrong;
      else {
        std::cerr << "Unknown --init value: " << v
                  << " (perturbed|random|wrong)\n";
        std::exit(2);
      }
    } else if (startsWith(a, "--verify=")) {
      const std::string v = a.substr(9);
      if (v == "lobpcg")
        opts.verify = RiemannianStaircaseParams::VerificationMethod::LOBPCG;
      else if (v == "dense")
        opts.verify = RiemannianStaircaseParams::VerificationMethod::DenseEigen;
      else {
        std::cerr << "Unknown --verify value: " << v << " (lobpcg|dense)\n";
        std::exit(2);
      }
    } else if (startsWith(a, "--seed=")) {
      opts.seed = static_cast<unsigned int>(std::stoul(a.substr(7)));
    } else if (startsWith(a, "--noiseSeed=")) {
      opts.noiseSeed = static_cast<unsigned int>(std::stoul(a.substr(12)));
    } else {
      std::cerr << "Unknown flag: " << a << "\n";
      std::exit(2);
    }
  }
  return opts;
}

const char* initName(InitKind k) {
  switch (k) {
    case InitKind::Perturbed: return "perturbed";
    case InitKind::Random:    return "random";
    case InitKind::Wrong:     return "wrong";
  }
  return "?";
}

const char* verifyName(RiemannianStaircaseParams::VerificationMethod m) {
  switch (m) {
    case RiemannianStaircaseParams::VerificationMethod::LOBPCG: return "lobpcg";
    case RiemannianStaircaseParams::VerificationMethod::DenseEigen:
      return "dense";
  }
  return "?";
}

/// Round the BM solution (D=2 row-orthonormal form) at rank p back to one
/// Rot2 per variable. Each variable's value is r=2 rows by p columns; at
/// p=2 each 2x2 block is approximately R^T, and at p>2 we project to that
/// form via rank-d=2 truncated SVD. Then sign-fix by majority vote on the
/// per-block determinants and SO(2)-project each block.
std::vector<Rot2> roundToRot2(
    const Values& Y,
    const RiemannianStaircaseOptimizer::Layout& layout) {
  // 1. Assemble the global (totalDim x p) Y.
  const Matrix Yglobal = layout.stack(Y);

  // 2. Rank-d=2 truncated SVD: collapse any extra (lifted) columns back to
  // the natural 2D rotation column count.
  Eigen::JacobiSVD<Matrix> svd(Yglobal,
                               Eigen::ComputeFullU | Eigen::ComputeFullV);
  const int d = 2;
  Matrix Y2 = svd.matrixU().leftCols(d) *
              svd.singularValues().head(d).asDiagonal();
  // Y2 is (totalDim x 2). Each 2-row block is approximately R^T.

  // 3. Global sign gauge (see W100 example for the full derivation): flip
  // one column of Y2 if majority of per-block determinants are negative.
  const size_t N = layout.size();
  size_t numNegDet = 0;
  for (const auto& [_, slice] : layout.slices) {
    const Matrix2 block = Y2.block(slice.offset, 0, 2, 2);
    if (block.determinant() < 0) ++numNegDet;
  }
  if (numNegDet > N / 2) Y2.col(1) *= -1.0;

  // 4. SO(2)-project each block. Block ≈ R^T, so R ≈ block^T.
  std::vector<Rot2> rotations;
  rotations.reserve(N);
  for (const auto& [_, slice] : layout.slices) {
    const Matrix2 block = Y2.block(slice.offset, 0, 2, 2);
    rotations.push_back(Rot2::ClosestTo(block.transpose()));
  }
  return rotations;
}

Values makeInitial(size_t N, double deltaAngle, InitKind kind,
                   unsigned int seed) {
  Values v;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uni(-M_PI, M_PI);
  for (size_t i = 0; i < N; ++i) {
    double angle = 0.0;
    switch (kind) {
      case InitKind::Perturbed:
        angle = i * deltaAngle + 0.01 * static_cast<double>(i);
        break;
      case InitKind::Random:
        angle = uni(rng);
        break;
      case InitKind::Wrong:
        angle = -static_cast<double>(i) * deltaAngle;
        break;
    }
    InsertQcqpValue<Rot2, 2>(Symbol('x', i), Rot2::fromAngle(angle), &v);
  }
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  const CliOptions opts = parseCli(argc, argv);
  std::cout << "init=" << initName(opts.init)
            << " verify=" << verifyName(opts.verify)
            << " seed=" << opts.seed
            << " noiseSeed=" << opts.noiseSeed << "\n\n";

  // -------------------------------------------------------------------
  // 1. Ground truth: 4 rotations evenly spaced around the unit circle.
  // -------------------------------------------------------------------
  constexpr size_t N = 100;
  constexpr double deltaAngle = 2.0 * M_PI / N;  // 90 degrees per step
  std::vector<Rot2> truth;
  for (size_t i = 0; i < N; ++i) {
    truth.push_back(Rot2::fromAngle(i * deltaAngle));
  }
  std::cout << "Ground truth angles (deg):";
  for (const auto& R : truth) std::cout << " " << R.degrees();
  std::cout << "\n";

  // -------------------------------------------------------------------
  // 2. Factor graph: noisy relative rotation measurements on a ring.
  // -------------------------------------------------------------------
  std::mt19937 noiseRng(opts.noiseSeed);
  std::normal_distribution<double> noise(0.0, 0.05);  // ~1.1 deg stddev

  NonlinearFactorGraph graph;
  for (size_t i = 0; i < N; ++i) {
    const size_t j = (i + 1) % N;
    const double measured = deltaAngle + noise(noiseRng);
    graph.emplace_shared<FrobeniusBetweenFactor<Rot2>>(
        Symbol('x', i), Symbol('x', j), Rot2::fromAngle(measured));
  }

  // -------------------------------------------------------------------
  // 3. Initialization, selected by the --init flag.
  // -------------------------------------------------------------------
  const Values initial = makeInitial(N, deltaAngle, opts.init, opts.seed);

  // -------------------------------------------------------------------
  // 4. Configure and run the staircase optimizer.
  // -------------------------------------------------------------------
  RiemannianStaircaseParams params;
  // Matrix-form K=2 is the base of the staircase ladder for Rot2.
  params.pMin = 2;
  params.pMax = 5;
  params.alpha = 1e-1;     // step along the descent direction
  params.eta = 1e-3;       // certificate tolerance
  params.verbose = true;   // print per-level diagnostics
  params.verificationMethod = opts.verify;
  params.almParams->maxIterations = 200;
  params.almParams->absoluteViolationTolerance = 1e-8;
  params.almParams->relativeViolationTolerance = 1e-8;

  RiemannianStaircaseOptimizer rso(graph, initial, params);
  const auto result = rso.optimize();

  // -------------------------------------------------------------------
  // 5. Report the certificate and the rank progression.
  // -------------------------------------------------------------------
  std::cout << "\n========== Result ==========\n";
  std::cout << "Certified:      " << (result.certified ? "yes" : "no") << "\n";
  std::cout << "Final rank:     " << result.finalRank << "\n";
  std::cout << "Min eig(S):     " << result.minEigenvalue << "\n";
  std::cout << "Ranks visited: ";
  for (auto p : result.ranksVisited) std::cout << " " << p;
  std::cout << "\nCost per level:";
  for (auto c : result.costPerLevel) std::cout << " " << c;
  std::cout << "\n";

  // -------------------------------------------------------------------
  // 6. Round the BM solution back to Rot2 and compare with ground truth.
  //    Rotation averaging has a global gauge (any solution composed with
  //    a global rotation is equally valid), so we anchor on pose 0.
  // -------------------------------------------------------------------
  const auto layout =
      RiemannianStaircaseOptimizer::Layout::From(result.values);
  auto recovered = roundToRot2(result.values, layout);

  // x0_recovered = truth[0] * gauge  =>  gauge = truth[0]^-1 * recovered[0].
  const Rot2 gauge = truth[0].between(recovered[0]);

  std::cout << "\nRecovered vs ground truth (deg, gauge-aligned to pose 0):\n";
  std::cout << std::fixed << std::setprecision(3);
  for (size_t i = 0; i < N; ++i) {
    const Rot2 aligned = gauge.inverse() * recovered[i];
    std::cout << "  pose " << i
              << ":  recovered = " << std::setw(8) << aligned.degrees()
              << "    truth = " << std::setw(8) << truth[i].degrees() << "\n";
  }
  return result.certified ? 0 : 1;
}
