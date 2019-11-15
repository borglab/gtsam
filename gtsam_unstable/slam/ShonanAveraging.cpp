/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanAveraging.cpp
 * @date   March 2019
 * @author Frank Dellaert
 * @brief  Shonan Averaging algorithm
 */

#include <gtsam_unstable/slam/ShonanAveraging.h>

#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>
#include <gtsam_unstable/slam/FrobeniusFactor.h>

#include <Eigen/Eigenvalues>

#include <complex>
#include <iostream>
#include <map>
#include <random>

namespace gtsam {

static std::mt19937 kRandomNumberGenerator(42);

/* ************************************************************************* */
ShonanAveragingParameters::ShonanAveragingParameters(const string& verbosity,
                                                     const std::string& method,
                                                     double noiseSigma)
    : prior(true), karcher(true), noiseSigma(noiseSigma) {
  lm.setVerbosityLM(verbosity);

  // Set parameters to be similar to ceres
  LevenbergMarquardtParams::SetCeresDefaults(&lm);

  // By default, we will do conjugate gradient
  lm.linearSolverType = LevenbergMarquardtParams::Iterative;

  // Create subgraph builder parameters
  SubgraphBuilderParameters builderParameters;
  builderParameters.skeletonType = SubgraphBuilderParameters::KRUSKAL;
  builderParameters.skeletonWeight = SubgraphBuilderParameters::EQUAL;
  builderParameters.augmentationWeight = SubgraphBuilderParameters::SKELETON;
  builderParameters.augmentationFactor = 0.0;

  auto pcg = boost::make_shared<PCGSolverParameters>();

  // Choose optimization method
  if (method == "SUBGRAPH") {
    lm.iterativeParams =
        boost::make_shared<SubgraphSolverParameters>(builderParameters);
  } else if (method == "SGPC") {
    pcg->preconditioner_ =
        boost::make_shared<SubgraphPreconditionerParameters>(builderParameters);
    lm.iterativeParams = pcg;
  } else if (method == "JACOBI") {
    pcg->preconditioner_ =
        boost::make_shared<BlockJacobiPreconditionerParameters>();
    lm.iterativeParams = pcg;
  } else if (method == "QR") {
    lm.setLinearSolverType("MULTIFRONTAL_QR");
  } else if (method == "CHOLESKY") {
    lm.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
  } else {
    throw std::invalid_argument("ShonanAveragingParameters: unknown method");
  }
}

/* ************************************************************************* */
ShonanAveraging::ShonanAveraging(const string& g2oFile,
                                 const ShonanAveragingParameters& parameters)
    : parameters_(parameters) {
  auto corruptingNoise = noiseModel::Isotropic::Sigma(3, parameters.noiseSigma);
  factors_ = parse3DFactors(g2oFile, corruptingNoise);
  poses_ = parse3DPoses(g2oFile);
  Q_ = buildQ();
}

/* ************************************************************************* */
NonlinearFactorGraph ShonanAveraging::buildGraphAt(size_t p) const {
  NonlinearFactorGraph graph;
  for (const auto& factor : factors_) {
    const auto& keys = factor->keys();
    const auto& Tij = factor->measured();
    const auto& model = factor->noiseModel();
    graph.emplace_shared<FrobeniusWormholeFactor>(
        keys[0], keys[1], SO3(Tij.rotation().matrix()), p, model);
  }
  return graph;
}

/* ************************************************************************* */
Values ShonanAveraging::initializeRandomlyAt(size_t p) const {
  Values initial;
  for (size_t j = 0; j < nrPoses(); j++) {
    initial.insert(j, SOn::Random(kRandomNumberGenerator, p));
  }
  return initial;
}

/* ************************************************************************* */
static Values Optimize(const NonlinearFactorGraph& graph, const Values& initial,
                       const LevenbergMarquardtParams& LMparameters) {
  LevenbergMarquardtOptimizer lm(graph, initial, LMparameters);
  Values result = lm.optimize();
  return result;
}

/* ************************************************************************* */
double ShonanAveraging::costAt(size_t p, const Values& values) const {
  const NonlinearFactorGraph graph = buildGraphAt(p);
  return graph.error(values);
}

/* ************************************************************************* */
Values ShonanAveraging::tryOptimizingAt(
    size_t p, const boost::optional<const Values&> initialEstimate) const {
  // Build graph
  auto graph = buildGraphAt(p);

  // Initialize randomly if no initial estimate is given
  // TODO(frank): add option to do chordal init
  const Values initial =
      initialEstimate ? *initialEstimate : initializeRandomlyAt(p);

  // Prior is only added here as depends on initial value (and cost is zero)
  if (parameters_.prior) {
    if (parameters_.karcher) {
      const size_t d = p * (p - 1) / 2;
      graph.emplace_shared<KarcherMeanFactor<SOn>>(graph.keys(), d);
    } else {
      graph.emplace_shared<PriorFactor<SOn>>(0, initial.at<SOn>(0));
    }
  }

  // Optimize
  return Optimize(graph, initial, parameters_.lm);
}

/* ************************************************************************* */
Values ShonanAveraging::projectFrom(size_t p, const Values& values) const {
  Values SO3_values;
  for (size_t j = 0; j < nrPoses(); j++) {
    const SOn Q = values.at<SOn>(j);
    assert(Q.rows() == p);
    const SO3 R = SO3::ClosestTo(Q.matrix().topLeftCorner(3, 3));
    SO3_values.insert(j, R);
  }
  return SO3_values;
}

/* ************************************************************************* */
double ShonanAveraging::cost(const Values& values) const {
  NonlinearFactorGraph graph;
  for (const auto& factor : factors_) {
    const auto& keys = factor->keys();
    const auto& Tij = factor->measured();
    const auto& model = factor->noiseModel();
    graph.emplace_shared<FrobeniusBetweenFactor<SO3>>(
        keys[0], keys[1], SO3(Tij.rotation().matrix()), model);
  }
  return graph.error(values);
}

/* ************************************************************************* */
// heavily cribbed from David's construct_rotational_connection_Laplacian
ShonanAveraging::Sparse ShonanAveraging::buildQ(bool useNoiseModel) const {
  assert(useNoiseModel == false);

  constexpr size_t d = 3;  // for now only for 3D rotations

  // Each measurement contributes 2*d elements along the diagonal of the
  // connection Laplacian, and 2*d^2 elements on a pair of symmetric
  // off-diagonal blocks
  constexpr size_t stride = 2 * (d + d * d);

  // Reserve space for triplets
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * factors_.size());

  for (const auto& factor : factors_) {
    // Get pose keys
    const auto& keys = factor->keys();
    size_t i = keys[0];
    size_t j = keys[1];

    // Extract rotation measurement
    const auto& Tij = factor->measured();
    const Matrix3 Rij = Tij.rotation().matrix();

    // Get kappa from noise model
    double kappa;
    if (useNoiseModel) {
      // const auto& m = factor->noiseModel();
      // isotropic = noiseModel_FrobeniusNoiseModel9.FromPose3NoiseModel(m)
      // sigma = isotropic.sigma()
      // kappa_ij = 1.0/(sigma*sigma)
    } else {
      kappa = 1.0;
    }

    // Elements of ith block-diagonal
    for (size_t k = 0; k < d; k++)
      triplets.emplace_back(d * i + k, d * i + k, kappa);

    // Elements of jth block-diagonal
    for (size_t k = 0; k < d; k++)
      triplets.emplace_back(d * j + k, d * j + k, kappa);

    // Elements of ij block
    for (size_t r = 0; r < d; r++)
      for (size_t c = 0; c < d; c++)
        triplets.emplace_back(i * d + r, j * d + c, -kappa * Rij(r, c));

    // Elements of ji block
    for (size_t r = 0; r < d; r++)
      for (size_t c = 0; c < d; c++)
        triplets.emplace_back(j * d + r, i * d + c, -kappa * Rij(c, r));
  }

  // Construct and return a sparse matrix from these triplets
  const size_t N = nrPoses();
  ShonanAveraging::Sparse LGrho(d * N, d * N);
  LGrho.setFromTriplets(triplets.begin(), triplets.end());

  return LGrho;
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::computeLambda(const Values& values) const {
  constexpr size_t d = 3;  // for now only for 3D rotations

  // Each pose contributes 2*d elements along the diagonal of Lambda
  constexpr size_t stride = d * d;

  // Reserve space for triplets
  const size_t N = nrPoses();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * N);

  // Project to pxdN Stiefel manifold
  const size_t p = values.at<SOn>(0).rows();
  Matrix S(p, N * d);
  for (size_t j = 0; j < N; j++) {
    const SOn Q = values.at<SOn>(j);
    S.block(0, j * d, p, d) = Q.matrix().leftCols(d);  // project Qj to Stiefel
  }

  // Do sparse-dense multiply to get Q*S'
  auto QSt = Q_ * S.transpose();

  for (size_t j = 0; j < N; j++) {
    // Compute B, the building block for the j^th diagonal block of Lambda
    Matrix B = QSt.middleRows(j, d) * S.middleCols(j, d);

    // Elements of jth block-diagonal
    for (size_t r = 0; r < d; r++)
      for (size_t c = 0; c < d; c++)
        triplets.emplace_back(j * d + r, j * d + c, 0.5 * (B(r, c) + B(c, r)));
  }

  // Construct and return a sparse matrix from these triplets
  ShonanAveraging::Sparse Lambda(d * N, d * N);
  Lambda.setFromTriplets(triplets.begin(), triplets.end());
  return Lambda;
}

/* ************************************************************************* */
double ShonanAveraging::computeMinEigenValue(const Values& values) const {
  /// Based on Luca's MATLAB version on BitBucket repo.
  assert(values.size() == nrPoses());
  auto Lambda = computeLambda(values);
  auto A = Q_ - Lambda;
  Eigen::EigenSolver<Matrix> solver(Matrix(A), false);
  auto lambdas = solver.eigenvalues();
  double lambda_min = lambdas(0).real();
  for (size_t i = 1; i < lambdas.size(); i++) {
    lambda_min = min(lambdas(i).real(), lambda_min);
  }
  return lambda_min;
}

/* ************************************************************************* */
bool ShonanAveraging::checkOptimality(const Values& values) const {
  double lambda_min = computeMinEigenValue(values);
  return lambda_min > -1e-4;  // TODO(frank): move tolerance to params
}

/* ************************************************************************* */
void ShonanAveraging::run(size_t p_max) const {
  // TODO(frank): check optimality and return optimal values
  for (size_t p = 3; p <= p_max; p++) {
    tryOptimizingAt(p);
  }
}

/* ************************************************************************* */
}  // namespace gtsam
