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
// Project to pxdN Stiefel manifold
static Matrix StiefelElementMatrix(const Values& values) {
  constexpr size_t d = 3;  // for now only for 3D rotations
  const size_t N = values.size();
  const size_t p = values.at<SOn>(0).rows();
  Matrix S(p, N * d);
  for (size_t j = 0; j < N; j++) {
    const SOn Q = values.at<SOn>(j);
    S.block(0, j * d, p, d) = Q.matrix().leftCols(d);  // project Qj to Stiefel
  }
  return S;
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::computeLambda(
    const Values& values) const {
  constexpr size_t d = 3;  // for now only for 3D rotations

  // Each pose contributes 2*d elements along the diagonal of Lambda
  constexpr size_t stride = d * d;

  // Reserve space for triplets
  const size_t N = nrPoses();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * N);

  // Project to pxdN Stiefel manifold
  Matrix S = StiefelElementMatrix(values);

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
/// MINIMUM EIGENVALUE COMPUTATIONS

#include "Spectra/SymEigsSolver.h"

using SparseMatrix = ShonanAveraging::Sparse;

/** This is a lightweight struct used in conjunction with Spectra to compute
 * the minimum eigenvalue and eigenvector of a sparse matrix A; it has a single
 * nontrivial function, perform_op(x,y), that computes and returns the product
 * y = (A + sigma*I) x */
struct MatrixProdFunctor {
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const SparseMatrix& A_;

  // Spectral shift
  double sigma_;

  // Constructor
  MatrixProdFunctor(const SparseMatrix& A, double sigma = 0)
      : A_(A), sigma_(sigma) {}

  int rows() const { return A_.rows(); }
  int cols() const { return A_.cols(); }

  // Matrix-vector multiplication operation
  void perform_op(const double* x, double* y) const {
    // Wrap the raw arrays as Eigen Vector types
    Eigen::Map<const Vector> X(x, rows());
    Eigen::Map<Vector> Y(y, rows());

    // Do the multiplication using wrapped Eigen vectors
    Y = A_ * X;
  }
};

/// Function to compute the minimum eigenvalue of A using Lanczos in Spectra.
/// This does 2 things:
///
/// (1)  Quick (coarse) eigenvalue computation to estimate the largest-magnitude
/// eigenvalue (2)  A second eigenvalue computation applied to A-sigma*I, where
/// sigma is chosen to make the minimum eigenvalue of A the extremal eigenvalue
/// of A-sigma*I
///
/// Upon completion, this returns a boolean value indicating whether the minimum
/// eigenvalue was computed to the required precision -- if so, its sets the
/// values of min_eigenvalue and min_eigenvector appropriately

/// Note that in the following function signature, Y is supposed to be the
/// block-row-matrix that is a critical point for the optimization algorithm;
/// either S (Stiefel manifold) or R (block rotations).  We use this to
/// construct a starting vector v for the Lanczos process that will be close to
/// the minimum eigenvector we're looking for whenever the relaxation is exact
/// -- this is a key feature that helps to make this method fast.  Note that
/// instead of passing in all of Y, it would be enough to pass in one of Y's
/// *rows*, if that's more convenient.

// For the defaults, David Rosen says:
//   - max_iterations refers to the max number of Lanczos iterations to run;
//   ~1000 should be sufficiently large
//   - We've been using 10^-4 for the nonnegativity tolerance
//   - for num_Lanczos_vectors, 20 is a good default value

bool A_min_eig(const SparseMatrix& A, const Matrix& Y, double& min_eigenvalue,
               Vector& min_eigenvector, size_t& num_iterations,
               size_t max_iterations = 1000,
               double min_eigenvalue_nonnegativity_tolerance = 10e-4,
               Eigen::Index num_Lanczos_vectors = 20) {
  // a. Estimate the largest-magnitude eigenvalue of this matrix using Lanczos
  MatrixProdFunctor lm_op(A);
  Spectra::SymEigsSolver<double, Spectra::SELECT_EIGENVALUE::LARGEST_MAGN,
                         MatrixProdFunctor>
      largest_magnitude_eigensolver(&lm_op, 1,
                                    std::min(num_Lanczos_vectors, A.rows()));
  largest_magnitude_eigensolver.init();

  int num_converged = largest_magnitude_eigensolver.compute(
      max_iterations, 1e-4, Spectra::SELECT_EIGENVALUE::LARGEST_MAGN);

  // Check convergence and bail out if necessary
  if (num_converged != 1) return false;

  double lambda_lm = largest_magnitude_eigensolver.eigenvalues()(0);

  if (lambda_lm < 0) {
    // The largest-magnitude eigenvalue is negative, and therefore also the
    // minimum eigenvalue, so just return this solution
    min_eigenvalue = lambda_lm;
    min_eigenvector = largest_magnitude_eigensolver.eigenvectors(1);
    min_eigenvector.normalize();  // Ensure that this is a unit vector
    return true;
  }

  // The largest-magnitude eigenvalue is positive, and is therefore the
  // maximum  eigenvalue.  Therefore, after shifting the spectrum of A
  // by -2*lambda_lm (by forming A - 2*lambda_max*I), the  shifted
  // spectrum will lie in the interval [lambda_min(A) - 2* lambda_max(A),
  // -lambda_max*A]; in particular, the largest-magnitude eigenvalue of
  //  A - 2*lambda_max*I is lambda_min - 2*lambda_max, with  corresponding
  // eigenvector v_min

  MatrixProdFunctor min_shifted_op(A, -2 * lambda_lm);

  Spectra::SymEigsSolver<double, Spectra::SELECT_EIGENVALUE::LARGEST_MAGN,
                         MatrixProdFunctor>
      min_eigensolver(&min_shifted_op, 1,
                      std::min(num_Lanczos_vectors, A.rows()));

  // If Y is a critical point of F, then Y^T is also in the null space of S -
  // Lambda(Y) (cf. Lemma 6 of the tech report), and therefore its rows are
  // eigenvectors corresponding to the eigenvalue 0.  In the case  that the
  // relaxation is exact, this is the *minimum* eigenvalue, and therefore the
  // rows of Y are exactly the eigenvectors that we're looking for.  On the
  // other hand, if the relaxation is *not* exact, then S - Lambda(Y) has at
  // least one strictly negative eigenvalue, and the rows of Y are *unstable
  // fixed points* for the Lanczos iterations.  Thus, we will take a slightly
  // "fuzzed" version of the first row of Y as an initialization for the Lanczos
  // iterations; this allows for rapid convergence in the case that the
  // relaxation is exact (since are starting close to a solution), while
  // simultaneously allowing the iterations to escape from this fixed point in
  // the case that the relaxation is not exact.
  Vector v0 = Y.row(0).transpose();
  Vector perturbation(v0.size());
  perturbation.setRandom();
  perturbation.normalize();
  Vector xinit = v0 + (.03 * v0.norm()) * perturbation;  // Perturb v0 by ~3%

  // Use this to initialize the eigensolver
  min_eigensolver.init(xinit.data());

  // Now determine the relative precision required in the Lanczos method in
  // order to be able to estimate the smallest eigenvalue within an *absolute*
  // tolerance of 'min_eigenvalue_nonnegativity_tolerance'
  num_converged = min_eigensolver.compute(
      max_iterations, min_eigenvalue_nonnegativity_tolerance / lambda_lm,
      Spectra::SELECT_EIGENVALUE::LARGEST_MAGN);

  if (num_converged != 1) return false;

  min_eigenvector = min_eigensolver.eigenvectors(1);
  min_eigenvector.normalize();  // Ensure that this is a unit vector
  min_eigenvalue = min_eigensolver.eigenvalues()(0) + 2 * lambda_lm;
  num_iterations = min_eigensolver.num_iterations();
  return true;
}

/* ************************************************************************* */
double ShonanAveraging::computeMinEigenValue(const Values& values) const {
  /// Based on Luca's MATLAB version on BitBucket repo.
  assert(values.size() == nrPoses());
  auto Lambda = computeLambda(values);
  auto A = Q_ - Lambda;
#ifdef SLOW_EIGEN_COMPUTATION
  Eigen::EigenSolver<Matrix> solver(Matrix(A), false);
  auto lambdas = solver.eigenvalues();
  double lambda_min = lambdas(0).real();
  for (size_t i = 1; i < lambdas.size(); i++) {
    lambda_min = min(lambdas(i).real(), lambda_min);
  }
#else
  // Project to pxdN Stiefel manifold, TODO(frank) already done in computeLambda
  Matrix S = StiefelElementMatrix(values);
  double lambda_min;
  Vector min_eigenvector;
  size_t num_iterations;
  bool success = A_min_eig(A, S, lambda_min, min_eigenvector, num_iterations);
  if (!success) {
    throw std::runtime_error("A_min_eig failed to compute minimum eigenvalue.");
  }
#endif
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
