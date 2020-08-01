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

#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>
#include <gtsam_unstable/slam/ShonanAveraging.h>
#include <gtsam_unstable/slam/ShonanGaugeFactor.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <complex>
#include <iostream>
#include <map>
#include <random>
#include <vector>

namespace gtsam {

static std::mt19937 kRandomNumberGenerator(42);

/* ************************************************************************* */
ShonanAveragingParameters::ShonanAveragingParameters(
    const LevenbergMarquardtParams &_lm, const std::string &method,
    double noiseSigma, double optimalityThreshold)
    : prior(true), karcher(true), fixGauge(false), noiseSigma(noiseSigma),
      optimalityThreshold(optimalityThreshold), lm(_lm) {
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
ShonanAveraging::ShonanAveraging(const BetweenFactorPose3s& factors,
                                 const std::map<Key, Pose3>& poses,
                                 const ShonanAveragingParameters& parameters)
    : parameters_(parameters), factors_(factors), poses_(poses), d_(3) {
  // TODO(frank): add noise here rather than when parsing
  auto corruptingNoise =
      noiseModel::Isotropic::Sigma(d_, parameters.noiseSigma);
  Q_ = buildQ();
  D_ = buildD();
  L_ = D_ - Q_;
}

/* ************************************************************************* */
// Copy poses from Values
static std::map<Key, Pose3> PoseMapFromValues(const Values& values) {
  Values::ConstFiltered<Pose3> pose_filtered = values.filter<Pose3>();
  std::map<Key, Pose3> poses;
  for (const auto& it : pose_filtered) {
    poses[it.key] = it.value;
  }
  return poses;
}

/* ************************************************************************* */
ShonanAveraging::ShonanAveraging(const BetweenFactorPose3s& factors,
                                 const Values& values,
                                 const ShonanAveragingParameters& parameters)
    : ShonanAveraging(factors, PoseMapFromValues(values), parameters) {}

/* ************************************************************************* */
ShonanAveraging::ShonanAveraging(const string& g2oFile,
                                 const ShonanAveragingParameters& parameters)
    : ShonanAveraging(parse3DFactors(g2oFile), parse3DPoses(g2oFile),
                      parameters) {}

/* ************************************************************************* */
NonlinearFactorGraph ShonanAveraging::buildGraphAt(size_t p) const {
  NonlinearFactorGraph graph;
  auto G = boost::make_shared<Matrix>(SOn::VectorizedGenerators(p));
  for (const auto &factor : factors_) {
    const auto &keys = factor->keys();
    const auto &Tij = factor->measured();
    const auto &model = factor->noiseModel();
    graph.emplace_shared<FrobeniusWormholeFactor>(keys[0], keys[1],
                                                  Tij.rotation(), p, model, G);
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
double ShonanAveraging::costAt(size_t p, const Values& values) const {
  const NonlinearFactorGraph graph = buildGraphAt(p);
  return graph.error(values);
}

/* ************************************************************************* */
boost::shared_ptr<LevenbergMarquardtOptimizer>
ShonanAveraging::createOptimizerAt(
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
      const size_t dim = SOn::Dimension(p);
      graph.emplace_shared<KarcherMeanFactor<SOn>>(graph.keys(), dim);
    } else {
      graph.emplace_shared<PriorFactor<SOn>>(
          parameters_.anchor.first,
          SOn::Lift(p, parameters_.anchor.second.matrix()));
    }
  }

  // Add gauge factors for p>d_+1
  if (parameters_.fixGauge && p>d_+1) {
    for (auto key: graph.keys())
      graph.emplace_shared<ShonanGaugeFactor>(key, p);
  }

  // Optimize
  auto optimizer = boost::make_shared<LevenbergMarquardtOptimizer>(
      graph, initial, parameters_.lm);
  return optimizer;
}

/* ************************************************************************* */
Values ShonanAveraging::tryOptimizingAt(
    size_t p, const boost::optional<const Values&> initialEstimate) const {
  boost::shared_ptr<LevenbergMarquardtOptimizer> lm =
      createOptimizerAt(p, initialEstimate);
  Values result = lm->optimize();
  return result;
}

/* ************************************************************************* */
// Project to pxdN Stiefel manifold
static Matrix StiefelElementMatrix(const Values &values, size_t d = 3) {
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
Values ShonanAveraging::projectFrom(size_t p, const Values& values) const {
  Values Rot3_values;
  for (size_t j = 0; j < nrPoses(); j++) {
    const SOn Q = values.at<SOn>(j);
    assert(Q.rows() == p);
    const Rot3 R = Rot3::ClosestTo(Q.matrix().topLeftCorner(d_, d_));
    Rot3_values.insert(j, R);
  }
  return Rot3_values;
}

/* ************************************************************************* */
Values ShonanAveraging::roundSolution(const Matrix S) const {
  const size_t N = nrPoses();
  // First, compute a thin SVD of S
  Eigen::JacobiSVD<Matrix> svd(S, Eigen::ComputeThinV);
  Vector sigmas = svd.singularValues();

  // Construct a diagonal matrix comprised of the first d singular values
  using DiagonalMatrix = Eigen::DiagonalMatrix<double, Eigen::Dynamic>;
  DiagonalMatrix Sigma_d(d_);
  auto& diagonal = Sigma_d.diagonal();
  for (size_t i = 0; i < d_; ++i) diagonal(i) = sigmas(i);

  // First, construct a rank-d truncated singular value decomposition for S
  Matrix R = Sigma_d * svd.matrixV().leftCols(d_).transpose();
  Vector determinants(N);

  size_t ng0 = 0;  // This will count the number of blocks whose

  // determinants have positive sign
  for (size_t i = 0; i < N; ++i) {
    // Compute the determinant of the ith dxd block of R
    determinants(i) = R.block(0, i * d_, d_, d_).determinant();
    if (determinants(i) > 0) ++ng0;
  }

  if (ng0 < N / 2) {
    // Less than half of the total number of blocks have the correct sign,
    // so reverse their orientations Get a reflection matrix that we can use
    // to reverse the signs of those blocks of R that have the wrong
    // determinant
    Matrix reflector = Matrix::Identity(d_, d_);
    reflector(d_ - 1, d_ - 1) = -1;
    R = reflector * R;
  }

  // Finally, project each dxd rotation block to SO(d)
  Values Rot3_values;
  for (size_t i = 0; i < N; ++i) {
    const Rot3 Ri = Rot3::ClosestTo(R.block(0, i * d_, d_, d_));
    Rot3_values.insert(i, Ri);
  }
  return Rot3_values;
}

/* ************************************************************************* */
Values ShonanAveraging::roundSolution(const Values& values) const {
  // Project to pxdN Stiefel manifold...
  Matrix S = StiefelElementMatrix(values);
  // ...and call version above.
  return roundSolution(S);
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
  // Finally, project each dxd rotation block to SO(d)
  Values SO3_values;
  const size_t N = nrPoses();
  for (size_t i = 0; i < N; ++i) {
    SO3_values.insert(i, SO3(values.at<Rot3>(i).matrix()));
  }
  return graph.error(SO3_values);
}

/* ************************************************************************* */
// Get kappa from noise model
static double Kappa(const BetweenFactor<Pose3>::shared_ptr& factor) {
  const auto& model = factor->noiseModel();
  const auto& isotropic = ConvertPose3NoiseModel(model, 3);
  const double sigma = isotropic->sigma();
  return 1.0 / (sigma * sigma);
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::buildD() const {
  // Each measurement contributes 2*d elements along the diagonal of the
  // degree matrix.
  const size_t stride = 2 * d_;

  // Reserve space for triplets
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * factors_.size());

  for (const auto& factor : factors_) {
    // Get pose keys
    const auto& keys = factor->keys();
    size_t i = keys[0];
    size_t j = keys[1];

    // Get kappa from noise model
    double kappa = Kappa(factor);

    for (size_t k = 0; k < d_; k++) {
      // Elements of ith block-diagonal
      triplets.emplace_back(d_ * i + k, d_ * i + k, kappa);
      // Elements of jth block-diagonal
      triplets.emplace_back(d_ * j + k, d_ * j + k, kappa);
    }
  }

  // Construct and return a sparse matrix from these triplets
  const size_t N = nrPoses();
  Sparse D(d_ * N, d_ * N);
  D.setFromTriplets(triplets.begin(), triplets.end());

  return D;
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::buildQ() const {
  // Each measurement contributes 2*d^2 elements on a pair of symmetric
  // off-diagonal blocks
  const size_t stride = 2 * d_ * d_;

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
    double kappa = Kappa(factor);

    for (size_t r = 0; r < d_; r++) {
      for (size_t c = 0; c < d_; c++) {
        // Elements of ij block
        triplets.emplace_back(i * d_ + r, j * d_ + c, kappa * Rij(r, c));
        // Elements of ji block
        triplets.emplace_back(j * d_ + r, i * d_ + c, kappa * Rij(c, r));
      }
    }
  }

  // Construct and return a sparse matrix from these triplets
  const size_t N = nrPoses();
  Sparse Q(d_ * N, d_ * N);
  Q.setFromTriplets(triplets.begin(), triplets.end());

  return Q;
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::computeLambda(const Matrix& S) const {
  // Each pose contributes 2*d_ elements along the diagonal of Lambda
  const size_t stride = d_ * d_;

  // Reserve space for triplets
  const size_t N = nrPoses();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * N);

  // Do sparse-dense multiply to get Q*S'
  auto QSt = Q_ * S.transpose();

  for (size_t j = 0; j < N; j++) {
    // Compute B, the building block for the j^th diagonal block of Lambda
    Matrix B = QSt.middleRows(d_ * j, d_) * S.middleCols(d_ * j, d_);

    // Elements of jth block-diagonal
    for (size_t r = 0; r < d_; r++)
      for (size_t c = 0; c < d_; c++)
        triplets.emplace_back(j * d_ + r, j * d_ + c,
                              0.5 * (B(r, c) + B(c, r)));
  }

  // Construct and return a sparse matrix from these triplets
  Sparse Lambda(d_ * N, d_ * N);
  Lambda.setFromTriplets(triplets.begin(), triplets.end());
  return Lambda;
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::computeLambda(
    const Values& values) const {
  // Project to pxdN Stiefel manifold...
  Matrix S = StiefelElementMatrix(values);
  // ...and call version above.
  return computeLambda(S);
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::computeA(const Values& values) const {
  assert(values.size() == nrPoses());
  const Matrix S = StiefelElementMatrix(values);
  auto Lambda = computeLambda(S);
  return Lambda - Q_;
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
  explicit MatrixProdFunctor(const SparseMatrix& A, double sigma = 0)
      : A_(A), sigma_(sigma) {}

  int rows() const { return A_.rows(); }
  int cols() const { return A_.cols(); }

  // Matrix-vector multiplication operation
  void perform_op(const double* x, double* y) const {
    // Wrap the raw arrays as Eigen Vector types
    Eigen::Map<const Vector> X(x, rows());
    Eigen::Map<Vector> Y(y, rows());

    // Do the multiplication using wrapped Eigen vectors
    Y = A_ * X + sigma_ * X;
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
/// values of minEigenValue and minEigenVector appropriately

/// Note that in the following function signature, S is supposed to be the
/// block-row-matrix that is a critical point for the optimization algorithm;
/// either S (Stiefel manifold) or R (block rotations).  We use this to
/// construct a starting vector v for the Lanczos process that will be close to
/// the minimum eigenvector we're looking for whenever the relaxation is exact
/// -- this is a key feature that helps to make this method fast.  Note that
/// instead of passing in all of S, it would be enough to pass in one of S's
/// *rows*, if that's more convenient.

// For the defaults, David Rosen says:
//   - maxIterations refers to the max number of Lanczos iterations to run;
//   ~1000 should be sufficiently large
//   - We've been using 10^-4 for the nonnegativity tolerance
//   - for numLanczosVectors, 20 is a good default value

static bool SparseMinimumEigenValue(
    const SparseMatrix& A, const Matrix& S, double* minEigenValue,
    Vector* minEigenVector = 0, size_t* numIterations = 0,
    size_t maxIterations = 1000,
    double minEigenvalueNonnegativityTolerance = 10e-4,
    Eigen::Index numLanczosVectors = 20) {
  // a. Estimate the largest-magnitude eigenvalue of this matrix using Lanczos
  MatrixProdFunctor lmOperator(A);
  Spectra::SymEigsSolver<double, Spectra::SELECT_EIGENVALUE::LARGEST_MAGN,
                         MatrixProdFunctor>
      lmEigenValueSolver(&lmOperator, 1, std::min(numLanczosVectors, A.rows()));
  lmEigenValueSolver.init();

  const int lmConverged = lmEigenValueSolver.compute(
      maxIterations, 1e-4, Spectra::SELECT_EIGENVALUE::LARGEST_MAGN);

  // Check convergence and bail out if necessary
  if (lmConverged != 1) return false;

  const double lmEigenValue = lmEigenValueSolver.eigenvalues()(0);

  if (lmEigenValue < 0) {
    // The largest-magnitude eigenvalue is negative, and therefore also the
    // minimum eigenvalue, so just return this solution
    *minEigenValue = lmEigenValue;
    if (minEigenVector) {
      *minEigenVector = lmEigenValueSolver.eigenvectors(1);
      minEigenVector->normalize();  // Ensure that this is a unit vector
    }
    return true;
  }

  // The largest-magnitude eigenvalue is positive, and is therefore the
  // maximum  eigenvalue.  Therefore, after shifting the spectrum of A
  // by -2*lmEigenValue (by forming A - 2*lambda_max*I), the  shifted
  // spectrum will lie in the interval [minEigenValue(A) - 2* lambda_max(A),
  // -lambda_max*A]; in particular, the largest-magnitude eigenvalue of
  //  A - 2*lambda_max*I is minEigenValue - 2*lambda_max, with corresponding
  // eigenvector v_min

  MatrixProdFunctor minShiftedOperator(A, -2 * lmEigenValue);

  Spectra::SymEigsSolver<double, Spectra::SELECT_EIGENVALUE::LARGEST_MAGN,
                         MatrixProdFunctor>
      minEigenValueSolver(&minShiftedOperator, 1,
                          std::min(numLanczosVectors, A.rows()));

  // If S is a critical point of F, then S^T is also in the null space of S -
  // Lambda(S) (cf. Lemma 6 of the tech report), and therefore its rows are
  // eigenvectors corresponding to the eigenvalue 0.  In the case  that the
  // relaxation is exact, this is the *minimum* eigenvalue, and therefore the
  // rows of S are exactly the eigenvectors that we're looking for.  On the
  // other hand, if the relaxation is *not* exact, then S - Lambda(S) has at
  // least one strictly negative eigenvalue, and the rows of S are *unstable
  // fixed points* for the Lanczos iterations.  Thus, we will take a slightly
  // "fuzzed" version of the first row of S as an initialization for the
  // Lanczos iterations; this allows for rapid convergence in the case that
  // the relaxation is exact (since are starting close to a solution), while
  // simultaneously allowing the iterations to escape from this fixed point in
  // the case that the relaxation is not exact.
  Vector v0 = S.row(0).transpose();
  Vector perturbation(v0.size());
  perturbation.setRandom();
  perturbation.normalize();
  Vector xinit = v0 + (.03 * v0.norm()) * perturbation;  // Perturb v0 by ~3%

  // Use this to initialize the eigensolver
  minEigenValueSolver.init(xinit.data());

  // Now determine the relative precision required in the Lanczos method in
  // order to be able to estimate the smallest eigenvalue within an *absolute*
  // tolerance of 'minEigenvalueNonnegativityTolerance'
  const int minConverged = minEigenValueSolver.compute(
      maxIterations, minEigenvalueNonnegativityTolerance / lmEigenValue,
      Spectra::SELECT_EIGENVALUE::LARGEST_MAGN);

  if (minConverged != 1) return false;

  *minEigenValue = minEigenValueSolver.eigenvalues()(0) + 2 * lmEigenValue;
  if (minEigenVector) {
    *minEigenVector = minEigenValueSolver.eigenvectors(1);
    minEigenVector->normalize();  // Ensure that this is a unit vector
  }
  if (numIterations) *numIterations = minEigenValueSolver.num_iterations();
  return true;
}

/* ************************************************************************* */
ShonanAveraging::Sparse ShonanAveraging::computeA(const Matrix& S) const {
  auto Lambda = computeLambda(S);
  return Lambda - Q_;
}

/* ************************************************************************* */
double ShonanAveraging::computeMinEigenValue(const Values& values,
                                             Vector* minEigenVector) const {
  assert(values.size() == nrPoses());
  const Matrix S = StiefelElementMatrix(values);
  auto A = computeA(S);
#ifdef SLOW_EIGEN_COMPUTATION
  Eigen::EigenSolver<Matrix> solver(Matrix(A), false);
  auto lambdas = solver.eigenvalues();
  double minEigenValue = lambdas(0).real();
  for (size_t i = 1; i < lambdas.size(); i++) {
    minEigenValue = min(lambdas(i).real(), minEigenValue);
  }
#else
  double minEigenValue;
  bool success = SparseMinimumEigenValue(A, S, &minEigenValue, minEigenVector);
  if (!success) {
    throw std::runtime_error(
        "SparseMinimumEigenValue failed to compute minimum eigenvalue.");
  }
#endif
  return minEigenValue;
}

/* ************************************************************************* */
std::pair<double, Vector> ShonanAveraging::computeMinEigenVector(
    const Values& values) const {
  Vector minEigenVector;
  double minEigenValue = computeMinEigenValue(values, &minEigenVector);
  return std::make_pair(minEigenValue, minEigenVector);
}

/* ************************************************************************* */
bool ShonanAveraging::checkOptimality(const Values& values) const {
  double minEigenValue = computeMinEigenValue(values);
  return minEigenValue > parameters_.optimalityThreshold;
}

/* ************************************************************************* */
Vector ShonanAveraging::MakeATangentVector(size_t p, const Vector &v, size_t i,
                                           size_t d) {
  // Create a tangent direction xi with eigenvector segment v_i
  const size_t dimension = p * (p - 1) / 2;
  const auto v_i = v.segment(i * d, d);
  Vector xi = Vector::Zero(dimension);
  double sign = pow(-1.0, round((p + 1) / 2) + 1);
  for (size_t j = 0; j < d; j++) {
    xi(j + p - d - 1) = sign * v_i(d - j - 1);
    sign = -sign;
  }
  return xi;
}

/* ************************************************************************* */
Matrix ShonanAveraging::riemannianGradient(size_t p,
                                           const Values& values) const {
  Matrix S_dot = StiefelElementMatrix(values);
  // calculate the gradient of F(Q_dot) at Q_dot
  Matrix euclideanGradient = 2 * (L_ * (S_dot.transpose())).transpose();
  // cout << "euclidean gradient rows and cols" << euclideanGradient.rows() <<
  // "\t" << euclideanGradient.cols() << endl;

  // project the gradient onto the entire euclidean space
  Matrix symBlockDiagProduct(p, d_ * nrPoses());
  for (size_t i = 0; i < nrPoses(); i++) {
    // Compute block product
    Matrix P = S_dot.block(0, i * d_, p, d_).transpose() *
               euclideanGradient.block(0, i * d_, p, d_);
    // Symmetrize this block
    Matrix S = .5 * (P + P.transpose());
    // Compute S_dot * S and set corresponding block
    symBlockDiagProduct.block(0, i * d_, p, d_) = S_dot.block(0, i * d_, p, d_) * S;
  }
  Matrix riemannianGradient = euclideanGradient - symBlockDiagProduct;
  return riemannianGradient;
}

/* ************************************************************************* */
Values ShonanAveraging::dimensionLifting(size_t p, const Values& values,
                                         const Vector& minEigenVector) const {
  Values newValues;
  // for all poses, initialize with the eigenvector segment v_i
  for (size_t i = 0; i < nrPoses(); i++) {
    // Initialize SO(p) with topleft block the old value Q \in SO(p-1)
    auto Q = SOn::Lift(p, values.at<SOn>(i).matrix());
    // Create a tangent direction xi with eigenvector segment v_i
    const Vector xi = MakeATangentVector(p, minEigenVector, i);
    // Move the old value in the descent direction
    const SOn Qplus = Q.retract(xi);
    newValues.insert(i, Qplus);
  }
  return newValues;
}

/* ************************************************************************* */
Values ShonanAveraging::initializeWithDescent(
    size_t p, const Values& values, const Vector& minEigenVector,
    double minEigenValue, double gradienTolerance,
    double preconditionedGradNormTolerance) const {
  double funcVal = costAt(p - 1, values);
  double alphaMin = 1e-2;
  double alpha =
      std::max(1024 * alphaMin, 10 * gradienTolerance / fabs(minEigenValue));
  vector<double> alphas;
  vector<double> fvals;
  // line search
  while ((alpha >= alphaMin)) {
    Values Qplus = dimensionLifting(p, values, alpha * minEigenVector);
    double funcValTest = costAt(p, Qplus);
    Matrix gradTest = riemannianGradient(p, Qplus);
    double gradTestNorm = gradTest.norm();
    // Record alpha and funcVal
    alphas.push_back(alpha);
    fvals.push_back(funcValTest);
    if ((funcVal > funcValTest) && (gradTestNorm > gradienTolerance)) {
      return Qplus;
    }
    alpha /= 2;
  }

  auto fminIter = min_element(fvals.begin(), fvals.end());
  auto minIdx = distance(fvals.begin(), fminIter);
  double fMin = fvals[minIdx];
  double aMin = alphas[minIdx];
  if (fMin < funcVal) {
    Values Qplus = dimensionLifting(p, values, aMin * minEigenVector);
    return Qplus;
  }

  return dimensionLifting(p, values, alpha * minEigenVector);
}

/* ************************************************************************* */
std::pair<Values, double> ShonanAveraging::run(size_t pMin, size_t pMax,
                                               bool withDescent) const {
  Values Qstar;
  Vector minEigenVector;
  double minEigenValue = 0;
  for (size_t p = pMin; p <= pMax; p++) {
    const Values initial =
        (p > pMin && withDescent)
            ? initializeWithDescent(p, Qstar, minEigenVector, minEigenValue)
            : initializeRandomlyAt(p);
    Qstar = tryOptimizingAt(p, initial);
    minEigenValue = computeMinEigenValue(Qstar, &minEigenVector);
    if (minEigenValue > parameters_.optimalityThreshold) {
      const Values SO3Values = roundSolution(Qstar);
      return std::make_pair(SO3Values, minEigenValue);
    }
  }
  throw std::runtime_error("Shonan::run did not converge for given pMax");
}

/* ************************************************************************* */
std::pair<Values, double> ShonanAveraging::runWithRandom(size_t pMin,
                                                         size_t pMax) const {
  return run(pMin, pMax, false);
}

/* ************************************************************************* */
std::pair<Values, double> ShonanAveraging::runWithDescent(size_t pMin,
                                                          size_t pMax) const {
  return run(pMin, pMax, true);
}

/* ************************************************************************* */
}  // namespace gtsam
