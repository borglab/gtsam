/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   ShonanAveraging.cpp
 * @date   March 2019 - August 2020
 * @author Frank Dellaert, David Rosen, and Jing Wu
 * @brief  Shonan Averaging algorithm
 */

#include <SymEigsSolver.h>
#include <cmath>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include <gtsam/sfm/ShonanFactor.h>
#include <gtsam/sfm/ShonanGaugeFactor.h>
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <complex>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <vector>

namespace gtsam {

// In Wrappers we have no access to this so have a default ready
static std::mt19937 kRandomNumberGenerator(42);

using Sparse = Eigen::SparseMatrix<double>;

/* ************************************************************************* */
template <size_t d>
ShonanAveragingParameters<d>::ShonanAveragingParameters(
    const LevenbergMarquardtParams &_lm, const std::string &method,
    double optimalityThreshold, double alpha, double beta, double gamma)
    : lm(_lm),
      optimalityThreshold(optimalityThreshold),
      alpha(alpha),
      beta(beta),
      gamma(gamma),
      useHuber(false),
      certifyOptimality(true) {
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
// Explicit instantiation for d=2 and d=3
template struct ShonanAveragingParameters<2>;
template struct ShonanAveragingParameters<3>;

/* ************************************************************************* */
// Calculate number of unknown rotations referenced by measurements
// Throws exception of keys are not numbered as expected (may change in future).
template <size_t d>
static size_t NrUnknowns(
    const typename ShonanAveraging<d>::Measurements &measurements) {
  Key maxKey = 0;
  std::set<Key> keys;
  for (const auto &measurement : measurements) {
    for (const Key &key : measurement.keys()) {
      maxKey = std::max(key, maxKey);
      keys.insert(key);
    }
  }
  size_t N = keys.size();
  if (maxKey != N - 1) {
    throw std::invalid_argument(
        "As of now, ShonanAveraging expects keys numbered 0..N-1");
  }
  return N;
}

/* ************************************************************************* */
template <size_t d>
ShonanAveraging<d>::ShonanAveraging(const Measurements &measurements,
                                    const Parameters &parameters)
    : parameters_(parameters),
      measurements_(measurements),
      nrUnknowns_(NrUnknowns<d>(measurements)) {
  for (const auto &measurement : measurements_) {
    const auto &model = measurement.noiseModel();
    if (model && model->dim() != SO<d>::dimension) {
      measurement.print("Factor with incorrect noise model:\n");
      throw std::invalid_argument(
          "ShonanAveraging: measurements passed to "
          "constructor have incorrect dimension.");
    }
  }
  Q_ = buildQ();
  D_ = buildD();
  L_ = D_ - Q_;
}

/* ************************************************************************* */
template <size_t d>
NonlinearFactorGraph ShonanAveraging<d>::buildGraphAt(size_t p) const {
  NonlinearFactorGraph graph;
  auto G = boost::make_shared<Matrix>(SO<-1>::VectorizedGenerators(p));

  for (const auto &measurement : measurements_) {
    const auto &keys = measurement.keys();
    const auto &Rij = measurement.measured();
    const auto &model = measurement.noiseModel();
    graph.emplace_shared<ShonanFactor<d>>(keys[0], keys[1], Rij, p, model, G);
  }
  // Possibly add Karcher prior
  if (parameters_.beta > 0) {
    const size_t dim = SOn::Dimension(p);
    graph.emplace_shared<KarcherMeanFactor<SOn>>(graph.keys(), dim);
  }
  // Possibly add gauge factors - they are probably useless as gradient is zero
  if (parameters_.gamma > 0 && p > d + 1) {
    for (auto key : graph.keys())
      graph.emplace_shared<ShonanGaugeFactor>(key, p, d, parameters_.gamma);
  }
  return graph;
}

/* ************************************************************************* */
template <size_t d>
double ShonanAveraging<d>::costAt(size_t p, const Values &values) const {
  const NonlinearFactorGraph graph = buildGraphAt(p);
  return graph.error(values);
}

/* ************************************************************************* */
template <size_t d>
boost::shared_ptr<LevenbergMarquardtOptimizer>
ShonanAveraging<d>::createOptimizerAt(size_t p, const Values &initial) const {
  // Build graph
  NonlinearFactorGraph graph = buildGraphAt(p);

  // Anchor prior is added here as depends on initial value (and cost is zero)
  if (parameters_.alpha > 0) {
    size_t i;
    Rot value;
    const size_t dim = SOn::Dimension(p);
    std::tie(i, value) = parameters_.anchor;
    auto model = noiseModel::Isotropic::Precision(dim, parameters_.alpha);
    graph.emplace_shared<PriorFactor<SOn>>(i, SOn::Lift(p, value.matrix()),
                                           model);
  }
  // Optimize
  return boost::make_shared<LevenbergMarquardtOptimizer>(graph, initial,
                                                         parameters_.lm);
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::tryOptimizingAt(size_t p,
                                           const Values &initial) const {
  auto lm = createOptimizerAt(p, initial);
  return lm->optimize();
}

/* ************************************************************************* */
// Project to pxdN Stiefel manifold
template <size_t d>
Matrix ShonanAveraging<d>::StiefelElementMatrix(const Values &values) {
  const size_t N = values.size();
  const size_t p = values.at<SOn>(0).rows();
  Matrix S(p, N * d);
  for (const auto& it : values.extract<SOn>()) {
    S.middleCols<d>(it.first * d) =
        it.second.matrix().leftCols<d>();  // project Qj to Stiefel
  }
  return S;
}

/* ************************************************************************* */
template <>
Values ShonanAveraging<2>::projectFrom(size_t p, const Values &values) const {
  Values result;
  for (const auto& it : values.extract<SOn>()) {
    assert(it.second.rows() == p);
    const auto &M = it.second.matrix();
    const Rot2 R = Rot2::atan2(M(1, 0), M(0, 0));
    result.insert(it.first, R);
  }
  return result;
}

template <>
Values ShonanAveraging<3>::projectFrom(size_t p, const Values &values) const {
  Values result;
  for (const auto& it : values.extract<SOn>()) {
    assert(it.second.rows() == p);
    const auto &M = it.second.matrix();
    const Rot3 R = Rot3::ClosestTo(M.topLeftCorner<3, 3>());
    result.insert(it.first, R);
  }
  return result;
}

/* ************************************************************************* */
template <size_t d>
static Matrix RoundSolutionS(const Matrix &S) {
  const size_t N = S.cols() / d;
  // First, compute a thin SVD of S
  Eigen::JacobiSVD<Matrix> svd(S, Eigen::ComputeThinV);
  const Vector sigmas = svd.singularValues();

  // Construct a diagonal matrix comprised of the first d singular values
  using DiagonalMatrix = Eigen::DiagonalMatrix<double, d>;
  DiagonalMatrix Sigma_d;
  Sigma_d.diagonal() = sigmas.head<d>();

  // Now, construct a rank-d truncated singular value decomposition for S
  Matrix R = Sigma_d * svd.matrixV().leftCols<d>().transpose();

  // Count the number of blocks whose determinants have positive sign
  size_t numPositiveBlocks = 0;
  for (size_t i = 0; i < N; ++i) {
    // Compute the determinant of the ith dxd block of R
    double determinant = R.middleCols<d>(d * i).determinant();
    if (determinant > 0) ++numPositiveBlocks;
  }

  if (numPositiveBlocks < N / 2) {
    // Less than half of the total number of blocks have the correct sign.
    // To reverse their orientations, multiply with a reflection matrix.
    DiagonalMatrix reflector;
    reflector.setIdentity();
    reflector.diagonal()(d - 1) = -1;
    R = reflector * R;
  }

  return R;
}

/* ************************************************************************* */
template <>
Values ShonanAveraging<2>::roundSolutionS(const Matrix &S) const {
  // Round to a 2*2N matrix
  Matrix R = RoundSolutionS<2>(S);

  // Finally, project each dxd rotation block to SO(2)
  Values values;
  for (size_t j = 0; j < nrUnknowns(); ++j) {
    const Rot2 Ri = Rot2::atan2(R(1, 2 * j), R(0, 2 * j));
    values.insert(j, Ri);
  }
  return values;
}

template <>
Values ShonanAveraging<3>::roundSolutionS(const Matrix &S) const {
  // Round to a 3*3N matrix
  Matrix R = RoundSolutionS<3>(S);

  // Finally, project each dxd rotation block to SO(3)
  Values values;
  for (size_t j = 0; j < nrUnknowns(); ++j) {
    const Rot3 Ri = Rot3::ClosestTo(R.middleCols<3>(3 * j));
    values.insert(j, Ri);
  }
  return values;
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::roundSolution(const Values &values) const {
  // Project to pxdN Stiefel manifold...
  Matrix S = StiefelElementMatrix(values);
  // ...and call version above.
  return roundSolutionS(S);
}

/* ************************************************************************* */
template <size_t d>
double ShonanAveraging<d>::cost(const Values &values) const {
  NonlinearFactorGraph graph;
  for (const auto &measurement : measurements_) {
    const auto &keys = measurement.keys();
    const auto &Rij = measurement.measured();
    const auto &model = measurement.noiseModel();
    graph.emplace_shared<FrobeniusBetweenFactor<SO<d>>>(
        keys[0], keys[1], SO<d>(Rij.matrix()), model);
  }
  // Finally, project each dxd rotation block to SO(d)
  Values result;
  for (const auto& it : values.extract<Rot>()) {
    result.insert(it.first, SO<d>(it.second.matrix()));
  }
  return graph.error(result);
}

/* ************************************************************************* */
// Get kappa from noise model
template <typename T, size_t d>
static double Kappa(const BinaryMeasurement<T> &measurement,
                    const ShonanAveragingParameters<d> &parameters) {
  const auto &isotropic = boost::dynamic_pointer_cast<noiseModel::Isotropic>(
      measurement.noiseModel());
  double sigma;
  if (isotropic) {
    sigma = isotropic->sigma();
  } else {
    const auto &robust = boost::dynamic_pointer_cast<noiseModel::Robust>(
        measurement.noiseModel());
    // Check if noise model is robust
    if (robust) {
      // If robust, check if optimality certificate is expected
      if (parameters.getCertifyOptimality()) {
        throw std::invalid_argument(
            "Certification of optimality does not work with robust cost.");
      } else {
        // Optimality certificate not required, so setting default sigma
        sigma = 1;
      }
    } else {
      throw std::invalid_argument(
          "Shonan averaging noise models must be isotropic (but robust losses "
          "are allowed).");
    }
  }
  return 1.0 / (sigma * sigma);
}

/* ************************************************************************* */
template <size_t d>
Sparse ShonanAveraging<d>::buildD() const {
  // Each measurement contributes 2*d elements along the diagonal of the
  // degree matrix.
  static constexpr size_t stride = 2 * d;

  // Reserve space for triplets
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * measurements_.size());

  for (const auto &measurement : measurements_) {
    // Get pose keys
    const auto &keys = measurement.keys();

    // Get kappa from noise model
    double kappa = Kappa<Rot, d>(measurement, parameters_);

    const size_t di = d * keys[0], dj = d * keys[1];
    for (size_t k = 0; k < d; k++) {
      // Elements of ith block-diagonal
      triplets.emplace_back(di + k, di + k, kappa);
      // Elements of jth block-diagonal
      triplets.emplace_back(dj + k, dj + k, kappa);
    }
  }

  // Construct and return a sparse matrix from these triplets
  const size_t dN = d * nrUnknowns();
  Sparse D(dN, dN);
  D.setFromTriplets(triplets.begin(), triplets.end());

  return D;
}

/* ************************************************************************* */
template <size_t d>
Sparse ShonanAveraging<d>::buildQ() const {
  // Each measurement contributes 2*d^2 elements on a pair of symmetric
  // off-diagonal blocks
  static constexpr size_t stride = 2 * d * d;

  // Reserve space for triplets
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * measurements_.size());

  for (const auto &measurement : measurements_) {
    // Get pose keys
    const auto &keys = measurement.keys();

    // Extract rotation measurement
    const auto Rij = measurement.measured().matrix();

    // Get kappa from noise model
    double kappa = Kappa<Rot, d>(measurement, parameters_);

    const size_t di = d * keys[0], dj = d * keys[1];
    for (size_t r = 0; r < d; r++) {
      for (size_t c = 0; c < d; c++) {
        // Elements of ij block
        triplets.emplace_back(di + r, dj + c, kappa * Rij(r, c));
        // Elements of ji block
        triplets.emplace_back(dj + r, di + c, kappa * Rij(c, r));
      }
    }
  }

  // Construct and return a sparse matrix from these triplets
  const size_t dN = d * nrUnknowns();
  Sparse Q(dN, dN);
  Q.setFromTriplets(triplets.begin(), triplets.end());

  return Q;
}

/* ************************************************************************* */
template <size_t d>
Sparse ShonanAveraging<d>::computeLambda(const Matrix &S) const {
  // Each pose contributes 2*d elements along the diagonal of Lambda
  static constexpr size_t stride = d * d;

  // Reserve space for triplets
  const size_t N = nrUnknowns();
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * N);

  // Do sparse-dense multiply to get Q*S'
  auto QSt = Q_ * S.transpose();

  for (size_t j = 0; j < N; j++) {
    // Compute B, the building block for the j^th diagonal block of Lambda
    const size_t dj = d * j;
    Matrix B = QSt.middleRows(dj, d) * S.middleCols<d>(dj);

    // Elements of jth block-diagonal
    for (size_t r = 0; r < d; r++)
      for (size_t c = 0; c < d; c++)
        triplets.emplace_back(dj + r, dj + c, 0.5 * (B(r, c) + B(c, r)));
  }

  // Construct and return a sparse matrix from these triplets
  Sparse Lambda(d * N, d * N);
  Lambda.setFromTriplets(triplets.begin(), triplets.end());
  return Lambda;
}

/* ************************************************************************* */
template <size_t d>
Sparse ShonanAveraging<d>::computeLambda(const Values &values) const {
  // Project to pxdN Stiefel manifold...
  Matrix S = StiefelElementMatrix(values);
  // ...and call version above.
  return computeLambda(S);
}

/* ************************************************************************* */
template <size_t d>
Sparse ShonanAveraging<d>::computeA(const Values &values) const {
  assert(values.size() == nrUnknowns());
  const Matrix S = StiefelElementMatrix(values);
  auto Lambda = computeLambda(S);
  return Lambda - Q_;
}

/* ************************************************************************* */
template <size_t d>
Sparse ShonanAveraging<d>::computeA(const Matrix &S) const {
  auto Lambda = computeLambda(S);
  return Lambda - Q_;
}

/* ************************************************************************* */
// Perturb the initial initialVector by adding a spherically-uniformly
// distributed random vector with 0.03*||initialVector||_2 magnitude to
// initialVector
// ref : Part III. C, Rosen, D. and Carlone, L., 2017, September. Computational
// enhancements for certifiably correct SLAM. In Proceedings of the
// International Conference on Intelligent Robots and Systems.
static Vector perturb(const Vector &initialVector) {
  // generate a 0.03*||x_0||_2 as stated in David's paper
  int n = initialVector.rows();
  Vector disturb = Vector::Random(n);
  disturb.normalize();

  double magnitude = initialVector.norm();
  Vector perturbedVector = initialVector + 0.03 * magnitude * disturb;
  perturbedVector.normalize();
  return perturbedVector;
}

/* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS
// Alg.6 from paper Distributed Certifiably Correct Pose-Graph Optimization,
// it takes in the certificate matrix A as input, the maxIterations and the
// minEigenvalueNonnegativityTolerance is set to 1000 and 10e-4 ad default,
// there are two parts
// in this algorithm:
// (1) compute the maximum eigenpair (\lamda_dom, \vect{v}_dom) of A by power
// method. if \lamda_dom is less than zero, then return the eigenpair. (2)
// compute the maximum eigenpair (\theta, \vect{v}) of C = \lamda_dom * I - A by
// accelerated power method. Then return (\lamda_dom - \theta, \vect{v}).
static bool PowerMinimumEigenValue(
    const Sparse &A, const Matrix &S, double &minEigenValue,
    Vector *minEigenVector = 0, size_t *numIterations = 0,
    size_t maxIterations = 1000,
    double minEigenvalueNonnegativityTolerance = 10e-4) {

  // a. Compute dominant eigenpair of S using power method
  PowerMethod<Sparse> pmOperator(A);

  const bool pmConverged = pmOperator.compute(
      maxIterations, 1e-5);

  // Check convergence and bail out if necessary
  if (!pmConverged) return false;

  const double pmEigenValue = pmOperator.eigenvalue();

  if (pmEigenValue < 0) {
    // The largest-magnitude eigenvalue is negative, and therefore also the
    // minimum eigenvalue, so just return this solution
    minEigenValue = pmEigenValue;
    if (minEigenVector) {
      *minEigenVector = pmOperator.eigenvector();
      minEigenVector->normalize();  // Ensure that this is a unit vector
    }
    return true;
  }

  const Sparse C = pmEigenValue * Matrix::Identity(A.rows(), A.cols()).sparseView() - A;
  const boost::optional<Vector> initial = perturb(S.row(0));
  AcceleratedPowerMethod<Sparse> apmShiftedOperator(C, initial);

  const bool minConverged = apmShiftedOperator.compute(
      maxIterations, minEigenvalueNonnegativityTolerance / pmEigenValue);

  if (!minConverged) return false;

  minEigenValue = pmEigenValue - apmShiftedOperator.eigenvalue();
  if (minEigenVector) {
    *minEigenVector = apmShiftedOperator.eigenvector();
    minEigenVector->normalize();  // Ensure that this is a unit vector
  }
  if (numIterations) *numIterations = apmShiftedOperator.nrIterations();
  return true;
}

/** This is a lightweight struct used in conjunction with Spectra to compute
 * the minimum eigenvalue and eigenvector of a sparse matrix A; it has a single
 * nontrivial function, perform_op(x,y), that computes and returns the product
 * y = (A + sigma*I) x */
struct MatrixProdFunctor {
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Sparse &A_;

  // Spectral shift
  double sigma_;

  // Constructor
  explicit MatrixProdFunctor(const Sparse &A, double sigma = 0)
      : A_(A), sigma_(sigma) {}

  int rows() const { return A_.rows(); }
  int cols() const { return A_.cols(); }

  // Matrix-vector multiplication operation
  void perform_op(const double *x, double *y) const {
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
    const Sparse &A, const Matrix &S, double *minEigenValue,
    Vector *minEigenVector = 0, size_t *numIterations = 0,
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
      *minEigenVector = lmEigenValueSolver.eigenvectors(1).col(0);
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
    *minEigenVector = minEigenValueSolver.eigenvectors(1).col(0);
    minEigenVector->normalize();  // Ensure that this is a unit vector
  }
  if (numIterations) *numIterations = minEigenValueSolver.num_iterations();
  return true;
}

/* ************************************************************************* */
template <size_t d>
double ShonanAveraging<d>::computeMinEigenValue(const Values &values,
                                                Vector *minEigenVector) const {
  assert(values.size() == nrUnknowns());
  const Matrix S = StiefelElementMatrix(values);
  auto A = computeA(S);

  double minEigenValue;
  bool success = SparseMinimumEigenValue(A, S, &minEigenValue, minEigenVector);
  if (!success) {
    throw std::runtime_error(
        "SparseMinimumEigenValue failed to compute minimum eigenvalue.");
  }
  return minEigenValue;
}

/* ************************************************************************* */
template <size_t d>
double ShonanAveraging<d>::computeMinEigenValueAP(const Values &values,
                                                Vector *minEigenVector) const {
  assert(values.size() == nrUnknowns());
  const Matrix S = StiefelElementMatrix(values);
  auto A = computeA(S);

  double minEigenValue = 0;
  bool success = PowerMinimumEigenValue(A, S, minEigenValue, minEigenVector);
  if (!success) {
    throw std::runtime_error(
        "PowerMinimumEigenValue failed to compute minimum eigenvalue.");
  }
  return minEigenValue;
}

/* ************************************************************************* */
template <size_t d>
std::pair<double, Vector> ShonanAveraging<d>::computeMinEigenVector(
    const Values &values) const {
  Vector minEigenVector;
  double minEigenValue = computeMinEigenValue(values, &minEigenVector);
  return std::make_pair(minEigenValue, minEigenVector);
}

/* ************************************************************************* */
template <size_t d>
bool ShonanAveraging<d>::checkOptimality(const Values &values) const {
  double minEigenValue = computeMinEigenValue(values);
  return minEigenValue > parameters_.optimalityThreshold;
}

/* ************************************************************************* */
template <size_t d>
VectorValues ShonanAveraging<d>::TangentVectorValues(size_t p,
                                                     const Vector &v) {
  VectorValues delta;
  // Create a tangent direction xi with eigenvector segment v_i
  const size_t dimension = SOn::Dimension(p);
  double sign0 = pow(-1.0, round((p + 1) / 2) + 1);
  for (size_t i = 0; i < v.size() / d; i++) {
    // Assumes key is 0-based integer
    const auto v_i = v.segment<d>(d * i);
    Vector xi = Vector::Zero(dimension);
    double sign = sign0;
    for (size_t j = 0; j < d; j++) {
      xi(j + p - d - 1) = sign * v_i(d - j - 1);
      sign = -sign;
    }
    delta.insert(i, xi);
  }
  return delta;
}

/* ************************************************************************* */
template <size_t d>
Matrix ShonanAveraging<d>::riemannianGradient(size_t p,
                                              const Values &values) const {
  Matrix S_dot = StiefelElementMatrix(values);
  // calculate the gradient of F(Q_dot) at Q_dot
  Matrix euclideanGradient = 2 * (L_ * (S_dot.transpose())).transpose();
  // cout << "euclidean gradient rows and cols" << euclideanGradient.rows() <<
  // "\t" << euclideanGradient.cols() << endl;

  // project the gradient onto the entire euclidean space
  Matrix symBlockDiagProduct(p, d * nrUnknowns());
  for (size_t i = 0; i < nrUnknowns(); i++) {
    // Compute block product
    const size_t di = d * i;
    const Matrix P = S_dot.middleCols<d>(di).transpose() *
                     euclideanGradient.middleCols<d>(di);
    // Symmetrize this block
    Matrix S = .5 * (P + P.transpose());
    // Compute S_dot * S and set corresponding block
    symBlockDiagProduct.middleCols<d>(di) = S_dot.middleCols<d>(di) * S;
  }
  Matrix riemannianGradient = euclideanGradient - symBlockDiagProduct;
  return riemannianGradient;
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::LiftwithDescent(size_t p, const Values &values,
                                           const Vector &minEigenVector) {
  Values lifted = LiftTo<SOn>(p, values);
  VectorValues delta = TangentVectorValues(p, minEigenVector);
  return lifted.retract(delta);
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::initializeWithDescent(
    size_t p, const Values &values, const Vector &minEigenVector,
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
    Values Qplus = LiftwithDescent(p, values, alpha * minEigenVector);
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
    Values Qplus = LiftwithDescent(p, values, aMin * minEigenVector);
    return Qplus;
  }

  return LiftwithDescent(p, values, alpha * minEigenVector);
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::initializeRandomly(std::mt19937 &rng) const {
  Values initial;
  for (size_t j = 0; j < nrUnknowns(); j++) {
    initial.insert(j, Rot::Random(rng));
  }
  return initial;
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::initializeRandomly() const {
  return initializeRandomly(kRandomNumberGenerator);
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::initializeRandomlyAt(size_t p,
                                                std::mt19937 &rng) const {
  const Values randomRotations = initializeRandomly(rng);
  return LiftTo<Rot3>(p, randomRotations);  // lift to p!
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::initializeRandomlyAt(size_t p) const {
  return initializeRandomlyAt(p, kRandomNumberGenerator);
}

/* ************************************************************************* */
template <size_t d>
std::pair<Values, double> ShonanAveraging<d>::run(const Values &initialEstimate,
                                                  size_t pMin,
                                                  size_t pMax) const {
  Values Qstar;
  Values initialSOp = LiftTo<Rot>(pMin, initialEstimate);  // lift to pMin!
  for (size_t p = pMin; p <= pMax; p++) {
    // Optimize until convergence at this level
    Qstar = tryOptimizingAt(p, initialSOp);
    if (parameters_.getUseHuber() || !parameters_.getCertifyOptimality()) {
      // in this case, there is no optimality certification
      if (pMin != pMax) {
        throw std::runtime_error(
            "When using robust norm, Shonan only tests a single rank. Set pMin = pMax");
      }
      const Values SO3Values = roundSolution(Qstar);
      return std::make_pair(SO3Values, 0);
    } else {
      // Check certificate of global optimality
      Vector minEigenVector;
      double minEigenValue = computeMinEigenValue(Qstar, &minEigenVector);
      if (minEigenValue > parameters_.optimalityThreshold) {
        // If at global optimum, round and return solution
        const Values SO3Values = roundSolution(Qstar);
        return std::make_pair(SO3Values, minEigenValue);
      }

      // Not at global optimimum yet, so check whether we will go to next level
      if (p != pMax) {
        // Calculate initial estimate for next level by following minEigenVector
        initialSOp =
            initializeWithDescent(p + 1, Qstar, minEigenVector, minEigenValue);
      }
    }
  }
  throw std::runtime_error("Shonan::run did not converge for given pMax");
}

/* ************************************************************************* */
// Explicit instantiation for d=2
template class ShonanAveraging<2>;

ShonanAveraging2::ShonanAveraging2(const Measurements &measurements,
                                   const Parameters &parameters)
    : ShonanAveraging<2>(maybeRobust(measurements, parameters.getUseHuber()),
                         parameters) {}

ShonanAveraging2::ShonanAveraging2(string g2oFile, const Parameters &parameters)
    : ShonanAveraging<2>(maybeRobust(parseMeasurements<Rot2>(g2oFile),
                                     parameters.getUseHuber()),
                         parameters) {}

// Extract Rot2 measurement from Pose2 betweenfactors
// Modeled after similar function in dataset.cpp
static BinaryMeasurement<Rot2> convertPose2ToBinaryMeasurementRot2(
    const BetweenFactor<Pose2>::shared_ptr &f) {
  auto gaussian =
      boost::dynamic_pointer_cast<noiseModel::Gaussian>(f->noiseModel());
  if (!gaussian)
    throw std::invalid_argument(
        "parseMeasurements<Rot2> can only convert Pose2 measurements "
        "with Gaussian noise models.");
  const Matrix3 M = gaussian->covariance();
  // the (2,2) entry of Pose2's covariance corresponds to Rot2's covariance
  // because the tangent space of Pose2 is ordered as (vx, vy, w)
  auto model = noiseModel::Isotropic::Variance(1, M(2, 2));
  return BinaryMeasurement<Rot2>(f->key<1>(), f->key<2>(), f->measured().rotation(),
                                 model);
}
    
static ShonanAveraging2::Measurements extractRot2Measurements(
    const BetweenFactorPose2s &factors) {
  ShonanAveraging2::Measurements result;
  result.reserve(factors.size());
  for (auto f : factors) result.push_back(convertPose2ToBinaryMeasurementRot2(f));
  return result;
}

ShonanAveraging2::ShonanAveraging2(const BetweenFactorPose2s &factors,
                                   const Parameters &parameters)
    : ShonanAveraging<2>(maybeRobust(extractRot2Measurements(factors),
                                     parameters.getUseHuber()),
                         parameters) {}    
    
/* ************************************************************************* */
// Explicit instantiation for d=3
template class ShonanAveraging<3>;

ShonanAveraging3::ShonanAveraging3(const Measurements &measurements,
                                   const Parameters &parameters)
    : ShonanAveraging<3>(maybeRobust(measurements, parameters.getUseHuber()),
                         parameters) {}

ShonanAveraging3::ShonanAveraging3(string g2oFile, const Parameters &parameters)
    : ShonanAveraging<3>(maybeRobust(parseMeasurements<Rot3>(g2oFile),
                                     parameters.getUseHuber()),
                         parameters) {}

// TODO(frank): Deprecate after we land pybind wrapper

// Extract Rot3 measurement from Pose3 betweenfactors
// Modeled after similar function in dataset.cpp
static BinaryMeasurement<Rot3> convert(
    const BetweenFactor<Pose3>::shared_ptr &f) {
  auto gaussian =
      boost::dynamic_pointer_cast<noiseModel::Gaussian>(f->noiseModel());
  if (!gaussian)
    throw std::invalid_argument(
        "parseMeasurements<Rot3> can only convert Pose3 measurements "
        "with Gaussian noise models.");
  const Matrix6 M = gaussian->covariance();
  // the upper-left 3x3 sub-block of Pose3's covariance corresponds to Rot3's covariance
  // because the tangent space of Pose3 is ordered as (w,T) where w and T are both Vector3's
  auto model = noiseModel::Gaussian::Covariance(M.block<3, 3>(0, 0));
  return BinaryMeasurement<Rot3>(f->key<1>(), f->key<2>(), f->measured().rotation(),
                                 model);
}

static ShonanAveraging3::Measurements extractRot3Measurements(
    const BetweenFactorPose3s &factors) {
  ShonanAveraging3::Measurements result;
  result.reserve(factors.size());
  for (auto f : factors) result.push_back(convert(f));
  return result;
}

ShonanAveraging3::ShonanAveraging3(const BetweenFactorPose3s &factors,
                                   const Parameters &parameters)
    : ShonanAveraging<3>(maybeRobust(extractRot3Measurements(factors),
                                     parameters.getUseHuber()),
                         parameters) {}

/* ************************************************************************* */
}  // namespace gtsam
