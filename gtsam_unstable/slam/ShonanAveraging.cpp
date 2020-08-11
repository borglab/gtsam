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
#include <gtsam/slam/FrobeniusFactor.h>
#include <gtsam/slam/KarcherMeanFactor-inl.h>
#include <gtsam_unstable/slam/ShonanGaugeFactor.h>

#include <SymEigsSolver.h>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <complex>
#include <iostream>
#include <map>
#include <random>
#include <vector>

namespace gtsam {

static std::mt19937 kRandomNumberGenerator(42);

using Sparse = Eigen::SparseMatrix<double>;

/* ************************************************************************* */
ShonanAveragingParameters::ShonanAveragingParameters(
    const LevenbergMarquardtParams &_lm, const std::string &method,
    double optimalityThreshold, double alpha, double beta, double gamma)
    : lm(_lm), optimalityThreshold(optimalityThreshold), alpha(alpha),
      beta(beta), gamma(gamma) {
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
template <size_t d>
ShonanAveraging<d>::ShonanAveraging(const Factors &factors,
                                    const size_t nrUnknowns,
                                    const ShonanAveragingParameters &parameters)
    : parameters_(parameters), factors_(factors), nrUnknowns_(nrUnknowns) {
  Q_ = buildQ();
  D_ = buildD();
  L_ = D_ - Q_;
}

/* ************************************************************************* */
// Calculate number of poses referenced by factors
template <size_t d>
static size_t
NrPosesReferenced(const typename ShonanAveraging<d>::Factors &factors) {
  std::set<Key> keys;
  for (const auto &factor : factors) {
    keys.insert(factor->key1());
    keys.insert(factor->key2());
  }
  return keys.size();
}

/* ************************************************************************* */
template <size_t d>
ShonanAveraging<d>::ShonanAveraging(const Factors &factors,
                                    const ShonanAveragingParameters &parameters)
    : ShonanAveraging(factors, NrPosesReferenced<d>(factors), parameters) {}

/* ************************************************************************* */
template <size_t d>
ShonanAveraging<d>::ShonanAveraging(const string &g2oFile,
                                    const ShonanAveragingParameters &parameters)
    : ShonanAveraging(parse3DFactors(g2oFile), parameters) {}

/* ************************************************************************* */
template <size_t d>
NonlinearFactorGraph ShonanAveraging<d>::buildGraphAt(size_t p) const {
  NonlinearFactorGraph graph;
  auto G = boost::make_shared<Matrix>(SOn::VectorizedGenerators(p));
  for (const auto &factor : factors_) {
    const auto &keys = factor->keys();
    const auto &Rij = factor->measured();
    const auto &model = factor->noiseModel();
    graph.emplace_shared<FrobeniusWormholeFactor>(keys[0], keys[1], Rij, p,
                                                  model, G);
  }
  return graph;
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::initializeRandomlyAt(size_t p) const {
  Values initial;
  for (size_t j = 0; j < nrUnknowns(); j++) {
    initial.insert(j, SOn::Random(kRandomNumberGenerator, p));
  }
  return initial;
}

/* ************************************************************************* */
template <size_t d>
double ShonanAveraging<d>::costAt(size_t p, const Values& values) const {
  const NonlinearFactorGraph graph = buildGraphAt(p);
  return graph.error(values);
}

/* ************************************************************************* */
template <size_t d>
boost::shared_ptr<LevenbergMarquardtOptimizer>
ShonanAveraging<d>::createOptimizerAt(
    size_t p, const boost::optional<Values> &initialEstimate) const {
  // Build graph
  NonlinearFactorGraph graph = buildGraphAt(p);

  // Initialize randomly if no initial estimate is given
  // TODO(frank): add option to do chordal init
  const Values initial =
      initialEstimate ? *initialEstimate : initializeRandomlyAt(p);

  const size_t dim = SOn::Dimension(p);

  // Anchor prior is only added here as depends on initial value (and cost is zero)
  if (parameters_.alpha > 0) {
    size_t i;
    Rot3 value;
    std::tie(i, value) = parameters_.anchor;
    auto model = noiseModel::Isotropic::Precision(dim, parameters_.alpha);
    graph.emplace_shared<PriorFactor<SOn>>(i, SOn::Lift(p, value.matrix()), model);
  }
  
  // TODO(frank): add Karcher prior when building graph?
  if (parameters_.beta > 0) {
    graph.emplace_shared<KarcherMeanFactor<SOn>>(graph.keys(), dim);
  }

  // TODO(frank): definitely add Gauge factors when building graph?
  if (parameters_.gamma > 0 && p > d + 1) {
    for (auto key : graph.keys())
      graph.emplace_shared<ShonanGaugeFactor>(key, p, d, parameters_.gamma);
  }

  // Optimize
  auto optimizer = boost::make_shared<LevenbergMarquardtOptimizer>(
      graph, initial, parameters_.lm);
  return optimizer;
}

/* ************************************************************************* */
template <size_t d>
Values ShonanAveraging<d>::tryOptimizingAt(
    size_t p, const boost::optional<Values>& initialEstimate) const {
  boost::shared_ptr<LevenbergMarquardtOptimizer> lm =
      createOptimizerAt(p, initialEstimate);
  Values result = lm->optimize();
  return result;
}

/* ************************************************************************* */
// Project to pxdN Stiefel manifold
template <size_t d=3>
static Matrix StiefelElementMatrix(const Values &values) {
  const size_t N = values.size();
  const size_t p = values.at<SOn>(0).rows();
  Matrix S(p, N * d);
  for (const auto& it: values.filter<SOn>()) {
    S.middleCols<d>(it.key * d) = it.value.matrix().leftCols<d>();  // project Qj to Stiefel
  }
  return S;
}

/* ************************************************************************* */
template <>
Values ShonanAveraging<2>::projectFrom(size_t p, const Values &values) const {
  Values result;
  for (const auto &it : values.filter<SOn>()) {
    assert(it.value.rows() == p);
    const auto &M = it.value.matrix();
    const Rot2 R = Rot2::atan2(M(1, 0), M(0, 0));
    result.insert(it.key, R);
  }
  return result;
}

template <>
Values ShonanAveraging<3>::projectFrom(size_t p, const Values &values) const {
  Values result;
  for (const auto &it : values.filter<SOn>()) {
    assert(it.value.rows() == p);
    const auto &M = it.value.matrix();
    const Rot3 R = Rot3::ClosestTo(M.topLeftCorner<3, 3>());
    result.insert(it.key, R);
  }
  return result;
}

/* ************************************************************************* */
template <size_t d>
static Matrix RoundSolutionS(const Matrix &S) {
  const size_t N = S.cols()/d;
  // First, compute a thin SVD of S
  Eigen::JacobiSVD<Matrix> svd(S, Eigen::ComputeThinV);
  const Vector sigmas = svd.singularValues();

  // Construct a diagonal matrix comprised of the first d singular values
  using DiagonalMatrix = Eigen::DiagonalMatrix<double, d>;
  DiagonalMatrix Sigma_d;
  Sigma_d.diagonal() = sigmas;

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
template <> Values ShonanAveraging<2>::roundSolutionS(const Matrix &S) const {
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

template <> Values ShonanAveraging<3>::roundSolutionS(const Matrix &S) const {
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
Values ShonanAveraging<d>::roundSolution(const Values& values) const {
  // Project to pxdN Stiefel manifold...
  Matrix S = StiefelElementMatrix(values);
  // ...and call version above.
  return roundSolutionS(S);
}

/* ************************************************************************* */
template <size_t d>
double ShonanAveraging<d>::cost(const Values &values) const {
  NonlinearFactorGraph graph;
  for (const auto &factor : factors_) {
    const auto &keys = factor->keys();
    const auto &Rij = factor->measured();
    const auto &model = factor->noiseModel();
    graph.emplace_shared<FrobeniusBetweenFactor<SO3>>(keys[0], keys[1],
                                                      SO3(Rij.matrix()), model);
  }
  // Finally, project each dxd rotation block to SO(d)
  Values SO3_values;
  for (const auto &it : values.filter<Rot3>()) {
    SO3_values.insert(it.key, SO3(it.value.matrix()));
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
template <size_t d>
Sparse ShonanAveraging<d>::buildD() const {
  // Each measurement contributes 2*d elements along the diagonal of the
  // degree matrix.
  static constexpr size_t stride = 2 * d;

  // Reserve space for triplets
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(stride * factors_.size());

  for (const auto &factor : factors_) {
    // Get pose keys
    const auto &keys = factor->keys();

    // Get kappa from noise model
    double kappa = Kappa(factor);

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
  triplets.reserve(stride * factors_.size());

  for (const auto& factor : factors_) {
    // Get pose keys
    const auto& keys = factor->keys();

    // Extract rotation measurement
    const Matrix3 Rij = factor->measured().matrix();

    // Get kappa from noise model
    double kappa = Kappa(factor);

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
template<size_t d>
Sparse ShonanAveraging<d>::computeLambda(const Matrix& S) const {
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
template<size_t d>
Sparse ShonanAveraging<d>::computeLambda(
    const Values& values) const {
  // Project to pxdN Stiefel manifold...
  Matrix S = StiefelElementMatrix(values);
  // ...and call version above.
  return computeLambda(S);
}

/* ************************************************************************* */
template<size_t d>
Sparse ShonanAveraging<d>::computeA(const Values& values) const {
  assert(values.size() == nrUnknowns());
  const Matrix S = StiefelElementMatrix(values);
  auto Lambda = computeLambda(S);
  return Lambda - Q_;
}

/* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

/** This is a lightweight struct used in conjunction with Spectra to compute
 * the minimum eigenvalue and eigenvector of a sparse matrix A; it has a single
 * nontrivial function, perform_op(x,y), that computes and returns the product
 * y = (A + sigma*I) x */
struct MatrixProdFunctor {
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Sparse& A_;

  // Spectral shift
  double sigma_;

  // Constructor
  explicit MatrixProdFunctor(const Sparse& A, double sigma = 0)
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
    const Sparse& A, const Matrix& S, double* minEigenValue,
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
template<size_t d>
Sparse ShonanAveraging<d>::computeA(const Matrix& S) const {
  auto Lambda = computeLambda(S);
  return Lambda - Q_;
}

/* ************************************************************************* */
template<size_t d>
double ShonanAveraging<d>::computeMinEigenValue(const Values& values,
                                             Vector* minEigenVector) const {
  assert(values.size() == nrUnknowns());
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
template<size_t d>
std::pair<double, Vector> ShonanAveraging<d>::computeMinEigenVector(
    const Values& values) const {
  Vector minEigenVector;
  double minEigenValue = computeMinEigenValue(values, &minEigenVector);
  return std::make_pair(minEigenValue, minEigenVector);
}

/* ************************************************************************* */
template<size_t d>
bool ShonanAveraging<d>::checkOptimality(const Values& values) const {
  double minEigenValue = computeMinEigenValue(values);
  return minEigenValue > parameters_.optimalityThreshold;
}

/* ************************************************************************* */
/// Create a tangent direction xi with eigenvector segment v_i
template<size_t d>
Vector ShonanAveraging<d>::MakeATangentVector(size_t p, const Vector &v, size_t i) {
  // Create a tangent direction xi with eigenvector segment v_i
  const size_t dimension = SOn::Dimension(p);
  const auto v_i = v.segment<d>(d * i);
  Vector xi = Vector::Zero(dimension);
  double sign = pow(-1.0, round((p + 1) / 2) + 1);
  for (size_t j = 0; j < d; j++) {
    xi(j + p - d - 1) = sign * v_i(d - j - 1);
    sign = -sign;
  }
  return xi;
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
  for (auto it : lifted.filter<SOn>()) {
    // Create a tangent direction xi with eigenvector segment v_i
    // Assumes key is 0-based integer
    const Vector xi = MakeATangentVector(p, minEigenVector, it.key);
    // Move the old value in the descent direction
    it.value = it.value.retract(xi);
  }
  return lifted;
}

/* ************************************************************************* */
template<size_t d>
Values ShonanAveraging<d>::initializeWithDescent(
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
template<size_t d>
std::pair<Values, double>
ShonanAveraging<d>::run(size_t pMin, size_t pMax, bool withDescent,
                     const boost::optional<Values>& initialEstimate) const {
  Values Qstar;
  Values initial = (initialEstimate) ? LiftTo<Rot3>(pMin, *initialEstimate)
                                     : initializeRandomlyAt(pMin);
  for (size_t p = pMin; p <= pMax; p++) {
    Qstar = tryOptimizingAt(p, initial);
    Vector minEigenVector;
    double minEigenValue = computeMinEigenValue(Qstar, &minEigenVector);
    if (minEigenValue > parameters_.optimalityThreshold) {
      const Values SO3Values = roundSolution(Qstar);
      return std::make_pair(SO3Values, minEigenValue);
    }
    if (p != pMax) {
      initial = withDescent ? initializeWithDescent(
                                  p + 1, Qstar, minEigenVector, minEigenValue)
                            : initializeRandomlyAt(p + 1);
    }
  }
  throw std::runtime_error("Shonan::run did not converge for given pMax");
}

/* ************************************************************************* */
template<size_t d>
std::pair<Values, double> ShonanAveraging<d>::runWithRandom(
    size_t pMin, size_t pMax,
    const boost::optional<Values>& initialEstimate) const {
  return run(pMin, pMax, false, initialEstimate);
}

/* ************************************************************************* */
template<size_t d>
std::pair<Values, double> ShonanAveraging<d>::runWithDescent(
    size_t pMin, size_t pMax,
    const boost::optional<Values>& initialEstimate) const {
  return run(pMin, pMax, true, initialEstimate);
}

/* ************************************************************************* */
// Explicit instantiation for d=2 and d=3
template class ShonanAveraging<2>;
template class ShonanAveraging<3>;

/* ************************************************************************* */
}  // namespace gtsam
