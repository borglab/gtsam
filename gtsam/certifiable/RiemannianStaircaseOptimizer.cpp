/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.cpp
 * @brief   Implementation of the Burer-Monteiro staircase.
 */

#include <gtsam/certifiable/RiemannianStaircaseOptimizer.h>

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/linear/HessianFactor.h>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/SparseCholesky>

/// Work around a GCC 16 false positive in Spectra's SortEigenvalue comparator.
#if defined(__GNUC__) && __GNUC__ == 16
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif

#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/SymEigsSolver.h>
#include <Spectra/Util/SimpleRandom.h>

#if defined(__GNUC__) && __GNUC__ == 16
#pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace gtsam {
namespace {

using Clock = std::chrono::steady_clock;

double ElapsedSeconds(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

}  // namespace

/* ************************************************************************* */
RiemannianStaircaseOptimizer::RiemannianStaircaseOptimizer(
    const NonlinearFactorGraph& graph, const Values& initialValues,
    const RiemannianStaircaseParams& params)
    : graph_(graph), initialValues_(initialValues), params_(params) {
  validateParams(params_);
  if (!params_.almParams) {
    params_.almParams = std::make_shared<AugmentedLagrangianParams>();
  }
  const auto buildStart = Clock::now();
  try {
    pMinQcqp_ = std::make_shared<QcqpProblem>(graph_, params_.pMin);
  } catch (const std::exception& e) {
    throw std::invalid_argument(
        std::string("RiemannianStaircaseOptimizer: QcqpProblem(graph, pMin) "
                    "failed — is the source graph QCQP-representable? "
                    "Underlying error: ") +
        e.what());
  }
  // Q is the same matrix at every staircase level, so assemble it here and
  // let each level reuse it.
  dataMatrix_ = buildDataMatrix(*pMinQcqp_, Layout::From(initialValues_));
  pMinQcqpBuildTime_ = ElapsedSeconds(buildStart);
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::Layout
RiemannianStaircaseOptimizer::Layout::From(const Values& values) {
  Layout layout;
  // Sorted-Key order so the layout is deterministic across runs.
  std::set<Key> sortedKeys;
  for (const auto& [key, _] : values.extract<Matrix>()) sortedKeys.insert(key);

  size_t offset = 0;
  for (Key key : sortedKeys) {
    const Matrix X = values.at<Matrix>(key);
    const size_t rowDim = static_cast<size_t>(X.rows());
    layout.slices.emplace(key, LayoutSlice{offset, rowDim});
    offset += rowDim;
  }
  layout.totalDim = offset;
  return layout;
}

/* ************************************************************************* */
size_t RiemannianStaircaseOptimizer::Layout::offsetOf(Key key) const {
  auto it = slices.find(key);
  if (it == slices.end()) {
    throw std::out_of_range(
        "RiemannianStaircaseOptimizer::Layout::offsetOf: key not in layout.");
  }
  return it->second.offset;
}

/* ************************************************************************* */
size_t RiemannianStaircaseOptimizer::Layout::rowDimOf(Key key) const {
  auto it = slices.find(key);
  if (it == slices.end()) {
    throw std::out_of_range(
        "RiemannianStaircaseOptimizer::Layout::rowDimOf: key not in layout.");
  }
  return it->second.rowDim;
}

/* ************************************************************************* */
const RiemannianStaircaseOptimizer::LayoutSlice&
RiemannianStaircaseOptimizer::Layout::sliceOf(Key key) const {
  auto it = slices.find(key);
  if (it == slices.end()) {
    throw std::out_of_range(
        "RiemannianStaircaseOptimizer::Layout::sliceOf: key not in layout.");
  }
  return it->second;
}

/* ************************************************************************* */
bool RiemannianStaircaseOptimizer::Layout::conformsTo(
    const Values& values) const {
  size_t matrixCount = 0;
  for (const auto& [key, X] : values.extract<Matrix>()) {
    ++matrixCount;
    auto it = slices.find(key);
    if (it == slices.end()) return false;
    if (static_cast<size_t>(X.rows()) != it->second.rowDim) return false;
  }
  return matrixCount == slices.size();
}

/* ************************************************************************* */
Matrix RiemannianStaircaseOptimizer::Layout::stack(const Values& values) const {
  if (slices.empty()) return Matrix();
  // Probe column count from the first present value.
  size_t cols = 0;
  bool first = true;
  for (const auto& [key, X] : values.extract<Matrix>()) {
    if (first) {
      cols = static_cast<size_t>(X.cols());
      first = false;
    } else if (static_cast<size_t>(X.cols()) != cols) {
      throw std::invalid_argument(
          "Layout::stack: all matrix-valued variables must share the same "
          "column count.");
    }
  }
  Matrix Y = Matrix::Zero(static_cast<int>(totalDim), static_cast<int>(cols));
  for (const auto& [key, slice] : slices) {
    if (!values.exists(key)) {
      throw std::invalid_argument("Layout::stack: missing key in values.");
    }
    const Matrix X = values.at<Matrix>(key);
    if (static_cast<size_t>(X.rows()) != slice.rowDim) {
      throw std::invalid_argument(
          "Layout::stack: row count mismatch for key.");
    }
    Y.block(static_cast<int>(slice.offset), 0,
            static_cast<int>(slice.rowDim), static_cast<int>(cols)) = X;
  }
  return Y;
}

/* ************************************************************************* */
Values RiemannianStaircaseOptimizer::Layout::unstack(const Matrix& Y) const {
  if (static_cast<size_t>(Y.rows()) != totalDim) {
    throw std::invalid_argument(
        "Layout::unstack: input row count does not match totalDim.");
  }
  Values values;
  for (const auto& [key, slice] : slices) {
    values.insert(
        key,
        Matrix(Y.block(static_cast<int>(slice.offset), 0,
                       static_cast<int>(slice.rowDim), Y.cols())));
  }
  return values;
}

/* ************************************************************************* */
size_t RiemannianStaircaseOptimizer::Layout::maxRowDim() const {
  size_t maxDim = 0;
  for (const auto& [_, slice] : slices) {
    if (slice.rowDim > maxDim) maxDim = slice.rowDim;
  }
  return maxDim;
}

/* ************************************************************************* */
void RiemannianStaircaseOptimizer::validateParams(
    const RiemannianStaircaseParams& params) {
  const size_t maxEigenIndex =
      static_cast<size_t>(std::numeric_limits<Eigen::Index>::max());
  if (params.pMin > maxEigenIndex || params.pMax > maxEigenIndex) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer: pMin and pMax must fit in "
        "Eigen::Index.");
  }
  if (params.pMin > params.pMax) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer: pMin must be <= pMax.");
  }
  if (params.pMin == 0) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer: pMin must be >= 1.");
  }
}

/* ************************************************************************* */
Values RiemannianStaircaseOptimizer::padInitialValues(const Values& Y,
                                                     size_t pMin) {
  if (pMin >
      static_cast<size_t>(std::numeric_limits<Eigen::Index>::max())) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::padInitialValues: pMin must fit in "
        "Eigen::Index.");
  }
  const Eigen::Index targetCols = static_cast<Eigen::Index>(pMin);

  Values padded = Y;
  for (const auto& [key, X] : Y.extract<Matrix>()) {
    const Eigen::Index cols = X.cols();
    if (cols == targetCols) continue;
    if (cols > targetCols) {
      throw std::invalid_argument(
          "RiemannianStaircaseOptimizer::padInitialValues: initial value has "
          "more columns than pMin; either lower pMin or drop the extra "
          "columns.");
    }
    Matrix pad = Matrix::Zero(X.rows(), targetCols);
    pad.leftCols(cols) = X;
    padded.update(key, pad);
  }
  return padded;
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::InnerSolveResult
RiemannianStaircaseOptimizer::runLocalSolver(
    const QcqpProblem& qcqp, const Values& Y0,
    AugmentedLagrangianParams::shared_ptr almParams) {
  // Local copy so we can force storeOptProgress without mutating the caller's.
  auto localParams = almParams
      ? std::make_shared<AugmentedLagrangianParams>(*almParams)
      : std::make_shared<AugmentedLagrangianParams>();
  localParams->storeOptProgress = true;

  AugmentedLagrangianOptimizer alm(qcqp, Y0, localParams);
  Values Y = alm.optimize();
  if (alm.progress().empty()) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::runLocalSolver: ALM produced no "
        "progress.");
  }
  const auto& last = alm.progress().back();
  return {Y, last.lambdaEq, last.augmentedLagrangianStationarity, last.muEq};
}

/* ************************************************************************* */
Eigen::SparseMatrix<double> RiemannianStaircaseOptimizer::buildDataMatrix(
    const QcqpProblem& qcqp, const Layout& layout) {
  const size_t n = layout.totalDim;
  std::vector<Eigen::Triplet<double>> triplets;

  // QpCost stores each Q_ij as a K-Kronecker expansion; the natural
  // rowDim_i x rowDim_j block is the top-left corner of each expanded block.
  for (const auto& factor : qcqp.costs()) {
    auto qpCost = std::dynamic_pointer_cast<const QpCost>(factor);
    if (!qpCost) {
      throw std::runtime_error(
          "buildDataMatrix: non-QpCost factor in qcqp.costs().");
    }
    const HessianFactor& hessian = qpCost->hessianFactor();
    const KeyVector& keys = qpCost->keys();
    for (size_t i = 0; i < keys.size(); ++i) {
      const size_t off_i = layout.offsetOf(keys[i]);
      const size_t rd_i = layout.rowDimOf(keys[i]);
      for (size_t j = i; j < keys.size(); ++j) {
        const size_t off_j = layout.offsetOf(keys[j]);
        const size_t rd_j = layout.rowDimOf(keys[j]);
        const Matrix expandedBlock = hessian.info().block(i, j);
        const Matrix Q_ij = expandedBlock.topLeftCorner(
            static_cast<int>(rd_i), static_cast<int>(rd_j));
        for (size_t r = 0; r < rd_i; ++r) {
          for (size_t c = 0; c < rd_j; ++c) {
            const double v = Q_ij(static_cast<int>(r), static_cast<int>(c));
            if (v == 0.0) continue;
            triplets.emplace_back(off_i + r, off_j + c, v);
            if (i != j) {
              triplets.emplace_back(off_j + c, off_i + r, v);
            }
          }
        }
      }
    }
  }

  Eigen::SparseMatrix<double> Q(static_cast<int>(n), static_cast<int>(n));
  Q.setFromTriplets(triplets.begin(), triplets.end());
  Q.makeCompressed();
  return Q;
}

/* ************************************************************************* */
Eigen::SparseMatrix<double> RiemannianStaircaseOptimizer::buildMultiplierMatrix(
    const QcqpProblem& qcqp, const Layout& layout,
    const std::vector<Vector>& lambdaEq) {
  if (lambdaEq.size() != qcqp.eConstraints().size()) {
    throw std::runtime_error(
        "buildMultiplierMatrix: lambdaEq size must match equality constraint "
        "count.");
  }

  const size_t n = layout.totalDim;
  std::vector<Eigen::Triplet<double>> triplets;

  // A*(lambda) = sum_m (lambda_m / sigma_m) * A_m. ALM tracks lambdas paired
  // with the whitened residual h_m / sigma_m, so divide by sigma_m to get the
  // multiplier on A_m.
  // lambdaEq is indexed in eConstraints() order (ALM convention).
  size_t m = 0;
  for (const auto& factor : qcqp.eConstraints()) {
    auto qcFactor =
        std::dynamic_pointer_cast<const QuadraticEqualityConstraintFactor>(
            factor);
    if (!qcFactor) {
      throw std::runtime_error(
          "buildMultiplierMatrix: every equality constraint must be a "
          "QuadraticEqualityConstraintFactor.");
    }
    const QuadraticConstraint& qc = qcFactor->quadraticConstraint();
    if (lambdaEq[m].size() != 1) {
      throw std::runtime_error(
          "buildMultiplierMatrix: QuadraticEqualityConstraintFactor expects a "
          "scalar multiplier; got lambdaEq[m].size() != 1.");
    }
    const double lambdaM = lambdaEq[m](0) / qc.sigma();
    const Key key = qc.key();
    const size_t off = layout.offsetOf(key);
    const size_t rd = layout.rowDimOf(key);
    const Matrix& A = qc.A();
    if (static_cast<size_t>(A.rows()) != rd ||
        static_cast<size_t>(A.cols()) != rd) {
      throw std::runtime_error(
          "buildMultiplierMatrix: constraint A size does not match key "
          "rowDim.");
    }
    for (size_t r = 0; r < rd; ++r) {
      for (size_t c = 0; c < rd; ++c) {
        const double v = lambdaM * A(static_cast<int>(r), static_cast<int>(c));
        if (v == 0.0) continue;
        triplets.emplace_back(off + r, off + c, v);
      }
    }
    ++m;
  }

  Eigen::SparseMatrix<double> Aadj(static_cast<int>(n), static_cast<int>(n));
  Aadj.setFromTriplets(triplets.begin(), triplets.end());
  Aadj.makeCompressed();
  return Aadj;
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::LeastSquaresMultipliers
RiemannianStaircaseOptimizer::leastSquaresMultipliers(const QcqpProblem& qcqp,
                                                     const Layout& layout,
                                                     const Values& Y) {
  return leastSquaresMultipliers(qcqp, layout, Y,
                                 buildDataMatrix(qcqp, layout));
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::LeastSquaresMultipliers
RiemannianStaircaseOptimizer::leastSquaresMultipliers(
    const QcqpProblem& qcqp, const Layout& layout, const Values& Y,
    const Eigen::SparseMatrix<double>& dataMatrix) {
  const size_t M = qcqp.eConstraints().size();
  LeastSquaresMultipliers result;
  result.lambdaEq.assign(M, Vector1(0.0));

  // Constraints are unary, so grouping by key partitions them and the
  // least-squares problem below separates over variables.
  struct Entry {
    size_t index;  ///< position in eConstraints(), keeps lambdaEq aligned
    Matrix A;      ///< symmetrized
    double sigma;
  };
  std::map<Key, std::vector<Entry>> groups;
  size_t m = 0;

  // Collect the constraints acting on each variable.
  for (const auto& factor : qcqp.eConstraints()) {
    const auto quadratic =
        std::dynamic_pointer_cast<const QuadraticEqualityConstraintFactor>(
            factor);
    if (!quadratic) {
      throw std::runtime_error(
          "leastSquaresMultipliers: every equality constraint must be a "
          "QuadraticEqualityConstraintFactor.");
    }
    const QuadraticConstraint& constraint = quadratic->quadraticConstraint();
    const Key key = constraint.key();
    const size_t rowDim = layout.rowDimOf(key);
    const Matrix& A = constraint.A();
    if (static_cast<size_t>(A.rows()) != rowDim ||
        static_cast<size_t>(A.cols()) != rowDim) {
      throw std::runtime_error(
          "leastSquaresMultipliers: constraint A size does not match key "
          "rowDim.");
    }
    groups[key].push_back({m, 0.5 * (A + A.transpose()), constraint.sigma()});
    ++m;
  }

  const Matrix Ystack = layout.stack(Y);
  const Matrix G = dataMatrix * Ystack;

  // Solve for each variable's multipliers separately, which the separability
  // above allows: variable n only sees its own block (QY)_n and its own A_k.
  for (const auto& [key, slice] : layout.slices) {
    const Matrix Yn = Ystack.middleRows(slice.offset, slice.rowDim);
    const Matrix Gn = G.middleRows(slice.offset, slice.rowDim);

    const auto group = groups.find(key);
    if (group == groups.end()) {
      // No multiplier can move this block, so the residual is (QY)_n itself.
      result.residual[key] = Gn.norm();
      continue;
    }

    // Columns of W are vec(A_k Y_n); solve min_lambda ||W lambda + vec(G_n)||.
    const auto& entries = group->second;
    const DenseIndex rows = Yn.size();
    Matrix W(rows, static_cast<DenseIndex>(entries.size()));
    for (size_t k = 0; k < entries.size(); ++k) {
      const Matrix column = entries[k].A * Yn;
      W.col(static_cast<DenseIndex>(k)) =
          Eigen::Map<const Vector>(column.data(), column.size());
    }
    const Eigen::Map<const Vector> g(Gn.data(), Gn.size());

    const Vector coefficients = W.colPivHouseholderQr().solve(-g);
    result.residual[key] = (W * coefficients + g).norm();

    // coefficients enter S = Q + sum_m c_m A_m directly; buildCertificate
    // forms S = Q + 2 sum_m (lambda_m / sigma_m) A_m, so convert.
    for (size_t k = 0; k < entries.size(); ++k) {
      result.lambdaEq[entries[k].index] =
          Vector1(0.5 * coefficients(static_cast<DenseIndex>(k)) *
                  entries[k].sigma);
    }
  }

  // Summed over the ordered residual map.
  double squaredTotal = 0.0;
  for (const auto& [key, residual] : result.residual)
    squaredTotal += residual * residual;
  result.totalResidual = std::sqrt(squaredTotal);
  return result;
}

/* ************************************************************************* */
Eigen::SparseMatrix<double> RiemannianStaircaseOptimizer::buildCertificate(
    const QcqpProblem& qcqp, const Layout& layout,
    const std::vector<Vector>& lambdaEq) {
  return buildCertificate(qcqp, layout, lambdaEq,
                          buildDataMatrix(qcqp, layout));
}

/* ************************************************************************* */
Eigen::SparseMatrix<double> RiemannianStaircaseOptimizer::buildCertificate(
    const QcqpProblem& qcqp, const Layout& layout,
    const std::vector<Vector>& lambdaEq,
    const Eigen::SparseMatrix<double>& dataMatrix) {
  Eigen::SparseMatrix<double> S = dataMatrix;
  // The factor of two reconciles the cost and constraint conventions:
  // QpCost is 0.5 * tr(X' Q X) so its gradient is Q Y, while a
  // QuadraticConstraint h = tr(X' A X) - b has gradient 2 A X. Stationarity of
  // L = cost + sum_m lambda_m h_m is therefore (Q + 2 A*(lambda)) Y = 0.
  // So a factor of 2 is added.
  S += 2.0 * buildMultiplierMatrix(qcqp, layout, lambdaEq);
  S.makeCompressed();
  return S;
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::SaddleEscape
RiemannianStaircaseOptimizer::saddleEscapeWithLineSearch(
    const QcqpProblem& liftedQcqp, const Layout& layout, const Values& Ystar,
    const Vector& vMin, const std::vector<Vector>& lambdaEq, double penalty,
    double minEigenvalue, const RiemannianStaircaseParams& params) {
  const auto lift = [&](double alpha) {
    return liftWithDescent(Ystar, vMin, layout, alpha);
  };

  SaddleEscape result;
  result.alpha = params.alpha;
  result.lifted = lift(params.alpha);
  if (!params.useSaddleLineSearch) return result;

  // The merit is ALM's own augmented Lagrangian.
  AugmentedLagrangianState state;
  state.lambdaEq = lambdaEq;
  state.lambdaIneq.assign(liftedQcqp.iConstraints().size(), 0.0);
  state.muEq = penalty;
  state.muIneq = penalty;

  const Values atZeroStep = lift(0.0);
  NonlinearFactorGraph merit;
  try {
    AugmentedLagrangianOptimizer alm(
        liftedQcqp, atZeroStep,
        std::make_shared<AugmentedLagrangianParams>(*params.almParams));
    merit = alm.augmentedLagrangianFunction(state, 0.0);
  } catch (const std::exception&) {
    return result;  /// merit unavailable; fall back to the fixed step
  }

  // Accept the first step that decreases the merit.
  const auto accepts = [&](const Values& lifted, double meritZero,
                           double* decrease) {
    *decrease = meritZero - merit.error(lifted);
    return *decrease > 0.0;
  };

  // Shonan's heuristic schedule: start large, then halve. A lambda_min of
  // exactly zero would divide to infinity, which halving never brings back
  // below alphaMin, so fall back to the fixed start there.
  const double alphaMin = params.alpha;
  const double curvature = std::abs(minEigenvalue);
  double step = 1024.0 * alphaMin;
  if (curvature > 0.0) {
    step = std::max(step, 10.0 * params.saddleStepTolerance / curvature);
  }

  const double meritZero = merit.error(atZeroStep);
  double decrease = 0.0;
  double bestStep = 0.0, bestDecrease = 0.0;
  Values candidate;
  bool accepted = false;
  while (step >= alphaMin) {
    candidate = lift(step);
    if (accepts(candidate, meritZero, &decrease)) {
      accepted = true;
      break;
    }
    if (decrease > bestDecrease) {
      bestDecrease = decrease;
      bestStep = step;
    }
    step *= 0.5;
  }

  // Nothing was accepted; keep the best merit decrease seen, if any.
  if (!accepted) {
    if (!(bestDecrease > 0.0)) {
      if (params.verbose) {
        std::cout << "  [escape] no accepted step; keeping alpha="
                  << params.alpha << std::endl;
      }
      return result;
    }
    step = bestStep;
    decrease = bestDecrease;
    candidate = lift(step);
  }

  result.alpha = step;
  result.lifted = candidate;
  result.meritDecrease = decrease;
  result.descentFound = true;
  if (params.verbose) {
    std::cout << "  [escape] alpha=" << step << " (fixed would be "
              << params.alpha << ") merit decrease=" << decrease << std::endl;
  }
  return result;
}

/* ************************************************************************* */
Values RiemannianStaircaseOptimizer::liftWithDescent(
    const Values& Ystar, const Vector& vMin, const Layout& layout,
    double alpha) {
  if (!layout.conformsTo(Ystar)) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::liftWithDescent: layout does not "
        "match values.");
  }
  if (static_cast<size_t>(vMin.size()) != layout.totalDim) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::liftWithDescent: vMin size does not "
        "match layout.totalDim.");
  }
  Values lifted;
  for (const auto& [key, X] : Ystar.extract<Matrix>()) {
    const LayoutSlice& slice = layout.sliceOf(key);
    Matrix Xlifted(X.rows(), X.cols() + 1);
    Xlifted.leftCols(X.cols()) = X;
    Xlifted.col(X.cols()) =
        alpha * vMin.segment(static_cast<int>(slice.offset),
                             static_cast<int>(slice.rowDim));
    lifted.insert(key, Xlifted);
  }
  return lifted;
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::RoundedSolution
RiemannianStaircaseOptimizer::truncateToRankD(const Values& Y,
                                              const Layout& layout, int d) {
  if (d < 1) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::truncateToRankD: d must be >= 1.");
  }
  const Matrix Ystacked = layout.stack(Y);
  if (d > Ystacked.cols()) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::truncateToRankD: d exceeds the column "
        "count of the stacked matrix.");
  }
  Eigen::JacobiSVD<Matrix> svd(Ystacked,
                               Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix Yd = svd.matrixU().leftCols(d) *
              svd.singularValues().head(d).asDiagonal();
  return {Yd};
}

/* ************************************************************************* */
std::tuple<bool, double, Vector> RiemannianStaircaseOptimizer::verify(
    const Eigen::SparseMatrix<double>& S,
    const RiemannianStaircaseParams& params, const Vector& warmStart) {
  switch (params.verificationMethod) {
    case RiemannianStaircaseParams::VerificationMethod::DenseEigen:
      return verifyDenseEigen(S, params);
    case RiemannianStaircaseParams::VerificationMethod::Spectra:
    default:
      return verifySpectra(S, params, warmStart);
  }
}

/* ************************************************************************* */
std::tuple<bool, double, Vector> RiemannianStaircaseOptimizer::verifyDenseEigen(
    const Eigen::SparseMatrix<double>& S,
    const RiemannianStaircaseParams& params) {
  const Matrix Sdense(S);
  Eigen::SelfAdjointEigenSolver<Matrix> es(Sdense);
  if (es.info() != Eigen::Success) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::verifyDenseEigen: "
        "SelfAdjointEigenSolver failed.");
  }
  const double lambdaMin = es.eigenvalues()(0);
  Vector vMin = es.eigenvectors().col(0);
  const bool passed = (lambdaMin >= -params.eta);
  if (params.verbose) {
    std::cout << "  [dense Eigen] verified=" << (passed ? 1 : 0)
              << " lambda_min=" << lambdaMin << " dim=" << S.rows()
              << std::endl;
  }
  return {passed, lambdaMin, vMin};
}

/* ************************************************************************* */
namespace {

/// PSD check by sparse Cholesky on M = S + eta*I. Success proves
/// lambda_min(S) >= -eta, and is much cheaper than an eigenvalue computation.
bool choleskyCertifies(const Eigen::SparseMatrix<double>& S,
                       const RiemannianStaircaseParams& params) {
  Eigen::SparseMatrix<double> M = S;
  for (int k = 0; k < S.rows(); ++k) M.coeffRef(k, k) += params.eta;
  M.makeCompressed();
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> chol;
  chol.compute(M);
  if (chol.info() != Eigen::Success) return false;
  if (params.verbose) {
    std::cout << "  [Chol] verified=1 dim=" << S.rows() << std::endl;
  }
  return true;
}

/// Applies `y = (A + sigma * I) x` without materializing the shifted matrix.
/// Adapted from Shonan rotation averaging.
struct ShiftedMatrixProduct {
  using Scalar = double;

  const Eigen::SparseMatrix<double>& A;
  double sigma;

  ShiftedMatrixProduct(const Eigen::SparseMatrix<double>& A, double sigma)
      : A(A), sigma(sigma) {}

  int rows() const { return static_cast<int>(A.rows()); }
  int cols() const { return static_cast<int>(A.cols()); }

  void perform_op(const double* x, double* y) const {
    Eigen::Map<const Vector> X(x, rows());
    Eigen::Map<Vector> Y(y, rows());
    Y = A * X + sigma * X;
  }
};

}  // namespace

/* ************************************************************************* */
std::tuple<bool, double, Vector>
RiemannianStaircaseOptimizer::verifySpectra(
    const Eigen::SparseMatrix<double>& S,
    const RiemannianStaircaseParams& params, const Vector& warmStart) {
  const int n = static_cast<int>(S.rows());

  // Stage 1, fast PSD check by Cholesky factorization.
  if (choleskyCertifies(S, params)) {
    return {true, -params.eta, Vector::Zero(n)};
  }

  // Stage 2 computes the minimal eigenpair by a spectral shift.
  // Most of the code is from Shonan rotation averaging's
  // SparseMinimumEigenValue.
  const int nev = 1;
  // Spectra needs 2*nev+1 <= ncv <= n. Below that Lanczos cannot run, and
  // silently switching methods would hide that, so report it instead.
  if (n < 2 * nev + 1) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::verifySpectra: S has fewer than 3 rows, "
        "which is too small for Lanczos. Use VerificationMethod::DenseEigen.");
  }
  const int ncv = std::clamp(static_cast<int>(params.numLanczosVectors),
                             2 * nev + 1, n);
  const int maxIters = static_cast<int>(params.maxSpectraIters);

  // Pass 1: coarse estimate of the largest-magnitude eigenvalue.
  ShiftedMatrixProduct unshifted(S, 0.0);
  Spectra::SymEigsSolver<ShiftedMatrixProduct> largest(unshifted, nev, ncv);
  largest.init();
  largest.compute(Spectra::SortRule::LargestMagn, maxIters, 1e-4,
                  Spectra::SortRule::LargestMagn);
  if (largest.info() != Spectra::CompInfo::Successful) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::verifySpectra: Spectra Lanczos "
        "failed on the first pass (largest-magnitude eigenvalue).");
  }
  const double lambdaMaxMagnitude = largest.eigenvalues()(0);

  // If the largest-magnitude eigenvalue is negative it is also lambda_min, and
  // its eigenvector is the descent direction, so no second pass is needed.
  if (lambdaMaxMagnitude < 0.0) {
    Vector vMin = largest.eigenvectors().col(0);
    vMin.normalize();
    if (params.verbose) {
      std::cout << "  [Spectra] lambda_min=" << lambdaMaxMagnitude
                << " (negative extremal, one pass) dim=" << n << std::endl;
    }
    return {false, lambdaMaxMagnitude, vMin};
  }

  // Pass 2: shifting by -2*lambda_max puts the spectrum in
  // [lambda_min - 2*lambda_max, -lambda_max], so the largest-magnitude
  // eigenvalue of S - 2*lambda_max*I is lambda_min - 2*lambda_max.
  ShiftedMatrixProduct shifted(S, -2.0 * lambdaMaxMagnitude);

  // Spectra's tolerance is relative to the eigenvalue being computed, whose
  // magnitude here is about 2*lambda_max.
  constexpr double kMinRelativeTol = 1e-8;
  double relativeTol =
      std::max(params.spectraTol / lambdaMaxMagnitude, kMinRelativeTol);

  Vector start;
  if (warmStart.size() == n) {
    Spectra::SimpleRandom<double> rng(0);
    Vector perturbation = rng.random_vec(n);
    perturbation.normalize();
    start = warmStart + (0.03 * warmStart.norm()) * perturbation;
  }

  constexpr int kMaxAttempts = 3;
  int attemptNcv = ncv;
  for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
    Spectra::SymEigsSolver<ShiftedMatrixProduct> smallest(shifted, nev,
                                                          attemptNcv);
    if (start.size() == n) {
      smallest.init(start.data());
    } else {
      smallest.init();
    }
    smallest.compute(Spectra::SortRule::LargestMagn, maxIters, relativeTol,
                     Spectra::SortRule::LargestMagn);

    if (smallest.info() == Spectra::CompInfo::Successful) {
      const double lambdaMin =
          smallest.eigenvalues()(0) + 2.0 * lambdaMaxMagnitude;
      Vector vMin = smallest.eigenvectors().col(0);
      vMin.normalize();
      if (params.verbose) {
        std::cout << "  [Spectra] lambda_min=" << lambdaMin
                  << " lambda_max=" << lambdaMaxMagnitude << " dim=" << n
                  << " iters=" << smallest.num_iterations();
        if (attempt > 0) {
          std::cout << " (retry " << attempt << ", tol=" << relativeTol
                    << ", ncv=" << attemptNcv << ")";
        }
        std::cout << std::endl;
      }
      return {false, lambdaMin, vMin};
    }

    relativeTol *= 100.0;
    attemptNcv = std::min(n, 2 * attemptNcv);
  }

  throw std::runtime_error(
      "RiemannianStaircaseOptimizer::verifySpectra: Spectra Lanczos failed "
      "on the second pass (shifted) at every tolerance tried.");
}

/* ************************************************************************* */
RiemannianStaircaseResult RiemannianStaircaseOptimizer::optimize() const {
  const auto totalStart = Clock::now();

  RiemannianStaircaseResult result;
  Values Y = padInitialValues(initialValues_, params_.pMin);

  // Algorithm 1, lines 2-13. See header for the full chain; per iteration:
  // local solve -> build S -> verify -> return on pass, else lift to p+1.
  // Carried from one level's saddle escape to the next level's solve, so
  // the problem the merit needed is not rebuilt.
  std::shared_ptr<const QcqpProblem> carriedQcqp;
  double carriedQcqpBuildTime = 0.0;

  for (size_t p = params_.pMin; p <= params_.pMax; ++p) {
    result.ranksVisited.push_back(p);
    if (params_.verbose) {
      std::cout << "Staircase at rank = " << p << std::endl;
    }

    std::shared_ptr<const QcqpProblem> qcqp;
    if (p == params_.pMin) {
      qcqp = pMinQcqp_;
      result.qcqpBuildTimePerLevel.push_back(pMinQcqpBuildTime_);
    } else if (carriedQcqp) {
      // The escape below already built this level's problem for its merit.
      qcqp = carriedQcqp;
      result.qcqpBuildTimePerLevel.push_back(carriedQcqpBuildTime);
      carriedQcqp.reset();
    } else {
      const auto buildStart = Clock::now();
      qcqp = std::make_shared<QcqpProblem>(graph_, p);
      result.qcqpBuildTimePerLevel.push_back(ElapsedSeconds(buildStart));
    }

    // Inner solve (Alg. 1 line 3). Swap point for non-ALM solvers — downstream
    // needs (Y*, lambdaEq) aligned with qcqp.eConstraints().
    const auto nlpStart = Clock::now();
    const InnerSolveResult inner = runLocalSolver(*qcqp, Y, params_.almParams);
    result.nlpTimePerLevel.push_back(ElapsedSeconds(nlpStart));
    Y = inner.Y;

    result.costPerLevel.push_back(qcqp->costs().error(Y));
    result.stationarityPerLevel.push_back(inner.stationarity);

    // Certificate + verify (Alg. 1 lines 4-9). Pass => global SDP optimum.
    const auto verifyStart = Clock::now();
    const Layout layout = Layout::From(Y);
    // Solvers without native multipliers need a closed-form extractor here;
    // not implemented yet, so insist on non-empty.
    if (inner.lambdaEq.empty()) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::optimize: local solver returned no "
          "multipliers and the closed-form extractor is not implemented.");
    }
    // An ALM run that stops before its first dual update returns lambda = 0.
    // Then S = Q + A*(0) = Q, which is positive semidefinite by construction
    // and would certify any point, so recover the multipliers from Y instead.
    std::vector<Vector> lambdaEq = inner.lambdaEq;
    double maxAbsLambda = 0.0;
    for (const auto& value : lambdaEq) {
      maxAbsLambda = std::max(maxAbsLambda, value.cwiseAbs().maxCoeff());
    }
    if (maxAbsLambda == 0.0) {
      const LeastSquaresMultipliers ls =
          leastSquaresMultipliers(*qcqp, layout, Y, dataMatrix_);
      lambdaEq = ls.lambdaEq;
      if (params_.verbose) {
        std::cout << "  [multipliers] ALM returned lambda = 0; using "
                     "least-squares multipliers (stationarity residual "
                  << ls.totalResidual << ")" << std::endl;
      }
    }

    const Eigen::SparseMatrix<double> S =
        buildCertificate(*qcqp, layout, lambdaEq, dataMatrix_);
    // S*Y = 0 at a critical point, so any column of Y is a null-space vector
    // and a good starting point for the Lanczos pass.
    auto [passed, lambdaMin, vMin] =
        verify(S, params_, layout.stack(Y).col(0));
    result.verifyTimePerLevel.push_back(ElapsedSeconds(verifyStart));
    result.minEigenvaluePerLevel.push_back(lambdaMin);

    if (params_.verbose) {
      // lambda_min only printed on failure (Cholesky gives just the bound).
      std::cout << "Staircase verified=" << (passed ? 1 : 0)
                << " cost=" << result.costPerLevel.back();
      if (!passed) std::cout << " lambda_min=" << lambdaMin;
      std::cout << std::endl;
    }

    if (passed) {
      result.values = Y;
      result.layout = layout;
      result.finalRank = p;
      result.certified = true;
      result.minEigenvalue = lambdaMin;
      result.rounded =
          truncateToRankD(Y, layout, static_cast<int>(layout.maxRowDim()));
      result.totalTime = pMinQcqpBuildTime_ + ElapsedSeconds(totalStart);
      return result;
    }

    // Alg. 1 lines 10-12: S not PSD, lift to rank p+1 along v_min. The
    // O(alpha^2) violation is absorbed by the next inner solve.
    if (p < params_.pMax) {
      const auto buildStart = Clock::now();
      carriedQcqp = std::make_shared<QcqpProblem>(graph_, p + 1);
      carriedQcqpBuildTime = ElapsedSeconds(buildStart);
      Y = saddleEscapeWithLineSearch(*carriedQcqp, layout, Y, vMin, lambdaEq,
                                     inner.penalty, lambdaMin, params_)
              .lifted;
    } else {
      result.minEigenvalue = lambdaMin;
    }
  }

  // pMax exhausted without certification: return best-effort.
  result.values = Y;
  result.layout = Layout::From(Y);
  result.finalRank = params_.pMax;
  result.certified = false;
  result.totalTime = pMinQcqpBuildTime_ + ElapsedSeconds(totalStart);
  return result;
}

/* ************************************************************************* */
Values RiemannianStaircaseResult::roundedValues() const {
  if (!rounded) {
    throw std::runtime_error(
        "RiemannianStaircaseResult::roundedValues: no rounded solution is "
        "available.");
  }
  return layout.unstack(rounded->Yd);
}

namespace {

template <typename T>
Vector ToVector(const std::vector<T>& values) {
  Vector result(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    result(static_cast<Eigen::Index>(i)) = static_cast<double>(values[i]);
  }
  return result;
}

}  // namespace

Vector RiemannianStaircaseResult::getRanksVisited() const {
  return ToVector(ranksVisited);
}

Vector RiemannianStaircaseResult::getCostPerLevel() const {
  return ToVector(costPerLevel);
}

Vector RiemannianStaircaseResult::getMinEigenvaluePerLevel() const {
  return ToVector(minEigenvaluePerLevel);
}

Vector RiemannianStaircaseResult::getStationarityPerLevel() const {
  return ToVector(stationarityPerLevel);
}

Vector RiemannianStaircaseResult::getQcqpBuildTimePerLevel() const {
  return ToVector(qcqpBuildTimePerLevel);
}

Vector RiemannianStaircaseResult::getNlpTimePerLevel() const {
  return ToVector(nlpTimePerLevel);
}

Vector RiemannianStaircaseResult::getVerifyTimePerLevel() const {
  return ToVector(verifyTimePerLevel);
}

}  // namespace gtsam
