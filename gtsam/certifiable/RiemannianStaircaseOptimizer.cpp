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

#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/linear/HessianFactor.h>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/SparseCholesky>

#include <Spectra/MatOp/SparseSymMatProd.h>
#include <Spectra/SymEigsSolver.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace gtsam {

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
  return {Y, alm.progress().back().lambdaEq};
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
  //
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
Eigen::SparseMatrix<double> RiemannianStaircaseOptimizer::buildCertificate(
    const QcqpProblem& qcqp, const Layout& layout,
    const std::vector<Vector>& lambdaEq) {
  Eigen::SparseMatrix<double> S = buildDataMatrix(qcqp, layout);
  S += buildMultiplierMatrix(qcqp, layout, lambdaEq);
  S.makeCompressed();
  return S;
}

/* ************************************************************************* */
Values RiemannianStaircaseOptimizer::escapeSaddleAndLift(
    const Values& Ystar, const Vector& vMin, const Layout& layout,
    double alpha) {
  if (!layout.conformsTo(Ystar)) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::escapeSaddleAndLift: layout does not "
        "match values.");
  }
  if (static_cast<size_t>(vMin.size()) != layout.totalDim) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::escapeSaddleAndLift: vMin size does not "
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
    const RiemannianStaircaseParams& params) {
  switch (params.verificationMethod) {
    case RiemannianStaircaseParams::VerificationMethod::DenseEigen:
      return verifyDenseEigen(S, params);
    case RiemannianStaircaseParams::VerificationMethod::Spectra:
    default:
      return verifySpectra(S, params);
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
std::tuple<bool, double, Vector> RiemannianStaircaseOptimizer::verifySpectra(
    const Eigen::SparseMatrix<double>& S,
    const RiemannianStaircaseParams& params) {
  const int n = static_cast<int>(S.rows());

  // Stage 1: Cholesky on M = S + eta*I. Success => lambda_min(S) >= -eta;
  // return -eta as a conservative bound (v_min unused on the pass path).
  {
    Eigen::SparseMatrix<double> M = S;
    for (int k = 0; k < n; ++k) M.coeffRef(k, k) += params.eta;
    M.makeCompressed();
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> chol;
    chol.compute(M);
    if (chol.info() == Eigen::Success) {
      if (params.verbose) {
        std::cout << "  [Chol] verified=1 dim=" << n << std::endl;
      }
      return {true, -params.eta, Vector::Zero(n)};
    }
  }

  // Stage 2: two-pass Lanczos to recover precise (lambda_min, v_min).
  const int nev = 1;
  // Spectra needs 2*nev+1 <= ncv <= n; tiny n falls back to dense.
  if (n < 2 * nev + 1) {
    return verifyDenseEigen(S, params);
  }
  const int ncv = std::clamp(static_cast<int>(params.numLanczosVectors),
                             2 * nev + 1, n);
  const int maxIters = static_cast<int>(params.maxSpectraIters);
  const double tol = params.spectraTol;

  // Pass 1: lambda_max of S.
  Spectra::SparseSymMatProd<double> op(S);
  Spectra::SymEigsSolver<Spectra::SparseSymMatProd<double>> eigs(op, nev, ncv);
  eigs.init();
  eigs.compute(Spectra::SortRule::LargestAlge, maxIters, tol);
  if (eigs.info() != Spectra::CompInfo::Successful) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::verifySpectra: Spectra Lanczos failed "
        "on the first pass (lambda_max).");
  }
  const double lambdaMax = eigs.eigenvalues()(0);

  // Pass 2 (shift trick): largest eigenvalue of (lambdaMax * I - S) equals
  // lambdaMax - lambda_min, and Lanczos converges fastest at the extremal end.
  Eigen::SparseMatrix<double> shifted = -S;
  for (int k = 0; k < n; ++k) shifted.coeffRef(k, k) += lambdaMax;
  shifted.makeCompressed();
  Spectra::SparseSymMatProd<double> opShift(shifted);
  Spectra::SymEigsSolver<Spectra::SparseSymMatProd<double>> eigsShift(
      opShift, nev, ncv);
  eigsShift.init();
  eigsShift.compute(Spectra::SortRule::LargestAlge, maxIters, tol);
  if (eigsShift.info() != Spectra::CompInfo::Successful) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::verifySpectra: Spectra Lanczos failed "
        "on the second pass (shifted).");
  }
  const double shiftedMaxEig = eigsShift.eigenvalues()(0);
  const double lambdaMin = lambdaMax - shiftedMaxEig;
  Vector vMin = eigsShift.eigenvectors().col(0);
  vMin.normalize();

  const bool passed = (lambdaMin >= -params.eta);
  if (params.verbose) {
    std::cout << "  [Spectra] verified=" << (passed ? 1 : 0)
              << " lambda_min=" << lambdaMin << " lambda_max=" << lambdaMax
              << " dim=" << n << std::endl;
  }
  return {passed, lambdaMin, vMin};
}

/* ************************************************************************* */
RiemannianStaircaseResult RiemannianStaircaseOptimizer::optimize() const {
  using Clock = std::chrono::steady_clock;
  const auto seconds = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
  };
  const auto totalStart = Clock::now();

  RiemannianStaircaseResult result;
  Values Y = padInitialValues(initialValues_, params_.pMin);

  // Algorithm 1, lines 2-13. See header for the full chain; per iteration:
  // local solve -> build S -> verify -> return on pass, else lift to p+1.
  for (size_t p = params_.pMin; p <= params_.pMax; ++p) {
    result.ranksVisited.push_back(p);
    if (params_.verbose) {
      std::cout << "Staircase at rank = " << p << std::endl;
    }

    // Inner solve (Alg. 1 line 3). Swap point for non-ALM solvers — downstream
    // needs (Y*, lambdaEq) aligned with qcqp.eConstraints().
    const auto nlpStart = Clock::now();
    QcqpProblem qcqp(graph_, p);
    const InnerSolveResult inner =
        runLocalSolver(qcqp, Y, params_.almParams);
    Y = inner.Y;

    result.costPerLevel.push_back(qcqp.costs().error(Y));
    result.nlpTimePerLevel.push_back(seconds(nlpStart, Clock::now()));

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
    const Eigen::SparseMatrix<double> S =
        buildCertificate(qcqp, layout, inner.lambdaEq);
    auto [passed, lambdaMin, vMin] = verify(S, params_);
    result.verifyTimePerLevel.push_back(seconds(verifyStart, Clock::now()));
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
      result.totalTime = seconds(totalStart, Clock::now());
      return result;
    }

    // Alg. 1 lines 10-12: S not PSD, lift to rank p+1 along v_min. The
    // O(alpha^2) violation is absorbed by the next inner solve.
    if (p < params_.pMax) {
      Y = escapeSaddleAndLift(Y, vMin, layout, params_.alpha);
    } else {
      result.minEigenvalue = lambdaMin;
    }
  }

  // pMax exhausted without certification: return best-effort.
  result.values = Y;
  result.layout = Layout::From(Y);
  result.finalRank = params_.pMax;
  result.certified = false;
  result.totalTime = seconds(totalStart, Clock::now());
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

Vector RiemannianStaircaseResult::getNlpTimePerLevel() const {
  return ToVector(nlpTimePerLevel);
}

Vector RiemannianStaircaseResult::getVerifyTimePerLevel() const {
  return ToVector(verifyTimePerLevel);
}

}  // namespace gtsam
