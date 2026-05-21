/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.cpp
 * @brief   Riemannian Staircase outer loop wrapping ALM as the inner NLP
 *          solver (a more efficient Riemannian-manifold-based solver is
 *          planned for a future release to replace it).
 */

// Notation used throughout: N variables, each with row dim r_n; p = current
// BM column dim. Y_i is r_n × p; Y is totalDim × p (stacked in Layout order);
// S is totalDim × totalDim (p-independent); Q_ij is r_i × r_j; A_m is the
// m-th equality-constraint matrix; lambdaEq[m] pairs with qcqp.eConstraints()[m].
//
// QpCost's internal HessianFactor expands Q_ij to (r_i*p × r_j*p) via
// I_p ⊗ Q_ij; we recover Q_ij as the top-left r_i × r_j corner and embed it
// once into S at the global offsets.

#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/internal/FastVerification.h>
#include <gtsam/linear/HessianFactor.h>

#include <Eigen/Eigenvalues>

// gtsam/3rdparty/Spectra is on the include path as -isystem; same style as
// gtsam/sfm/ShonanAveraging.cpp.
#include <SymEigsSolver.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace gtsam {

/* ************************************************************************* */
void RiemannianStaircaseOptimizer::validateParams(
    const RiemannianStaircaseParams& params) {
  if (params.pMin == 0 || params.pMax == 0) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer: pMin and pMax must be >= 1.");
  }
  if (params.pMin > params.pMax) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer: pMin must be <= pMax.");
  }
}

/* ************************************************************************* */
RiemannianStaircaseOptimizer::Layout
RiemannianStaircaseOptimizer::Layout::From(const Values& values) {
  Layout layout;
  size_t off = 0;
  for (const auto& [key, mat] : values.extract<Matrix>()) {
    const size_t rowDim = static_cast<size_t>(mat.rows());
    layout.slices.emplace(key, LayoutSlice{off, rowDim});
    off += rowDim;
  }
  layout.totalDim = off;
  return layout;
}

size_t RiemannianStaircaseOptimizer::Layout::offsetOf(Key key) const {
  return sliceOf(key).offset;
}

size_t RiemannianStaircaseOptimizer::Layout::rowDimOf(Key key) const {
  return sliceOf(key).rowDim;
}

const RiemannianStaircaseOptimizer::LayoutSlice&
RiemannianStaircaseOptimizer::Layout::sliceOf(Key key) const {
  const auto it = slices.find(key);
  if (it == slices.end()) {
    throw std::out_of_range(
        "RiemannianStaircaseOptimizer::Layout: key not in layout.");
  }
  return it->second;
}

bool RiemannianStaircaseOptimizer::Layout::conformsTo(
    const Values& values) const {
  const auto valuesMap = values.extract<Matrix>();
  if (valuesMap.size() != slices.size()) return false;
  for (const auto& [key, mat] : valuesMap) {
    const auto it = slices.find(key);
    if (it == slices.end()) return false;
    if (static_cast<size_t>(mat.rows()) != it->second.rowDim) return false;
  }
  return true;
}

Matrix RiemannianStaircaseOptimizer::Layout::stack(const Values& values) const {
  const auto valuesMap = values.extract<Matrix>();
  if (valuesMap.size() != slices.size()) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::Layout::stack: values has a different "
        "key set than the layout.");
  }
  DenseIndex p = -1;
  for (const auto& [key, mat] : valuesMap) {
    if (p < 0) p = mat.cols();
    else if (mat.cols() != p) {
      throw std::invalid_argument(
          "RiemannianStaircaseOptimizer::Layout::stack: values have "
          "inconsistent column counts.");
    }
  }
  if (p < 0) p = 0;
  Matrix Y = Matrix::Zero(totalDim, p);
  for (const auto& [key, mat] : valuesMap) {
    const auto it = slices.find(key);
    if (it == slices.end()) {
      throw std::invalid_argument(
          "RiemannianStaircaseOptimizer::Layout::stack: values contains key "
          "not in layout.");
    }
    if (static_cast<size_t>(mat.rows()) != it->second.rowDim) {
      throw std::invalid_argument(
          "RiemannianStaircaseOptimizer::Layout::stack: row-dim mismatch "
          "between values and layout.");
    }
    Y.block(it->second.offset, 0, it->second.rowDim, p) = mat;
  }
  return Y;
}

Values RiemannianStaircaseOptimizer::Layout::unstack(const Matrix& Y) const {
  if (static_cast<size_t>(Y.rows()) != totalDim) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::Layout::unstack: row count of Y does "
        "not match layout's totalDim.");
  }
  Values v;
  for (const auto& [key, slice] : slices) {
    Matrix block = Y.block(slice.offset, 0, slice.rowDim, Y.cols());
    v.insert(key, block);
  }
  return v;
}

/* ************************************************************************* */
Eigen::SparseMatrix<double> RiemannianStaircaseOptimizer::assembleCertificate(
    const QcqpProblem& qcqp, const Layout& layout,
    const std::vector<Vector>& lambdaEq) {
  const size_t totalDim = layout.totalDim;

  std::vector<Eigen::Triplet<double>> triplets;

  // Cost: recover Q_ij as the top-left r_i × r_j corner of QpCost's
  // I_p ⊗ Q_ij expansion and embed it once into S.
  for (const auto& factor : qcqp.costs()) {
    const auto qp = std::dynamic_pointer_cast<QpCost>(factor);
    if (!qp) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::assembleCertificate: non-QpCost in "
          "qcqp.costs()");
    }
    const HessianFactor& H = qp->hessianFactor();
    const auto& factorKeys = qp->keys();
    const size_t nVars = factorKeys.size();
    for (size_t i = 0; i < nVars; ++i) {
      const Key keyI = factorKeys[i];
      if (!layout.contains(keyI)) {
        throw std::runtime_error(
            "RiemannianStaircaseOptimizer::assembleCertificate: cost factor "
            "references a key not in the layout.");
      }
      const auto& sliceI = layout.sliceOf(keyI);
      for (size_t j = i; j < nVars; ++j) {
        const Key keyJ = factorKeys[j];
        if (!layout.contains(keyJ)) {
          throw std::runtime_error(
              "RiemannianStaircaseOptimizer::assembleCertificate: cost factor "
              "references a key not in the layout.");
        }
        const auto& sliceJ = layout.sliceOf(keyJ);

        const Matrix expanded = H.info().block(i, j);
        const Matrix Q_ij =
            expanded.topLeftCorner(sliceI.rowDim, sliceJ.rowDim);

        for (size_t r = 0; r < sliceI.rowDim; ++r) {
          for (size_t c2 = 0; c2 < sliceJ.rowDim; ++c2) {
            const double v = Q_ij(r, c2);
            if (v == 0.0) continue;
            triplets.emplace_back(static_cast<int>(sliceI.offset + r),
                                  static_cast<int>(sliceJ.offset + c2), v);
            if (i != j) {
              triplets.emplace_back(static_cast<int>(sliceJ.offset + c2),
                                    static_cast<int>(sliceI.offset + r), v);
            }
          }
        }
      }
    }
  }

  // Constraint: A_m is the Hessian of h_m under the 0.5-trace convention,
  // so the Lagrangian Hessian is simply S = Q + Σ_m λ_m A_m.
  const auto& eqs = qcqp.eConstraints();
  if (lambdaEq.size() != eqs.size()) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::assembleCertificate: lambdaEq size "
        "does not match eConstraints size.");
  }
  for (size_t m = 0; m < eqs.size(); ++m) {
    const auto eqf =
        std::dynamic_pointer_cast<QuadraticEqualityConstraintFactor>(eqs[m]);
    if (!eqf) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::assembleCertificate: "
          "non-QuadraticEqualityConstraintFactor in qcqp.eConstraints()");
    }
    const auto& c = eqf->quadraticConstraint();
    const Vector& lamBox = lambdaEq[m];
    if (lamBox.size() == 0) continue;
    const double lam = lamBox(0);
    if (lam == 0.0) continue;
    const Matrix& A = c.A();
    if (!layout.contains(c.key())) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::assembleCertificate: equality "
          "constraint references a key not in the layout.");
    }
    const auto& slice = layout.sliceOf(c.key());
    if (static_cast<size_t>(A.rows()) != slice.rowDim ||
        static_cast<size_t>(A.cols()) != slice.rowDim) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::assembleCertificate: equality "
          "constraint A has dimension that disagrees with the layout's "
          "rowDim for that key.");
    }
    const size_t base = slice.offset;
    for (int r = 0; r < A.rows(); ++r) {
      for (int c2 = 0; c2 < A.cols(); ++c2) {
        const double v = lam * A(r, c2);
        if (v == 0.0) continue;
        triplets.emplace_back(static_cast<int>(base + r),
                              static_cast<int>(base + c2), v);
      }
    }
  }

  Eigen::SparseMatrix<double> S(static_cast<int>(totalDim),
                                static_cast<int>(totalDim));
  S.setFromTriplets(triplets.begin(), triplets.end(),
                    [](const double& a, const double& b) { return a + b; });
  return S;
}

/* ************************************************************************* */
std::tuple<bool, double, Vector> RiemannianStaircaseOptimizer::verify(
    const Eigen::SparseMatrix<double>& S) const {
  switch (params_.verificationMethod) {
    case RiemannianStaircaseParams::VerificationMethod::LOBPCG:
      return verifyLOBPCG(S);
    case RiemannianStaircaseParams::VerificationMethod::DenseEigen:
      return verifyDenseEigen(S);
    case RiemannianStaircaseParams::VerificationMethod::Spectra:
      return verifySpectra(S);
  }
  throw std::runtime_error(
      "RiemannianStaircaseOptimizer::verify: unknown VerificationMethod.");
}

/* ************************************************************************* */
std::tuple<bool, double, Vector>
RiemannianStaircaseOptimizer::verifyLOBPCG(
    const Eigen::SparseMatrix<double>& S) const {
#ifdef GTSAM_USE_LOBPCG_VERIFICATION
  double theta = 0.0;
  Vector v;
  std::size_t numIters = 0;
  const bool passed = internal::fast_verification(
      S, params_.eta, params_.nx, theta, v, numIters, params_.maxLOBPCGIters,
      params_.maxFillFactor, params_.dropTol);
  if (params_.verbose) {
    std::cout << "  [fast_verification] passed=" << (passed ? 1 : 0)
              << " theta=" << theta << " iters=" << numIters << std::endl;
  }
  // On Cholesky success, v is never written — pad with zeros so callers can
  // assume v.size() == S.rows() unconditionally.
  if (passed && v.size() != S.rows()) {
    v = Vector::Zero(S.rows());
  }
  return {passed, theta, v};
#else
  throw std::runtime_error(
      "RiemannianStaircaseOptimizer::verifyLOBPCG: LOBPCG verification was "
      "selected but GTSAM was built with GTSAM_USE_LOBPCG_VERIFICATION=OFF. "
      "Either rebuild with the option enabled or switch the params to "
      "VerificationMethod::DenseEigen.");
#endif
}

/* ************************************************************************* */
std::tuple<bool, double, Vector>
RiemannianStaircaseOptimizer::verifyDenseEigen(
    const Eigen::SparseMatrix<double>& S) const {
  // Materializes S as a dense matrix. Caller is responsible for choosing this
  // path only when totalDim is small enough (a few thousand rows or so).
  const Matrix Sdense(S);
  Eigen::SelfAdjointEigenSolver<Matrix> es(Sdense);
  if (es.info() != Eigen::Success) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::verifyDenseEigen: "
        "SelfAdjointEigenSolver failed.");
  }
  const double lambdaMin = es.eigenvalues()(0);
  const Vector vMin = es.eigenvectors().col(0);
  const bool passed = (lambdaMin >= -params_.eta);
  if (params_.verbose) {
    std::cout << "  [dense Eigen] passed=" << (passed ? 1 : 0)
              << " lambda_min=" << lambdaMin << " dim=" << S.rows()
              << std::endl;
  }
  return {passed, lambdaMin, vMin};
}

/* ************************************************************************* */
namespace {

/// Spectra-compatible matvec for (S + sigma*I).
struct SpectraShiftedOp {
  using Scalar = double;
  const Eigen::SparseMatrix<double>& S_;
  double sigma_;
  SpectraShiftedOp(const Eigen::SparseMatrix<double>& S, double sigma)
      : S_(S), sigma_(sigma) {}
  int rows() const { return static_cast<int>(S_.rows()); }
  int cols() const { return static_cast<int>(S_.cols()); }
  void perform_op(const double* x, double* y) const {
    Eigen::Map<const Vector> X(x, rows());
    Eigen::Map<Vector> Y(y, rows());
    Y = S_ * X + sigma_ * X;
  }
};

}  // namespace

std::tuple<bool, double, Vector>
RiemannianStaircaseOptimizer::verifySpectra(
    const Eigen::SparseMatrix<double>& S) const {
  // Two-pass Lanczos (mirrors ShonanAveraging::SparseMinimumEigenValue):
  // find lambda_max, then shift by -2*lambda_max so the LargestMagn pass on
  // the shifted matrix returns lambda_min - 2*lambda_max.
  const Eigen::Index n = S.rows();
  const Eigen::Index ncv = std::min<Eigen::Index>(
      static_cast<Eigen::Index>(params_.numLanczosVectors), n);

  SpectraShiftedOp lmOp(S, /*sigma=*/0.0);
  Spectra::SymEigsSolver<SpectraShiftedOp> lmSolver(lmOp, /*nev=*/1, ncv);
  lmSolver.init();
  const int lmConverged =
      lmSolver.compute(Spectra::SortRule::LargestMagn, params_.maxSpectraIters,
                       params_.spectraTol, Spectra::SortRule::LargestMagn);
  if (lmConverged != 1) {
    throw std::runtime_error(
        "RiemannianStaircaseOptimizer::verifySpectra: failed to compute "
        "largest-magnitude eigenvalue.");
  }
  const double lmEigenValue = lmSolver.eigenvalues()(0);

  double lambdaMin;
  Vector vMin;
  if (lmEigenValue < 0) {
    // lambda_max < 0 implies it is also lambda_min.
    lambdaMin = lmEigenValue;
    vMin = lmSolver.eigenvectors(1).col(0);
  } else {
    SpectraShiftedOp minOp(S, /*sigma=*/-2.0 * lmEigenValue);
    Spectra::SymEigsSolver<SpectraShiftedOp> minSolver(minOp, /*nev=*/1, ncv);
    minSolver.init();
    const int minConverged = minSolver.compute(
        Spectra::SortRule::LargestMagn, params_.maxSpectraIters,
        params_.spectraTol / lmEigenValue, Spectra::SortRule::LargestMagn);
    if (minConverged != 1) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::verifySpectra: failed to compute "
          "minimum eigenvalue after spectral shift.");
    }
    lambdaMin = minSolver.eigenvalues()(0) + 2.0 * lmEigenValue;
    vMin = minSolver.eigenvectors(1).col(0);
  }
  vMin.normalize();

  const bool passed = (lambdaMin >= -params_.eta);
  if (params_.verbose) {
    std::cout << "  [Spectra] passed=" << (passed ? 1 : 0)
              << " lambda_min=" << lambdaMin << " lambda_max=" << lmEigenValue
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
  Values Y = initialValues_;

  // ALM must record per-iteration state so we can pull lambdaEq at the end.
  if (params_.almParams) {
    params_.almParams->storeOptProgress = true;
  }

  for (size_t p = params_.pMin; p <= params_.pMax; ++p) {
    result.ranksVisited.push_back(p);
    if (params_.verbose) {
      std::cout << "[Staircase] rank p = " << p << std::endl;
    }

    const auto nlpStart = Clock::now();
    qcqp_.rebuildAt(p);
    AugmentedLagrangianOptimizer alm(qcqp_, Y, params_.almParams);
    Y = alm.optimize();
    result.costPerLevel.push_back(qcqp_.costs().error(Y));
    result.nlpTimePerLevel.push_back(seconds(nlpStart, Clock::now()));

    const auto verifyStart = Clock::now();
    if (alm.progress().empty()) {
      throw std::runtime_error(
          "RiemannianStaircaseOptimizer::optimize: ALM produced no progress; "
          "did you set storeOptProgress on the params?");
    }
    const auto& finalState = alm.progress().back();
    const auto& lambdaEq = finalState.lambdaEq;

    const Layout layout = Layout::From(Y);
    const Eigen::SparseMatrix<double> S =
        assembleCertificate(qcqp_, layout, lambdaEq);
    auto [passed, lambdaMin, vMin] = verify(S);
    result.verifyTimePerLevel.push_back(seconds(verifyStart, Clock::now()));

    result.minEigenvaluePerLevel.push_back(lambdaMin);
    if (params_.verbose) {
      std::cout << "[Staircase] passed=" << (passed ? 1 : 0)
                << " lambda_min=" << lambdaMin
                << " cost=" << result.costPerLevel.back() << std::endl;
    }

    if (passed) {
      result.values = Y;
      result.finalRank = p;
      result.certified = true;
      result.minEigenvalue = lambdaMin;
      result.totalTime = seconds(totalStart, Clock::now());
      return result;
    }

    if (p < params_.pMax) {
      Y = escapeSaddleAndLift(Y, vMin, layout, params_.alpha);
    } else {
      result.minEigenvalue = lambdaMin;
    }
  }

  result.values = Y;
  result.finalRank = params_.pMax;
  result.certified = false;
  result.totalTime = seconds(totalStart, Clock::now());
  return result;
}

/* ************************************************************************* */
Values RiemannianStaircaseOptimizer::escapeSaddleAndLift(const Values& Ystar,
                                                        const Vector& vMin,
                                                        const Layout& layout,
                                                        double alpha) {
  if (!layout.conformsTo(Ystar)) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::escapeSaddleAndLift: layout does not "
        "match Ystar (different key set or row dims).");
  }
  if (static_cast<size_t>(vMin.size()) < layout.totalDim) {
    throw std::invalid_argument(
        "RiemannianStaircaseOptimizer::escapeSaddleAndLift: vMin is shorter "
        "than the layout's totalDim.");
  }
  Values Ynext;
  for (const auto& [key, Yi] : Ystar.extract<Matrix>()) {
    const auto& slice = layout.sliceOf(key);
    const size_t base = slice.offset;
    const size_t rn = slice.rowDim;
    const size_t p = static_cast<size_t>(Yi.cols());
    if (base + rn > static_cast<size_t>(vMin.size())) {
      throw std::invalid_argument(
          "RiemannianStaircaseOptimizer::escapeSaddleAndLift: vMin is shorter "
          "than the layout requires.");
    }
    Matrix YiNew(rn, p + 1);
    YiNew.leftCols(p) = Yi;
    YiNew.col(p) = alpha * vMin.segment(base, rn);
    Ynext.insert(key, YiNew);
  }
  return Ynext;
}

}  // namespace gtsam
