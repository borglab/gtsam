/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.cpp
 * @brief   Riemannian Staircase outer loop wrapping ALM as the inner solver.
 */

// =============================================================================
// Dimensional and ordering invariants (keep this section close at hand).
//
// For a problem with N variables, each variable's ambient row dimension r_n,
// and current BM column dimension p (== K == column count of every variable
// value; r_n is the per-type intrinsic dim, e.g. 2 for Rot2, 3 for Rot3):
//
//   Y_i  : r_n x p       single variable's matrix value
//   Y    : totalDim x p  global value, stacked in canonical layout order
//   S    : totalDim x totalDim  sparse symmetric, INDEPENDENT of p
//   Q_ij : r_i x r_j     cost block between vars i and j (also independent
//                        of p; the p-dependence lives in QpCost's internal
//                        HessianFactor, not in S)
//   A_m  : r_n x r_n     constraint matrix for the m-th equality
//   lam  : scalar (boxed as Vector1)  ALM multiplier for m-th constraint
//
// QpCost stores an internal HessianFactor whose (i,j) block is
// r_i*p x r_j*p (the "ExpandedBlock" expansion: p block-diagonal copies of
// Q_ij; this is the Kronecker product I_p ⊗ Q_ij). The underlying Q_ij is
// the top-left r_i x r_j corner of any one copy. We embed Q_ij ONCE into S
// at the global offsets (offset[keys[i]], offset[keys[j]]); the
// ExpandedBlock structure lives only inside the HessianFactor.
//
// Ordering invariants — all enforced through the single canonical Layout
// returned by Layout::From(values):
//   1. Layout iteration order is canonical (std::map => sorted by Key).
//   2. lambdaEq[m] corresponds to qcqp.eConstraints()[m].
//   3. Multiple constraints share a key; we sum lam_m * (A_m + A_m^T) onto
//      the key's diagonal block (analytic Lagrangian Hessian).
//   4. qp->keys()[i] indexes into THAT factor; translate to global offset
//      via layout.offsetOf(qp->keys()[i]).
//   5. Per-key row dim (r_n) is sourced from the Layout. Cost / constraint
//      matrices are validated against it.
// =============================================================================

#include <gtsam/constrained/QpCost.h>
#include <gtsam/constrained/QuadraticConstraint.h>
#include <gtsam/constrained/RiemannianStaircaseOptimizer.h>
#include <gtsam/constrained/internal/FastVerification.h>
#include <gtsam/linear/HessianFactor.h>

#include <Eigen/Eigenvalues>

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
  // Discover the common column count from the first value.
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
  // The Layout is the single source of truth for per-key row dim and totalDim;
  // we validate equality-constraint matrix shapes against it as a sanity check
  // rather than rebuilding the row-dim map from the constraints.
  const size_t totalDim = layout.totalDim;

  std::vector<Eigen::Triplet<double>> triplets;

  // 1. Cost contributions.
  //    For each QpCost: its internal HessianFactor's (i, j) block is
  //    r_i*p x r_j*p with p block-diagonal copies of the underlying Q_ij.
  //    We recover Q_ij as the top-left r_i x r_j corner and embed it into
  //    S at (offset[keys[i]], offset[keys[j]]). Symmetric: also embed the
  //    transpose at (offset[keys[j]], offset[keys[i]]) for i != j.
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

  // 2. lambda * (A + A^T) contributions, block-diagonal by key.
  //
  // For h_m(X) = trace(X' A_m X) - b_m, the gradient is grad_X h_m = (A_m +
  // A_m^T) X. Lagrangian stationarity is grad f + Sum_m lambda_m grad h_m =
  // 0, so the certificate (Hessian of L wrt X at the optimum) is
  //   S = Q + Sum_m lambda_m (A_m + A_m^T).
  //
  // For symmetric A_m (the case for every type we currently support, e.g.
  // Rot2's QcqpConstraints) this simplifies to Q + 2 Sum_m lambda_m A_m.
  // Writing (A + A^T) makes the formula correct for non-symmetric A too and
  // keeps the certificate manifestly symmetric.
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
        // Contribution = lambda * (A(r, c2) + A(c2, r)). For symmetric A this
        // is 2 * lambda * A(r, c2).
        const double v = lam * (A(r, c2) + A(c2, r));
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
  // When passed, the Cholesky was successful and `v` was never written; emit
  // a zero vector to keep the contract that v has the right length.
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
  // Eigenvalues are in ascending order; column 0 is the minimum eigenvector.
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
RiemannianStaircaseResult RiemannianStaircaseOptimizer::optimize() const {
  using Clock = std::chrono::steady_clock;
  const auto seconds = [](Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
  };
  const auto totalStart = Clock::now();

  RiemannianStaircaseResult result;
  Values Y = initialValues_;

  // ALM must record per-iteration state so we can pull lambdaEq at the end.
  // shared_ptr-through-const is fine (the pointer is const, not the pointee).
  if (params_.almParams) {
    params_.almParams->storeOptProgress = true;
  }

  for (size_t p = params_.pMin; p <= params_.pMax; ++p) {
    result.ranksVisited.push_back(p);
    if (params_.verbose) {
      std::cout << "[Staircase] rank p = " << p << std::endl;
    }

    // 1. ALM phase: build a fresh QcqpProblem at column dim K = p.
    const auto almStart = Clock::now();
    QcqpProblem qcqp_p(graph_, p);
    AugmentedLagrangianOptimizer alm(qcqp_p, Y, params_.almParams);
    Y = alm.optimize();
    result.costPerLevel.push_back(qcqp_p.costs().error(Y));
    result.almTimePerLevel.push_back(seconds(almStart, Clock::now()));

    // 2. Verify phase: pull multipliers, assemble certificate, run verify().
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
        assembleCertificate(qcqp_p, layout, lambdaEq);
    auto [passed, lambdaMin, vMin] = verify(S);
    result.verifyTimePerLevel.push_back(seconds(verifyStart, Clock::now()));

    result.certifiedPerLevel.push_back(passed);
    result.minEigenvaluePerLevel.push_back(lambdaMin);
    if (params_.verbose) {
      std::cout << "[Staircase] passed=" << (passed ? 1 : 0)
                << " lambda_min=" << lambdaMin
                << " cost=" << result.costPerLevel.back() << std::endl;
    }

    if (passed) {
      // No lift at the certified level; record 0 to keep all per-level
      // vectors the same length.
      result.liftTimePerLevel.push_back(0.0);
      result.values = Y;
      result.finalRank = p;
      result.certified = true;
      result.minEigenvalue = lambdaMin;
      result.totalTime = seconds(totalStart, Clock::now());
      return result;
    }

    // 3. Lift phase: saddle escape if we haven't exhausted pMax.
    const auto liftStart = Clock::now();
    if (p < params_.pMax) {
      Y = escapeSaddleAndLift(Y, vMin, layout, params_.alpha);
    } else {
      // We were at pMax and failed; surface the last v_min/eig anyway.
      result.minEigenvalue = lambdaMin;
    }
    result.liftTimePerLevel.push_back(seconds(liftStart, Clock::now()));
  }

  // Hit pMax without certifying.
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
