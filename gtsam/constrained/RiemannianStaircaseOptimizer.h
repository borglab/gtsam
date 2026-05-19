/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.h
 * @brief   Riemannian Staircase outer loop wrapping ALM as the inner solver.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/Sparse>

#include <map>
#include <tuple>
#include <vector>

namespace gtsam {

/**
 * Parameters for the Riemannian Staircase outer loop.
 */
struct GTSAM_EXPORT RiemannianStaircaseParams {
  /**
   * Which eigenvalue solver to use for the certificate matrix.
   * - LOBPCG: fast_verification (Optimization + Preconditioners libraries
   *   + Cholmod). Production path; scales to tens of thousands of poses.
   *   Requires GTSAM_USE_LOBPCG_VERIFICATION at build time.
   * - DenseEigen: Eigen::SelfAdjointEigenSolver on Matrix(S). Allocates a
   *   dense totalDim x totalDim matrix, so memory blows up at scale. Useful
   *   for small problems and for cross-validating the LOBPCG path.
   */
  enum class VerificationMethod { LOBPCG, DenseEigen };

  /**
   * Starting and maximum column dimension K of the matrix-form QCQP variable.
   * pMin must be >= the intrinsic row dim of every variable type in the
   * graph (2 for Rot2, 3 for Rot3); pMax caps how far the staircase climbs
   * before giving up. The staircase always starts at K = pMin and grows K
   * by 1 per level via escapeSaddleAndLift.
   */
  size_t pMin = 2;            ///< starting K (typical: 2 for Rot2, 3 for Rot3).
  size_t pMax = 10;           ///< maximum K before giving up.
  double alpha = 1e-2;        ///< descent-direction step size for the lift.

  // --- Certificate verification ---
  VerificationMethod verificationMethod = VerificationMethod::LOBPCG;
  double eta = 1e-3;           ///< min-eigenvalue tolerance for certification.
  size_t nx = 1;               ///< block size for LOBPCG.
  size_t maxLOBPCGIters = 1000;  ///< max LOBPCG iterations.
  double maxFillFactor = 3.0;  ///< ILDL incomplete-Cholesky fill factor.
  double dropTol = 1e-3;       ///< ILDL incomplete-Cholesky drop tolerance.

  AugmentedLagrangianParams::shared_ptr almParams =
      std::make_shared<AugmentedLagrangianParams>();
  bool verbose = false;
};

/**
 * Result returned by RiemannianStaircaseOptimizer::optimize().
 */
struct GTSAM_EXPORT RiemannianStaircaseResult {
  Values values;                       ///< final BM solution (Matrix-valued).
  size_t finalRank = 0;                ///< BM rank of the returned values.
  bool certified = false;              ///< true if certificate passed.
  double minEigenvalue = 0.0;          ///< min eig of the final certificate.
  std::vector<size_t> ranksVisited;
  std::vector<double> costPerLevel;    ///< inner cost after each ALM solve.
  /// Per-level certification verdict (true iff that level's certificate
  /// passed; the staircase stops on the first true entry, so all earlier
  /// entries are false).
  std::vector<bool> certifiedPerLevel;
  /// Per-level minimum eigenvalue of the certificate matrix S (as reported
  /// by verify()). For LOBPCG, this is 0 when the Cholesky short-circuit
  /// succeeded (lower bound on lambda_min >= -eta); the dense path always
  /// returns the actual numeric min eigenvalue.
  std::vector<double> minEigenvaluePerLevel;
  /// Per-level wall-clock seconds spent in the inner ALM solve (includes
  /// the QcqpProblem rebuild at that rank).
  std::vector<double> almTimePerLevel;
  /// Per-level wall-clock seconds for certificate construction + verify()
  /// (Layout::From + assembleCertificate + verify dispatch).
  std::vector<double> verifyTimePerLevel;
  /// Per-level wall-clock seconds for the saddle-escape lift. The level
  /// that certifies records 0 here (no lift was performed).
  std::vector<double> liftTimePerLevel;
  /// Total wall-clock seconds for optimize() (sum of all phases above plus
  /// minor bookkeeping).
  double totalTime = 0.0;
};

/**
 * Riemannian Staircase outer loop with ALM as the inner solver.
 *
 *   - Layout::From() and assembleCertificate(): Key -> {offset, rowDim}
 *     mapping and the Lagrangian-Hessian certificate matrix
 *         S = Q + sum_m lambda_m * (A_m + A_m^T),
 *     assembled as a sparse symmetric matrix.
 *   - escapeSaddleAndLift(): lifts Y* at BM rank p to [Y* | alpha*v_min] at
 *     rank p+1; no retraction is needed because the new column is tangent
 *     to the constraint surface at first order.
 *   - optimize(): iterates p in [pMin, pMax], inner ALM solve, build
 *     certificate, verify, return on certification or lift+retry.
 *
 * Verification: two paths controlled by RiemannianStaircaseParams::
 * verificationMethod. The production path is LOBPCG via fast_verification
 * (compiled only when GTSAM_USE_LOBPCG_VERIFICATION is ON). A dense
 * Eigen::SelfAdjointEigenSolver fallback is always available for small
 * problems and as a cross-validation reference.
 */
class GTSAM_EXPORT RiemannianStaircaseOptimizer {
 public:
  /// Per-variable layout slice: row offset in the ambient ℝ^totalDim space
  /// and row dimension of the variable's matrix-valued QCQP value.
  struct LayoutSlice {
    size_t offset;
    size_t rowDim;
  };

  /**
   * Layout maps each Key to its {offset, rowDim} slice for the matrix-form
   * QCQP variable, in canonical sort-by-Key order, plus
   * totalDim = sum of row dims.
   *
   * Layout is the single source of truth for variable ordering and per-key
   * row dimension throughout the staircase pipeline: certificate assembly,
   * the saddle-escape lift, and rounding all consult the layout's slices.
   * Build it with Layout::From(values).
   */
  struct GTSAM_EXPORT Layout {
    std::map<Key, LayoutSlice> slices;
    size_t totalDim = 0;

    /// Build a Layout from a Values containing only Matrix-valued variables.
    /// Iteration order is canonical (std::map => sorted by Key), giving the
    /// same layout for the same Values across runs and across staircase
    /// levels (the lift grows columns, leaves row counts alone).
    static Layout From(const Values& values);

    /// True iff `key` is in the layout.
    bool contains(Key key) const { return slices.find(key) != slices.end(); }

    /// Number of variables tracked.
    size_t size() const { return slices.size(); }

    /// Row offset of `key` in the ambient ℝ^totalDim space. Throws if
    /// `key` is not in the layout.
    size_t offsetOf(Key key) const;

    /// Row dimension of `key`'s matrix-form variable. Throws if missing.
    size_t rowDimOf(Key key) const;

    /// {offset, rowDim} for `key`. Throws if missing.
    const LayoutSlice& sliceOf(Key key) const;

    /// True iff `values` has exactly the same Matrix-valued keys as this
    /// layout and each variable's row count matches the layout's rowDim.
    /// Useful as a fail-fast precondition check in consumers.
    bool conformsTo(const Values& values) const;

    /// Stack the Matrix-valued variables in `values` into a single dense
    /// (totalDim x p) matrix in canonical layout order. All values must
    /// share the same column count p; throws otherwise.
    Matrix stack(const Values& values) const;

    /// Inverse of stack: split a (totalDim x p) matrix into per-key
    /// (rowDim x p) Matrix-valued Values.
    Values unstack(const Matrix& Y) const;
  };

  RiemannianStaircaseOptimizer(const NonlinearFactorGraph& graph,
                               const Values& initialValues,
                               const RiemannianStaircaseParams& params = {})
      : graph_(graph), initialValues_(initialValues), params_(params) {
    validateParams(params_);
  }

  /// Run the staircase outer loop. See RiemannianStaircaseResult for outputs.
  RiemannianStaircaseResult optimize() const;

  /**
   * Assemble the certificate matrix as a sparse symmetric matrix:
   *
   *   S = Q + sum_m lambda_m * (A_m + A_m^T)
   *
   * which is the Hessian of the Lagrangian L = f + sum_m lambda_m h_m for
   * h_m(X) = trace(X' A_m X) - b_m. For symmetric A_m (every currently
   * supported case, e.g. Rot2 QcqpConstraints) this is Q + 2 * sum_m
   * lambda_m * A_m. Writing (A + A^T) keeps S manifestly symmetric and
   * works for non-symmetric A too.
   *
   * @param qcqp     the QcqpProblem at the current BM rank.
   * @param layout   single source of truth for per-key row offset and
   *                 row dim; built from the current Values via Layout::From.
   * @param lambdaEq multipliers from AugmentedLagrangianState::lambdaEq;
   *                 lambdaEq[m] corresponds to qcqp.eConstraints()[m].
   */
  static Eigen::SparseMatrix<double> assembleCertificate(
      const QcqpProblem& qcqp, const Layout& layout,
      const std::vector<Vector>& lambdaEq);

  /**
   * Lift Y* from BM rank p to rank p+1, injecting a descent direction in the
   * newly appended column. No retraction/projection is performed: the descent
   * direction [0 | v_min] is already tangent to the constraint surface at the
   * lifted point [Y* | 0] (the new column on Y* is zero), so a plain stack is
   * exact at first order in alpha. Constraint violation is O(alpha^2).
   *
   * @param Ystar  Values at rank p, each entry an r_n x p Matrix.
   * @param vMin   descent direction in ambient form, length = totalDim.
   * @param layout per-key {offset, rowDim} for Ystar; throws on mismatch.
   * @param alpha  step size for the descent direction (typical: 1e-2).
   * @return Values at rank p+1, each entry an r_n x (p+1) Matrix.
   */
  static Values escapeSaddleAndLift(const Values& Ystar, const Vector& vMin,
                                    const Layout& layout, double alpha);

  /**
   * Verify whether the certificate matrix S indicates a certifiably PSD point.
   *
   * Dispatches on params_.verificationMethod to either LOBPCG (production,
   * scalable; requires GTSAM_USE_LOBPCG_VERIFICATION at build time) or a
   * dense Eigen::SelfAdjointEigenSolver (fallback / cross-validation). The
   * min eigenvector is returned in v_min so it can be used as a saddle-escape
   * direction when verification fails.
   *
   * @return (passed, lambda_min, v_min). passed is true iff
   *         lambda_min >= -params_.eta.
   */
  std::tuple<bool, double, Vector> verify(
      const Eigen::SparseMatrix<double>& S) const;

  const NonlinearFactorGraph& graph_;
  Values initialValues_;
  RiemannianStaircaseParams params_;

 private:
  std::tuple<bool, double, Vector> verifyLOBPCG(
      const Eigen::SparseMatrix<double>& S) const;
  std::tuple<bool, double, Vector> verifyDenseEigen(
      const Eigen::SparseMatrix<double>& S) const;

  /// Sanity-check pMin/pMax. The per-type intrinsic-dim check (K >= d) is
  /// enforced inside each factor's qcqpFactors, so it's not duplicated here.
  static void validateParams(const RiemannianStaircaseParams& params);
};

}  // namespace gtsam
