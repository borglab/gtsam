/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.h
 * @brief   Riemannian Staircase outer loop with ALM as the inner NLP solver
 *          and a sparse-eigensolver certificate of optimality. A specialized
 *          Riemannian-manifold solver is planned to replace ALM in a future
 *          release.
 * @author  Zhexin Xu     (xu.zhex@northeastern.edu)
 * @author  David M. Rosen
 *
 * References:
 *   Xu, Sanderson, Zhang, Rosen — "Certifiable Estimation with Factor
 *     Graphs," arXiv:2603.01267, 2026.
 *   Rosen — "Accelerating certifiable estimation with preconditioned
 *     eigensolvers," IEEE RA-L 7(4):12507–12514, 2022.
 *   Rosen — "Scalable low-rank semidefinite programming for certifiably
 *     correct machine perception," WAFR 2020.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <Eigen/SVD>
#include <Eigen/Sparse>

#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

/// Hyperparameters for the Riemannian Staircase outer loop.
struct GTSAM_EXPORT RiemannianStaircaseParams {
  /// LOBPCG: sparse, scalable, requires GTSAM_USE_LOBPCG_VERIFICATION.
  /// Spectra: sparse Lanczos via gtsam/3rdparty, no CHOLMOD dependency.
  /// DenseEigen: full O(n^3) diagonalization — debugging only, does not scale.
  enum class VerificationMethod { LOBPCG, DenseEigen, Spectra };

  size_t pMin = 2;            ///< starting K; must be >= max intrinsic row dim.
  size_t pMax = 10;           ///< staircase cap before giving up.
  double alpha = 1e-2;        ///< descent step size on saddle escape.

  VerificationMethod verificationMethod = VerificationMethod::LOBPCG;
  double eta = 1e-3;           ///< min-eigenvalue tolerance for certification.
  size_t nx = 1;               ///< LOBPCG block size.
  size_t maxLOBPCGIters = 1000;
  double maxFillFactor = 3.0;  ///< ILDL fill factor.
  double dropTol = 1e-3;       ///< ILDL drop tolerance.

  size_t maxSpectraIters = 1000;
  /// Lanczos subspace size. A generous value (~100) is needed so the
  /// shifted-Lanczos pass discriminates a tiny negative eigenvalue from a
  /// near-zero one — they map to nearly-identical magnitudes after the shift.
  size_t numLanczosVectors = 100;
  double spectraTol = 1e-4;

  AugmentedLagrangianParams::shared_ptr almParams =
      std::make_shared<AugmentedLagrangianParams>();
  bool verbose = false;
};

/// Output of RiemannianStaircaseOptimizer::optimize().
struct GTSAM_EXPORT RiemannianStaircaseResult {
  Values values;                       ///< final BM solution (Matrix-valued).
  /// SO(d)-projected rotations, populated by `optimize<RotT>()` only when
  /// `certified == true`. Empty otherwise.
  Values rounded;
  size_t finalRank = 0;
  bool certified = false;
  double minEigenvalue = 0.0;
  std::vector<size_t> ranksVisited;
  /// Per-level inner cost in paper convention (`||residual||^2 = 2 * graph.error`).
  std::vector<double> costPerLevel;
  std::vector<double> minEigenvaluePerLevel;
  /// Per-level wall-clock seconds for the inner NLP solve (ALM today).
  std::vector<double> nlpTimePerLevel;
  /// Per-level wall-clock seconds for certificate assembly + verify().
  std::vector<double> verifyTimePerLevel;
  double totalTime = 0.0;
};

/**
 * Riemannian Staircase outer loop over a Burer–Monteiro low-rank
 * parametrization of a QCQP. Takes a `QcqpProblem` and an initial value and
 * returns a certified (or best-effort) low-rank BM solution. Factor-graph →
 * QCQP conversion lives outside this class.
 *
 * Per level: (1) inner NLP solve at rank p yielding (Y*, λ); (2) certificate
 * `S = Q + Σ_m λ_m A_m`; (3) verify via LOBPCG / Spectra / DenseEigen; on
 * fail, (4) `escapeSaddleAndLift` appends a descent column at rank p+1.
 *
 * ALM is the inner solver today; a specialized Riemannian-manifold solver is
 * planned to replace it.
 */
class GTSAM_EXPORT RiemannianStaircaseOptimizer {
 public:
  /// Placement of one QCQP variable inside the stacked BM matrix Y:
  /// `Y_i = Y.block(offset, 0, rowDim, p)`.
  struct LayoutSlice {
    size_t offset;
    size_t rowDim;
  };

  /**
   * Maps each Key to its (offset, rowDim) slice in the stacked BM decision
   * variable Y ∈ ℝ^{totalDim × p}.
   *
   *                  p columns
   *             ┌────────────────┐
   *        x0:  │     Y_{x0}     │  rowDim_{x0} rows, offset_{x0} = 0
   *             ├────────────────┤
   *        x1:  │     Y_{x1}     │  rowDim_{x1} rows, offset_{x1} = rowDim_{x0}
   *             ├────────────────┤
   *        x2:  │     Y_{x2}     │  rowDim_{x2} rows, offset_{x2} = rowDim_{x0}
   *             └────────────────┘                               + rowDim_{x1}
   *                ↑
   *                totalDim = Σ_k rowDim_k rows
   *
   * Offsets are assigned in increasing-Key order so the layout is
   * deterministic across runs and across staircase levels.
   */
  struct GTSAM_EXPORT Layout {
    std::unordered_map<Key, LayoutSlice> slices;
    size_t totalDim = 0;

    static Layout From(const Values& values);

    bool contains(Key key) const { return slices.find(key) != slices.end(); }
    size_t size() const { return slices.size(); }
    size_t offsetOf(Key key) const;
    size_t rowDimOf(Key key) const;
    const LayoutSlice& sliceOf(Key key) const;

    /// True iff `values` matches this layout exactly (keys + per-key rowDim).
    bool conformsTo(const Values& values) const;

    /// Stack the Matrix-valued variables in `values` into one (totalDim × p)
    /// matrix. All values must share the same column count p.
    Matrix stack(const Values& values) const;

    /// Inverse of stack: split a (totalDim × p) matrix into per-key slices.
    Values unstack(const Matrix& Y) const;
  };

  /// Construct from a QcqpProblem already built at K = params.pMin. The
  /// QcqpProblem must come from a NonlinearFactorGraph so the staircase
  /// can call `qcqp.rebuildAt(p)` when climbing K.
  RiemannianStaircaseOptimizer(const QcqpProblem& qcqp,
                               const Values& initialValues,
                               const RiemannianStaircaseParams& params = {})
      : qcqp_(qcqp), initialValues_(initialValues), params_(params) {
    validateParams(params_);
  }

  /// Run the staircase. Returns the BM solution in `result.values`;
  /// `result.rounded` is left empty (use the templated overload below).
  RiemannianStaircaseResult optimize() const;

  /// Run the staircase and, on certified success, also write SO(d)-projected
  /// rotations of type `RotT` into `result.rounded`. Equivalent to calling
  /// `optimize()` then `roundToRotation<RotT>` when certified.
  template <typename RotT>
  RiemannianStaircaseResult optimize() const {
    RiemannianStaircaseResult r = optimize();
    if (r.certified) {
      const Layout layout = Layout::From(r.values);
      for (auto& [key, R] : roundToRotation<RotT>(r.values, layout)) {
        r.rounded.insert(key, R);
      }
    }
    return r;
  }

  /// Assemble the sparse symmetric certificate `S = Q + Σ_m λ_m A_m`, the
  /// Hessian of the Lagrangian. Q and A_m are stored as Hessians directly
  /// (the 0.5 convention is absorbed at QCQP-conversion time), so no
  /// (A + A^T) symmetrization is needed here. A_m must be symmetric.
  static Eigen::SparseMatrix<double> assembleCertificate(
      const QcqpProblem& qcqp, const Layout& layout,
      const std::vector<Vector>& lambdaEq);

  /// Lift Y* from rank p to rank p+1 by appending `alpha * vMin` as the new
  /// column. No retraction: the descent direction `[0 | vMin]` is tangent to
  /// the constraint surface at `[Y* | 0]`, so the constraint violation is
  /// O(alpha^2) and the next ALM solve absorbs it.
  static Values escapeSaddleAndLift(const Values& Ystar, const Vector& vMin,
                                    const Layout& layout, double alpha);

  /**
   * Round the BM solution Y back to one `RotT` per variable.
   *
   * Recipe (d = traits<RotT>::QcqpIntrinsicDim):
   *   1. Stack into (totalDim × p) and rank-d truncated SVD → Yd.
   *   2. Global sign-gauge fix: if majority of d×d blocks have det < 0,
   *      flip the last column of Yd.
   *   3. Per-block: `R := RotT::ClosestTo(block.transpose())`.
   *
   * RotT must expose `traits<RotT>::QcqpIntrinsicDim >= 2` and a static
   * `RotT::ClosestTo`.
   */
  template <typename RotT>
  static std::vector<std::pair<Key, RotT>> roundToRotation(
      const Values& Y, const Layout& layout) {
    constexpr int d = traits<RotT>::QcqpIntrinsicDim;
    static_assert(d >= 2,
                  "RiemannianStaircaseOptimizer::roundToRotation: "
                  "traits<RotT>::QcqpIntrinsicDim must be >= 2.");

    const Matrix Yglobal = layout.stack(Y);
    Eigen::JacobiSVD<Matrix> svd(Yglobal,
                                 Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix Yd = svd.matrixU().leftCols(d) *
                svd.singularValues().head(d).asDiagonal();

    size_t numNegDet = 0;
    for (const auto& [_, slice] : layout.slices) {
      if (Yd.block(slice.offset, 0, d, d).determinant() < 0) ++numNegDet;
    }
    if (numNegDet > layout.size() / 2) Yd.col(d - 1) *= -1.0;

    std::vector<std::pair<Key, RotT>> rotations;
    rotations.reserve(layout.size());
    for (const auto& [key, slice] : layout.slices) {
      rotations.emplace_back(
          key, RotT::ClosestTo(Yd.block(slice.offset, 0, d, d).transpose()));
    }
    return rotations;
  }

  /// Dispatch on `params_.verificationMethod`. Returns
  /// `(passed, lambda_min, v_min)` where `passed == (lambda_min >= -eta)`.
  std::tuple<bool, double, Vector> verify(
      const Eigen::SparseMatrix<double>& S) const;

  /// `mutable` because optimize() calls qcqp_.rebuildAt(p) per level; the
  /// staircase is a pure function of its constructor args externally.
  mutable QcqpProblem qcqp_;
  Values initialValues_;
  RiemannianStaircaseParams params_;

 private:
  std::tuple<bool, double, Vector> verifyLOBPCG(
      const Eigen::SparseMatrix<double>& S) const;
  std::tuple<bool, double, Vector> verifyDenseEigen(
      const Eigen::SparseMatrix<double>& S) const;
  std::tuple<bool, double, Vector> verifySpectra(
      const Eigen::SparseMatrix<double>& S) const;

  static void validateParams(const RiemannianStaircaseParams& params);
};

}  // namespace gtsam
