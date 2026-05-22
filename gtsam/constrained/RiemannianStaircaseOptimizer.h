/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.h
 * @brief   Riemannian Staircase outer loop with a local solver for the
 *          inner low-rank factorized problem (currently the Augmented
 *          Lagrangian method) and a sparse-eigensolver certificate of
 *          optimality. A specialized structure-exploiting Riemannian-manifold
 *          solver is planned to replace ALM in a future release.
 * @author  Zhexin Xu
 * @author  David M. Rosen
 *
 * References:
 *   Zhexin Xu, Nikolas R. Sanderson, Hanna Jiamei Zhang, David M. Rosen —
 *     "Certifiable Estimation with Factor Graphs," arXiv:2603.01267, 2026.
 *   David M. Rosen — "Accelerating certifiable estimation with preconditioned
 *     eigensolvers," IEEE RA-L 7(4):12507–12514, 2022.
 *   David M. Rosen — "Scalable low-rank semidefinite programming for
 *     certifiably correct machine perception," WAFR 2020.
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

#include <optional>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

struct RiemannianStaircaseResult;

/// Hyperparameters for the Riemannian Staircase outer loop.
struct GTSAM_EXPORT RiemannianStaircaseParams {
  /// LOBPCG: sparse, scalable, requires GTSAM_USE_LOBPCG_VERIFICATION.
  /// Spectra: sparse Lanczos via gtsam/3rdparty, no CHOLMOD dependency.
  /// DenseEigen: full O(n^3) diagonalization — debugging only, does not scale.
  /// LOBPCG is recommended, especially for large-scale problems.
  enum class VerificationMethod { LOBPCG, DenseEigen, Spectra };

  size_t pMin = 2;            ///< Starting Riemannian Staircase level; must be ≥ the ambient column dimension d.
  size_t pMax = 10;           ///< staircase cap before giving up.
  double alpha = 1e-2;        ///< descent step size on saddle escape.

  VerificationMethod verificationMethod = VerificationMethod::LOBPCG;
  double eta = 1e-3;           ///< min-eigenvalue tolerance for certification.
  size_t nx = 1;               ///< LOBPCG block size.
  size_t maxLOBPCGIters = 1000;
  double maxFillFactor = 3.0;  ///< ILDL fill factor.
  double dropTol = 1e-3;       ///< ILDL drop tolerance.

  size_t maxSpectraIters = 1000;
  size_t numLanczosVectors = 100;
  double spectraTol = 1e-4;

  AugmentedLagrangianParams::shared_ptr almParams =
      std::make_shared<AugmentedLagrangianParams>();
  bool verbose = false;
};

/**
 * Riemannian Staircase outer loop over a Burer–Monteiro low-rank
 * parametrization of a QCQP. Takes a `QcqpProblem` and an initial value and
 * returns a certified (or best-effort) low-rank BM solution.
 *
 * Per level: (1) inner NLP solve at rank p yielding (Y*, λ); (2) certificate
 * `S = Q + Σ_m λ_m A_m`; (3) verify positive semidefiniteness via an
 * eigenvalue-optimization method; on fail, (4) `escapeSaddleAndLift` appends
 * a descent column at rank p+1.
 */
class GTSAM_EXPORT RiemannianStaircaseOptimizer {
 public:
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

    /// Largest `rowDim` across slices. Used as the default rank-d for
    /// `roundingSolution` since rotation blocks dominate; translations
    /// (rowDim=1) shouldn't drive the truncation.
    size_t maxRowDim() const;
  };

  /// Rank-d SVD truncation of the stacked BM matrix Y, with an optional
  /// last-column sign flip so the rotation-shaped blocks round to det ≥ 0
  /// rather than to reflections. `d` is recorded so downstream extractors
  /// don't need to be told again.
  struct RoundedSolution {
    Matrix Yd;     ///< (totalDim × d).
    int d;         ///< rounding dim used.
    bool flipped;  ///< whether the last-column sign flip was applied.
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

  /// Run the staircase. On certification, also fills `result.layout` and
  /// `result.rounded` with the rank-d projection (d = layout.maxRowDim()).
  /// Type-specific outputs are recovered by calling
  /// `extractRotations<RotT>` / `extractRowVectors` on `result.rounded`.
  RiemannianStaircaseResult optimize() const;

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

  /// SVD-truncate `layout.stack(Y)` to rank `d` and optionally sign-flip the
  /// last column so rotation-shaped blocks round to det ≥ 0. Caller can
  /// override `d`; `optimize()` uses `layout.maxRowDim()`. The result is
  /// consumed by the per-type extractors below.
  static RoundedSolution roundingSolution(const Values& Y, const Layout& layout,
                                          int d);

  /// Constrained-variable extractor: each slice with
  /// `rowDim == traits<RotT>::QcqpIntrinsicDim` is projected onto a `RotT`
  /// via `RotT::ClosestTo` (rotations live on a manifold and need this
  /// projection). Slices of other sizes are skipped. More constrained
  /// variable types can be added by following the same Layout-slice pattern.
  template <typename RotT>
  static std::vector<std::pair<Key, RotT>> extractRotations(
      const RoundedSolution& r, const Layout& layout) {
    constexpr int d = traits<RotT>::QcqpIntrinsicDim;
    std::vector<std::pair<Key, RotT>> rotations;
    for (const auto& [key, slice] : layout.slices) {
      if (slice.rowDim != static_cast<size_t>(d)) continue;
      rotations.emplace_back(
          key, RotT::ClosestTo(r.Yd.block(slice.offset, 0, d, d).transpose()));
    }
    return rotations;
  }

  /// Unconstrained-variable extractor: each `rowDim == 1` slice is read off
  /// as a plain `r.d`-vector (no projection needed; the variable lives in
  /// Euclidean space). Today this covers translations in PGO; the same
  /// pattern extends to any future unconstrained 1-row variable.
  static std::vector<std::pair<Key, Vector>> extractRowVectors(
      const RoundedSolution& r, const Layout& layout);

  /// Test the PSD of the certificate via minimum-eigenvalue optimization,
  /// using whichever backend `params_.verificationMethod` selects.
  std::tuple<bool, double, Vector> verify(
      const Eigen::SparseMatrix<double>& S) const;

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

struct GTSAM_EXPORT RiemannianStaircaseResult {
  Values values;                                  ///< BM solution.
  RiemannianStaircaseOptimizer::Layout layout;    ///< Per-key slices in Y.
  /// SVD-rounded BM solution (rank `layout.maxRowDim()`)
  std::optional<RiemannianStaircaseOptimizer::RoundedSolution> rounded;

  size_t finalRank = 0;
  bool certified = false;
  double minEigenvalue = 0.0;
  /// Per-level diagnostics recorded during the staircase climb.
  std::vector<size_t> ranksVisited;
  std::vector<double> costPerLevel;
  std::vector<double> minEigenvaluePerLevel;
  std::vector<double> nlpTimePerLevel;
  std::vector<double> verifyTimePerLevel;
  double totalTime = 0.0;
};

}  // namespace gtsam
