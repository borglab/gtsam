/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    RiemannianStaircaseOptimizer.h
 * @brief   Burer–Monteiro Solver for the SDP Relaxation of a QCQP-Representable Factor Graph
 *
 * @author  Zhexin Xu
 * @author  David M. Rosen
 *
 * References:
 *   Zhexin Xu, Nikolas R. Sanderson, Hanna Jiamei Zhang, David M. Rosen.
 *     "Certifiable Estimation with Factor Graphs." (Certi-fgo) arXiv:2603.01267, 2026.
 *   David M. Rosen. "Scalable low-rank semidefinite programming for
 *     certifiably correct machine perception." Proc. Workshop on the
 *     Algorithmic Foundations of Robotics (WAFR), 2020.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QcqpProblem.h>
#include <gtsam/dllexport.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <Eigen/Sparse>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

struct RiemannianStaircaseResult;

/// Parameters for the Riemannian Staircase solver.
struct GTSAM_EXPORT RiemannianStaircaseParams {
  /// Spectra: sparse Lanczos following SE-Sync's SparseMinimumEigenValue.
  /// DenseEigen: full O(n^3) self-adjoint eigensolver, not recommended for large problems.
  enum class VerificationMethod { Spectra, DenseEigen };

  size_t pMin = 2;             /// Initial level (column dimension here) of Riemannian staircase, should be >= the ambient dim.
  size_t pMax = 10;            ///< staircase cap before giving up.
  double alpha = 1e-2;         ///< descent step size on saddle escape.
  /// Fit the saddle-escape step to the merit instead of always using `alpha`.
  /// Off by default: it never changed the certifying rank on any dataset
  /// measured, and cost about 2x on M3500.
  bool useSaddleLineSearch = false;
  /// Sets where the step search starts:
  /// `max(1024 * alpha, 10 * saddleStepTolerance / |lambda_min|)`.
  double saddleStepTolerance = 1e-2;

  VerificationMethod verificationMethod = VerificationMethod::Spectra;
  double eta = 1e-3;           ///< min-eigenvalue tolerance for certification.

  /// Lanczos iteration cap. 1000 is plenty for one extremal eigenpair on
  /// typical rotation-averaging problems; raise only if Pass 2 fails to
  /// converge on a very ill-conditioned spectrum.
  size_t maxSpectraIters = 1000;
  /// Lanczos subspace size. 20 is the value Shonan RA uses.
  size_t numLanczosVectors = 20;
  /// Absolute tolerance on lambda_min.
  double spectraTol = 1e-4;

  /// Parameters for inner local solver ALM.
  AugmentedLagrangianParams::shared_ptr almParams =
      std::make_shared<AugmentedLagrangianParams>();
  bool verbose = false;

  /**
   * Return a copy of the inner ALM parameters.
   *
   * This accessor gives language wrappers a concrete parameter object instead
   * of exposing the nested shared pointer directly.
   */
  AugmentedLagrangianParams getAlmParams() const { return *almParams; }

  /**
   * Replace the inner ALM parameters with a copy of @p parameters.
   *
   * Copying keeps the staircase parameters independent of subsequent changes
   * to the caller's parameter object.
   */
  void setAlmParams(const AugmentedLagrangianParams& parameters) {
    almParams = std::make_shared<AugmentedLagrangianParams>(parameters);
  }
};

/**
 * Burer-Monteiro Riemannian Staircase over a QCQP-representable estimation
 * problem (Certi-FGO Algorithm 1).
 *
 * Problem chain (Certi-FGO Sec V; eq. numbers below refer to that paper):
 *   QCQP (eq. 12):  min_{X in R^{r x d}}  <Q, X X'>
 *                    s.t.  <A_m, X X'> = b_m,  m in [M].
 *   SDP  (eq. 13):  min_{Z in S^r_+}      <Q, Z>
 *                    s.t.  A(Z)_m = <A_m, Z> = b_m.
 *   BM   (eq. 16):  min_{Y in R^{r x p}}  <Q, Y Y'>
 *                    s.t.  <A_m, Y Y'> = b_m
 *                    (Z = Y Y' parameterizes the SDP at rank <= p).
 *
 * KKT certificate matrix (eq. 20):
 *      S := Q + 2 * sum_m (lambda_m / sigma_m) * A_m,
 * which is eq. (20)'s S = Q + A*(lambda) written in GTSAM's conventions:
 * QpCost is 0.5 * tr(X' Q X) while a QuadraticConstraint has gradient 2 A X,
 * and ALM multipliers pair with the whitened residual h_m / sigma_m. A* is
 * the adjoint of the constraint map A(Z)_m := <A_m, Z>. At a
 * 1st-order KKT point Y of the BM problem, Y is a global SDP optimum iff
 * S >= 0 (Thm 1(a)). Otherwise Thm 1(b) gives v with v' S v < 0, and
 * Y+ = [Y | 0] lifted with descent direction [0 | v] is a feasible
 * 2nd-order descent for the rank-(p+1) BM problem (saddle escape).
 *
 * Algorithm 1 per outer level p in [pMin, pMax]:
 *  (1) build the rank-p QCQP via `QcqpProblem(graph, p)`;
 *  (2) `runLocalSolver` (ALM today) yields (Y*, lambda*);
 *  (3) build S via `buildCertificate` (= `buildDataMatrix` +
 *      `buildMultiplierMatrix`);
 *  (4) `verify` checks S >= 0 (Cholesky-first; Spectra fallback for the
 *      saddle-escape eigenvector);
 *  (5) If Verified: `truncateToRankD` projects Y* to rank d =
 *      layout.maxRowDim() and the result is returned. Otherwise
 *      `saddleEscapeWithLineSearch` lifts to rank p+1 and the loop
 *      continues.
 *
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
   * The cost Q, constraints A_m, certificate S, and SVD rounding step all
   * work on Y as one (totalDim x p) dense matrix rather than on per-key
   * `Values`. `Layout` is how we get between the two forms: for each Key it
   * stores the row offset and row count of that variable's slice inside Y.
   * Offsets are assigned in sorted-Key order, so they stay the same across
   * runs and across staircase levels.
   *
   * Block structure of Y (keys x0, x1, x2 with rowDim 2, 3, 2, p columns):
   *
   *                  <----------- p ----------->
   *                +---------------------------+   ---            ---
   *                |                           |    ^              |
   *      x0 slice  |          Y_x0             |  rowDim=2         |
   *                |                           |    v              |
   *                +---------------------------+   ---             |
   *                |                           |    ^              |
   *      x1 slice  |          Y_x1             |  rowDim=3      totalDim
   *                |                           |    v              |
   *                +---------------------------+   ---             |
   *                |                           |    ^              |
   *      x2 slice  |          Y_x2             |  rowDim=2         |
   *                |                           |    v              v
   *                +---------------------------+   ---            ---
   *
   * Offsets: x0 -> 0, x1 -> 2, x2 -> 5. totalDim = 2 + 3 + 2 = 7.
   * Per-key slice: `Y_i = Y.block(offset_i, 0, rowDim_i, p)`.
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

    /// Stack the Matrix-valued variables in `values` into one (totalDim x p)
    /// matrix. All values must share the same column count p.
    Matrix stack(const Values& values) const;

    /// Inverse of stack: split a (totalDim x p) matrix into per-key slices.
    Values unstack(const Matrix& Y) const;

    /// Largest `rowDim` across slices. Used as the default rank-d for
    /// `truncateToRankD` since rotation blocks dominate; translations
    /// (rowDim=1) shouldn't drive the truncation.
    size_t maxRowDim() const;
  };

  /// Output of `truncateToRankD`. The projection rank is `Yd.cols()`.
  struct RoundedSolution {
    Matrix Yd;     ///< Rank-d projection of stacked Y, shape (totalDim × d).
  };

  /// Output of one local-solver call.
  struct InnerSolveResult {
    Values Y;
    std::vector<Vector> lambdaEq;
    double stationarity = 0.0;
    double penalty = 0.0;
  };

  /// Construct from a QCQP-representable factor graph and an initial
  /// feasible value. Every factor in `graph` must override
  /// `NonlinearFactor::qcqpFactors` (the base implementation throws).
  /// The rank-pMin problem is built once here for fail-fast validation and
  /// reused by `optimize`; higher staircase levels are built on demand.
  RiemannianStaircaseOptimizer(const NonlinearFactorGraph& graph,
                               const Values& initialValues,
                               const RiemannianStaircaseParams& params = {});

  /// Run the staircase. If it certifies, `result.rounded` holds the rank-d projection.
  RiemannianStaircaseResult optimize() const;

  /// Zero-pad each Matrix entry of `Y` to `pMin` columns. Use when entries are narrower than `pMin`.
  static Values padInitialValues(const Values& Y, size_t pMin);

  /// Run the local solver (ALM here). Returns Y* and multipliers (if the solver provides).
  static InnerSolveResult runLocalSolver(
      const QcqpProblem& qcqp, const Values& Y0,
      AugmentedLagrangianParams::shared_ptr almParams);

  /// Build Q = sum_k H_k, the data Hessian assembled from each QpCost
  /// factor's `hessianFactor()`. Each H_k is laid out in K-Kronecker form;
  /// the natural rowDim_i x rowDim_j block is its top-left corner.
  ///
  /// Q does not depend on the staircase rank: `layout` is indexed by row
  /// dimension, and lifting only adds columns. `dataMatrix()` caches it.
  static Eigen::SparseMatrix<double> buildDataMatrix(
      const QcqpProblem& qcqp, const Layout& layout);

  /// Build A*(lambda) = sum_m (lambda_m / sigma_m) * A_m, the adjoint constraint matrix.
  ///
  /// `lambda_m` are inner-solver multipliers for the whitened equality
  /// factors `h_m(X) = (trace(X' A_m X) - b_m) / sigma_m`; we divide by
  /// sigma_m to recover the unwhitened multiplier that pairs with A_m.
  /// `sigma_m` is GTSAM noise-model bookkeeping, not part of the SDP math.
  ///
  /// This returns A*(lambda) undoubled; `buildCertificate` applies the factor
  /// of two that the constraint gradient contributes.
  static Eigen::SparseMatrix<double> buildMultiplierMatrix(
      const QcqpProblem& qcqp, const Layout& layout,
      const std::vector<Vector>& lambdaEq);

  /// Build the SDP dual matrix S = Q + 2 * A*(lambda), i.e. `buildDataMatrix`
  /// plus twice `buildMultiplierMatrix`. The factor of two comes from the
  /// constraint gradient, as above.
  ///
  /// Assumes Y is a 1st-order KKT point of the BM problem (what the local
  /// solver should return). Under that assumption, S >= 0 iff Z = Y Y' is
  /// a global SDP optimum (Thm 1(a)).
  static Eigen::SparseMatrix<double> buildCertificate(
      const QcqpProblem& qcqp, const Layout& layout,
      const std::vector<Vector>& lambdaEq);

  /// Overload reusing an already-assembled Q, which is rank-independent.
  static Eigen::SparseMatrix<double> buildCertificate(
      const QcqpProblem& qcqp, const Layout& layout,
      const std::vector<Vector>& lambdaEq,
      const Eigen::SparseMatrix<double>& dataMatrix);

  /// Multipliers recovered from a point, with the residual they leave.
  struct LeastSquaresMultipliers {
    /// In `eConstraints()` order and in `buildCertificate`'s convention, so
    /// this is interchangeable with the solver's own multipliers.
    std::vector<Vector> lambdaEq;
    /// Per-variable `||(QY)_n + sum_m lambda_m A_m Y_n||_F`. Unconstrained
    /// variables contribute `||(QY)_n||_F`, which no multiplier can reduce.
    std::map<Key, double> residual;
    /// `||S Y||_F`. Zero exactly at a first-order stationary point.
    double totalResidual = 0.0;
  };

  /// Least-squares Lagrange multipliers at `Y`:
  ///
  ///     lambda_LS = argmin_lambda || S(lambda) Y ||_F
  ///
  /// with `S(lambda)` the same matrix `buildCertificate` forms, so the result
  /// can be passed straight to it. These are the multipliers that come
  /// closest to satisfying KKT stationarity, and are unique when LICQ holds
  /// at `Y`.
  ///
  /// Constraints are block-separable, so the multiplier term is block
  /// diagonal and the problem splits into one small solve per constrained
  /// variable.
  static LeastSquaresMultipliers leastSquaresMultipliers(
      const QcqpProblem& qcqp, const Layout& layout, const Values& Y);

  /// Overload reusing an already-assembled Q, which is rank-independent.
  static LeastSquaresMultipliers leastSquaresMultipliers(
      const QcqpProblem& qcqp, const Layout& layout, const Values& Y,
      const Eigen::SparseMatrix<double>& dataMatrix);

  /// Outcome of a line-searched saddle escape.
  struct SaddleEscape {
    Values lifted;               ///< Y at rank p+1, stepped by `alpha`.
    double alpha = 0.0;          ///< Step actually taken.
    double meritDecrease = 0.0;  ///< M(0) - M(alpha).
    bool descentFound = false;   ///< False when no step decreases the merit.
  };

  /// Pick the saddle-escape step by decreasing the augmented-Lagrangian merit
  /// along the lift direction.
  /// The criterion is merit decrease, ALM being the local solver in use.
  static SaddleEscape saddleEscapeWithLineSearch(
      const QcqpProblem& liftedQcqp, const Layout& layout, const Values& Ystar,
      const Vector& vMin, const std::vector<Vector>& lambdaEq, double penalty,
      double minEigenvalue, const RiemannianStaircaseParams& params);

  /// Lift Y* from rank p to rank p+1 by appending `alpha * vMin` as a new
  /// column (Thm 1(b), Algorithm 1 lines 10-12). With vMin the negative-
  /// eigenvalue eigenvector of S, this is also a feasible 2nd-order descent
  /// direction.
  static Values liftWithDescent(const Values& Ystar, const Vector& vMin,
                                const Layout& layout, double alpha);

  /// SVD-based rank-d projection for d >= 1: round the stacked BM matrix from
  /// rank p down to rank d.
  static RoundedSolution truncateToRankD(const Values& Y, const Layout& layout,
                                         int d);

  /// Test PSD of the certificate matrix S. Returns `(passed, lambda_min,
  /// v_min)`.
  ///
  /// The Spectra method is two stage, mirroring SE-Sync's `fast_verification`:
  ///   Stage 1 (fast check): sparse Cholesky on M = S + eta * I. Success
  ///     is binary and ~O(nnz), much cheaper than an eigenvalue computation.
  ///     Returns lambda_min = -eta as a conservative lower bound; v_min is
  ///     unused on the cert-pass path.
  ///   Stage 2 (only on Cholesky failure): two-pass Lanczos shift trick to get
  ///     lambda_min and the v_min the lift needs.
  static std::tuple<bool, double, Vector> verify(
      const Eigen::SparseMatrix<double>& S,
      const RiemannianStaircaseParams& params,
      const Vector& warmStart = Vector());

  // ===========================================================================

  const NonlinearFactorGraph& graph() const { return graph_; }
  const Values& initialValues() const { return initialValues_; }
  const RiemannianStaircaseParams& params() const { return params_; }

  /// The data matrix Q, assembled once at construction and reused at every
  /// staircase level.
  const Eigen::SparseMatrix<double>& dataMatrix() const { return dataMatrix_; }

 private:
  static std::tuple<bool, double, Vector> verifyDenseEigen(
      const Eigen::SparseMatrix<double>& S,
      const RiemannianStaircaseParams& params);

  static std::tuple<bool, double, Vector> verifySpectra(
      const Eigen::SparseMatrix<double>& S,
      const RiemannianStaircaseParams& params, const Vector& warmStart);

  static void validateParams(const RiemannianStaircaseParams& params);

  NonlinearFactorGraph graph_;
  Values initialValues_;
  RiemannianStaircaseParams params_;
  std::shared_ptr<const QcqpProblem> pMinQcqp_;
  Eigen::SparseMatrix<double> dataMatrix_;
  double pMinQcqpBuildTime_ = 0.0;
};

struct GTSAM_EXPORT RiemannianStaircaseResult {
  Values values;
  RiemannianStaircaseOptimizer::Layout layout;
  /// SVD-rounded BM solution
  std::optional<RiemannianStaircaseOptimizer::RoundedSolution> rounded;

  size_t finalRank = 0;
  bool certified = false;
  double minEigenvalue = 0.0;
  /// Per-level diagnostics recorded during the staircase climb.
  std::vector<size_t> ranksVisited;
  std::vector<double> costPerLevel;
  std::vector<double> minEigenvaluePerLevel;
  std::vector<double> stationarityPerLevel;
  std::vector<double> qcqpBuildTimePerLevel;
  std::vector<double> nlpTimePerLevel;
  std::vector<double> verifyTimePerLevel;
  /// Cumulative pipeline time, including the cached rank-pMin QCQP build.
  double totalTime = 0.0;

  /// Return true if certification produced a rounded rank-d solution.
  bool hasRoundedSolution() const { return rounded.has_value(); }

  /**
   * Return the rounded rank-d solution as matrix-valued variables.
   *
   * @throws std::runtime_error if the staircase did not certify and no rounded
   * solution is available.
   */
  Values roundedValues() const;

  /// Return the staircase ranks as a wrapper-friendly numeric vector.
  Vector getRanksVisited() const;

  /// Return the objective value recorded at each staircase level.
  Vector getCostPerLevel() const;

  /// Return the per-level bound on lambda_min(S).
  Vector getMinEigenvaluePerLevel() const;

  /// Return the augmented-Lagrangian gradient stationarity at each level.
  Vector getStationarityPerLevel() const;

  /// Return QCQP construction time at each staircase level, in seconds.
  Vector getQcqpBuildTimePerLevel() const;

  /// Return local nonlinear-solver time at each staircase level, in seconds.
  Vector getNlpTimePerLevel() const;

  /// Return the certificate-verification time at each level, in seconds.
  Vector getVerifyTimePerLevel() const;
};

}  // namespace gtsam
