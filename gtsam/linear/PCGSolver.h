/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PCGSolver.h
 * @brief Preconditioned Conjugate Gradient Solver for linear systems
 * @date Jan 31, 2012
 * @author Yong-Dian Jian
 * @author Sungtae An
 * @author Fan Jiang
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/linear/ConjugateGradientSolver.h>
#include <gtsam/linear/VectorValues.h>

#include <string>

namespace gtsam {

class GaussianFactorGraph;
class Preconditioner;
struct PreconditionerParameters;

/**
 * Parameters for Preconditioned Conjugate Gradient solver.
 */
struct GTSAM_EXPORT PCGSolverParameters : public ConjugateGradientParameters {
  typedef ConjugateGradientParameters Base;
  typedef std::shared_ptr<PCGSolverParameters> shared_ptr;

  /// Parameters used to construct the preconditioner for each solve.
  std::shared_ptr<PreconditionerParameters> preconditioner;
  bool parallel = true;   ///< Enable scheduler-backed parallel kernels.
  size_t numThreads = 0;  ///< Worker count (0 selects an automatic count).

  PCGSolverParameters() {}

  PCGSolverParameters(
      const std::shared_ptr<PreconditionerParameters>& preconditioner)
      : preconditioner(preconditioner) {}

  void print(std::ostream& os) const override;
  void print(const std::string& s) const;
};

/** Solution, convergence diagnostics, and phase timings from PCGSolver. */
struct GTSAM_EXPORT PCGSolverResult {
  VectorValues solution;         ///< Final solution in keyed form.
  ConjugateGradientStats stats;  ///< Convergence diagnostics for the PCG loop.
  double operatorSetupSeconds =
      0.0;  ///< Time to compile the flat matrix-free operator.
  double preconditionerSetupSeconds =
      0.0;  ///< Time to build and factorize the preconditioner.
  double solveSeconds =
      0.0;  ///< Time for PCG and conversion of its result to VectorValues.
};

/**
 * A virtual base class for the preconditioned conjugate gradient solver
 */
class GTSAM_EXPORT PCGSolver : public IterativeSolver {
 public:
  typedef IterativeSolver Base;
  typedef std::shared_ptr<PCGSolver> shared_ptr;

 protected:
  PCGSolverParameters parameters_;
  std::shared_ptr<Preconditioner> preconditioner_;

 public:
  /**
   * Construct a solver with the supplied iteration and preconditioner settings.
   *
   * @param p Settings copied into the solver.
   */
  PCGSolver(const PCGSolverParameters& p);
  ~PCGSolver() override {}

  using IterativeSolver::optimize;

  /**
   * Solve using explicit key metadata, damping, and an initial estimate.
   *
   * @param gfg Gaussian factor graph defining the normal equations.
   * @param keyInfo Ordering, dimensions, and scalar offsets for graph keys.
   * @param lambda Per-key damping forwarded to the preconditioner.
   * @param initial Initial estimate in keyed form.
   * @return Final solution in keyed form.
   */
  VectorValues optimize(const GaussianFactorGraph& gfg, const KeyInfo& keyInfo,
                        const std::map<Key, Vector>& lambda,
                        const VectorValues& initial) override;

  /**
   * Optimize from zero and return convergence diagnostics and phase timings.
   * Residual history is collected when collectResidualHistory is true.
   *
   * @param gfg Gaussian factor graph defining the normal equations.
   * @param collectResidualHistory Whether to retain per-iteration residuals.
   * @return Solution, convergence diagnostics, and phase timings.
   */
  PCGSolverResult optimizeDetailed(const GaussianFactorGraph& gfg,
                                   bool collectResidualHistory = true);

  /**
   * Optimize with explicit metadata and an initial estimate, returning
   * convergence diagnostics and phase timings.
   *
   * @param gfg Gaussian factor graph defining the normal equations.
   * @param keyInfo Ordering, dimensions, and scalar offsets for graph keys.
   * @param lambda Per-key damping forwarded to the preconditioner.
   * @param initial Initial estimate in keyed form.
   * @param collectResidualHistory Whether to retain per-iteration residuals.
   * @return Solution, convergence diagnostics, and phase timings.
   */
  PCGSolverResult optimizeDetailed(const GaussianFactorGraph& gfg,
                                   const KeyInfo& keyInfo,
                                   const std::map<Key, Vector>& lambda,
                                   const VectorValues& initial,
                                   bool collectResidualHistory = true);
};

/**
 * Compiled flat-vector system used by preconditioned conjugate gradients.
 *
 * Construction caches graph metadata and the right-hand side, compiles
 * supported factors into flat execution plans, and retains a compatibility
 * graph for other GaussianFactor implementations.
 *
 * @note Multiplication reuses mutable workspace and block-diagonal assembly is
 * lazy. Concurrent calls on one instance, or on copies sharing the same
 * implementation, require external synchronization.
 */
class GTSAM_EXPORT GaussianFactorGraphSystem {
  class Impl;

  const Preconditioner& preconditioner_;
  std::shared_ptr<Impl> impl_;

  const std::vector<Matrix>& hessianBlockDiagonal() const;

  friend class PCGSolver;

 public:
  /**
   * Compile a Gaussian factor graph into a flat-vector PCG system.
   *
   * Parallel execution is enabled by default for sufficiently large systems.
   * Set @p parallel to false or @p numThreads to one for serial execution.
   * A zero @p numThreads selects an automatically capped worker count.
   *
   * @param gfg Gaussian factor graph to compile and retain as needed.
   * @param preconditioner Built preconditioner used by split solves.
   * @param info Ordering, dimensions, and scalar offsets for graph keys.
   * @param lambda Per-key damping retained for API compatibility.
   * @param parallel Whether large kernels may use the task scheduler.
   * @param numThreads Requested worker count; zero selects automatically.
   */
  GaussianFactorGraphSystem(const GaussianFactorGraph& gfg,
                            const Preconditioner& preconditioner,
                            const KeyInfo& info,
                            const std::map<Key, Vector>& lambda,
                            bool parallel = true, size_t numThreads = 0);

  /// Overwrite @p r with the residual `b - H*x` in flat KeyInfo ordering.
  void residual(const Vector& x, Vector& r) const;

  /// Overwrite @p y with the Hessian-vector product `H*x`.
  void multiply(const Vector& x, Vector& y) const;

  /// Overwrite @p y with the left split-preconditioner solve `L^{-1}*x`.
  void leftPrecondition(const Vector& x, Vector& y) const;

  /// Overwrite @p y with the right split-preconditioner solve `L^{-T}*x`.
  void rightPrecondition(const Vector& x, Vector& y) const;

  /// Scale @p x in place by @p alpha.
  void scal(const double alpha, Vector& x) const;

  /// Return the Euclidean dot product of two flat vectors.
  double dot(const Vector& x, const Vector& y) const;

  /// Accumulate `alpha*x` into @p y.
  void axpy(const double alpha, const Vector& x, Vector& y) const;

  /// Overwrite @p b with the cached normal-equation right-hand side.
  void getb(Vector& b) const;

  /// Return the effective worker count, or one for serial execution.
  size_t numThreads() const;
};

/// @name utility functions
/// @{

/**
 * Convert a flat vector to keyed blocks using an explicit ordering.
 *
 * @param v Flat vector whose blocks follow @p ordering.
 * @param ordering Key order used to slice @p v.
 * @param dimensions Scalar dimension of each ordered key.
 * @return Keyed vector blocks copied from @p v.
 * @throws std::invalid_argument if an ordered key has no dimension entry.
 */
GTSAM_EXPORT VectorValues
buildVectorValues(const Vector& v, const Ordering& ordering,
                  const std::map<Key, size_t>& dimensions);

/// @}

}  // namespace gtsam
