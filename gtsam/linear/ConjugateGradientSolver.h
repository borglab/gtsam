/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   ConjugateGradientSolver.h
 *  @brief  Implementation of Conjugate Gradient solver for a linear system
 *  @author Yong-Dian Jian
 *  @author Sungtae An
 *  @date   Nov 6, 2014
 * @author Fan Jiang
 **/

#pragma once

#include <gtsam/linear/IterativeSolver.h>

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * Parameters for the Conjugate Gradient method
 */
struct GTSAM_EXPORT ConjugateGradientParameters
    : public IterativeOptimizationParameters {
  typedef IterativeOptimizationParameters Base;
  typedef std::shared_ptr<ConjugateGradientParameters> shared_ptr;

  size_t minIterations;  ///< minimum number of cg iterations
  size_t maxIterations;  ///< maximum number of cg iterations
  size_t reset;          ///< number of iterations before reset
  double epsilon_rel;    ///< threshold for relative error decrease
  double epsilon_abs;    ///< threshold for absolute error decrease

  /* Matrix Operation Kernel */
  enum BLASKernel {
    GTSAM = 0,  ///< Jacobian Factor Graph of GTSAM
  } blas_kernel;

  ConjugateGradientParameters()
      : minIterations(1),
        maxIterations(500),
        reset(501),
        epsilon_rel(1e-3),
        epsilon_abs(1e-3),
        blas_kernel(GTSAM) {}

  ConjugateGradientParameters(size_t minIterations, size_t maxIterations,
                              size_t reset, double epsilon_rel,
                              double epsilon_abs, BLASKernel blas)
      : minIterations(minIterations),
        maxIterations(maxIterations),
        reset(reset),
        epsilon_rel(epsilon_rel),
        epsilon_abs(epsilon_abs),
        blas_kernel(blas) {}

  ConjugateGradientParameters(const ConjugateGradientParameters& p)
      : Base(p),
        minIterations(p.minIterations),
        maxIterations(p.maxIterations),
        reset(p.reset),
        epsilon_rel(p.epsilon_rel),
        epsilon_abs(p.epsilon_abs),
        blas_kernel(GTSAM) {}

  ConjugateGradientParameters& operator=(
      const ConjugateGradientParameters& other) = default;

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  inline size_t getMinIterations() const { return minIterations; }
  inline size_t getMaxIterations() const { return maxIterations; }
  inline size_t getReset() const { return reset; }
  inline double getEpsilon() const { return epsilon_rel; }
  inline double getEpsilon_rel() const { return epsilon_rel; }
  inline double getEpsilon_abs() const { return epsilon_abs; }

  inline void setMinIterations(size_t value) { minIterations = value; }
  inline void setMaxIterations(size_t value) { maxIterations = value; }
  inline void setReset(size_t value) { reset = value; }
  inline void setEpsilon(double value) { epsilon_rel = value; }
  inline void setEpsilon_rel(double value) { epsilon_rel = value; }
  inline void setEpsilon_abs(double value) { epsilon_abs = value; }
#endif

  void print() const { Base::print(); }
  void print(std::ostream& os) const override;

  static std::string blasTranslator(const BLASKernel k);
  static BLASKernel blasTranslator(const std::string& s);
};

/// Reason a conjugate-gradient solve stopped.
enum class ConjugateGradientTerminationReason {
  kConverged,           ///< The requested residual tolerance was reached.
  kMaxIterations,       ///< The iteration limit was reached first.
  kNumericalBreakdown,  ///< The recurrence encountered invalid numerics.
};

/** Diagnostics collected during a conjugate-gradient solve. */
struct GTSAM_EXPORT ConjugateGradientStats {
  size_t iterations = 0;  ///< Number of completed PCG updates.
  double initialPreconditionedResidualNorm =
      0.0;  ///< Norm of the initial split-preconditioned residual.
  double finalPreconditionedResidualNorm =
      0.0;  ///< Norm of the final split-preconditioned residual.
  /// Initial norm followed by one entry per completed PCG update, when enabled.
  std::vector<double> preconditionedResidualNormHistory;
  /// Reason the solve stopped.
  ConjugateGradientTerminationReason terminationReason =
      ConjugateGradientTerminationReason::kMaxIterations;

  /// Return whether the requested residual tolerance was reached.
  bool converged() const {
    return terminationReason == ConjugateGradientTerminationReason::kConverged;
  }
};

/** Solution and diagnostics returned by the detailed CG interface. */
template <class V>
struct ConjugateGradientResult {
  V solution;  ///< Final estimate in the caller's vector type.
  ConjugateGradientStats stats;  ///< Convergence diagnostics for the solve.
};

/**
 * Solve a linear system with split-preconditioned conjugate gradients.
 *
 * The system must provide `residual`, `multiply`, `leftPrecondition`,
 * `rightPrecondition`, `scal`, `dot`, and `axpy`. For a preconditioner
 * `M = L*L.transpose()`, the recurrence operates on the residual
 * `L.inverse() * (b - A*x)` and search direction `L.transpose().inverse() * r`.
 *
 * @tparam S Linear-system type providing the required vector operations.
 * @tparam V Vector type accepted by the system.
 * @param system Linear system and split preconditioner operations.
 * @param initial Initial estimate.
 * @param parameters Iteration limits, reset interval, and residual tolerances.
 * @param collectResidualHistory Whether to retain the initial and per-iteration
 * residual norms in the returned statistics.
 * @return Final estimate together with convergence diagnostics.
 *
 * REFERENCES:
 * [1] Y. Saad, "Preconditioned Iterations," in Iterative Methods for Sparse
 * Linear Systems, 2nd ed. SIAM, 2003, ch. 9, sec. 2, pp.276-281.
 */
template <class S, class V>
ConjugateGradientResult<V> preconditionedConjugateGradientDetailed(
    const S& system, const V& initial,
    const ConjugateGradientParameters& parameters,
    bool collectResidualHistory = true) {
  V estimate, residual, direction, q1, q2;
  estimate = residual = direction = q1 = q2 = initial;

  // Initialize the split-preconditioned residual and search direction.
  system.residual(estimate, q1);                 /* q1 = b-Ax */
  system.leftPrecondition(q1, residual);         /* r = L^{-1} (b-Ax) */
  system.rightPrecondition(residual, direction); /* p = L^{-T} r */

  double currentGamma = system.dot(residual, residual);
  const double initialGamma = currentGamma;

  const size_t iMaxIterations = parameters.maxIterations,
               iMinIterations = parameters.minIterations,
               iReset = parameters.reset;
  const double threshold =
      std::max(parameters.epsilon_abs,
               parameters.epsilon_rel * parameters.epsilon_rel * initialGamma);

  ConjugateGradientStats stats;
  stats.initialPreconditionedResidualNorm =
      std::isfinite(initialGamma) && initialGamma >= 0.0
          ? std::sqrt(initialGamma)
          : std::numeric_limits<double>::quiet_NaN();
  stats.finalPreconditionedResidualNorm =
      stats.initialPreconditionedResidualNorm;
  if (collectResidualHistory) {
    stats.preconditionedResidualNormHistory.reserve(iMaxIterations + 1);
    stats.preconditionedResidualNormHistory.push_back(
        stats.initialPreconditionedResidualNorm);
  }

  if (parameters.verbosity() >= ConjugateGradientParameters::COMPLEXITY)
    std::cout << "[PCG] epsilon = " << parameters.epsilon_rel
              << ", max = " << parameters.maxIterations
              << ", reset = " << parameters.reset
              << ", ||r0||^2 = " << currentGamma
              << ", threshold = " << threshold << std::endl;

  // Classify invalid and already-converged initial states before iterating.
  if (!std::isfinite(currentGamma) || currentGamma < 0.0) {
    stats.terminationReason =
        ConjugateGradientTerminationReason::kNumericalBreakdown;
  } else if (currentGamma == 0.0 ||
             (currentGamma <= threshold && iMinIterations == 0)) {
    stats.terminationReason = ConjugateGradientTerminationReason::kConverged;
  } else {
    while (stats.iterations < iMaxIterations &&
           (currentGamma > threshold || stats.iterations < iMinIterations)) {
      const size_t iteration = stats.iterations + 1;

      // Periodically replace the recursive residual with the exact residual.
      if (iReset != 0 && iteration % iReset == 0) {
        system.residual(estimate, q1);                 /* q1 = b-Ax */
        system.leftPrecondition(q1, residual);         /* r = L^{-1} (b-Ax) */
        system.rightPrecondition(residual, direction); /* p = L^{-T} r */
        currentGamma = system.dot(residual, residual);
        if (!std::isfinite(currentGamma) || currentGamma < 0.0) {
          stats.finalPreconditionedResidualNorm =
              std::numeric_limits<double>::quiet_NaN();
          if (collectResidualHistory) {
            stats.preconditionedResidualNormHistory.back() =
                stats.finalPreconditionedResidualNorm;
          }
          stats.terminationReason =
              ConjugateGradientTerminationReason::kNumericalBreakdown;
          break;
        }
        stats.finalPreconditionedResidualNorm = std::sqrt(currentGamma);
        if (collectResidualHistory) {
          stats.preconditionedResidualNormHistory.back() =
              stats.finalPreconditionedResidualNorm;
        }
        if (currentGamma == 0.0 ||
            (currentGamma <= threshold && stats.iterations >= iMinIterations)) {
          stats.terminationReason =
              ConjugateGradientTerminationReason::kConverged;
          break;
        }
      }

      // Apply one PCG step, rejecting invalid or non-positive curvature.
      system.multiply(direction, q1); /* q1 = A p */
      const double directionCurvature = system.dot(direction, q1);
      if (!std::isfinite(directionCurvature) || directionCurvature <= 0.0) {
        stats.terminationReason =
            ConjugateGradientTerminationReason::kNumericalBreakdown;
        break;
      }

      const double alpha = currentGamma / directionCurvature;
      if (!std::isfinite(alpha)) {
        stats.terminationReason =
            ConjugateGradientTerminationReason::kNumericalBreakdown;
        break;
      }

      system.axpy(alpha, direction, estimate); /* estimate += alpha * p */
      system.leftPrecondition(q1, q2);         /* q2 = L^{-1} * q1 */
      system.axpy(-alpha, q2, residual);       /* r -= alpha * q2 */
      const double previousGamma = currentGamma;
      currentGamma = system.dot(residual, residual); /* gamma = |r|^2 */
      ++stats.iterations;

      if (!std::isfinite(currentGamma) || currentGamma < 0.0) {
        stats.finalPreconditionedResidualNorm =
            std::numeric_limits<double>::quiet_NaN();
        if (collectResidualHistory) {
          stats.preconditionedResidualNormHistory.push_back(
              stats.finalPreconditionedResidualNorm);
        }
        stats.terminationReason =
            ConjugateGradientTerminationReason::kNumericalBreakdown;
        break;
      }

      stats.finalPreconditionedResidualNorm =
          std::sqrt(std::max(0.0, currentGamma));
      if (collectResidualHistory) {
        stats.preconditionedResidualNormHistory.push_back(
            stats.finalPreconditionedResidualNorm);
      }

      if (parameters.verbosity() >= ConjugateGradientParameters::ERROR)
        std::cout << "[PCG] k = " << iteration << ", alpha = " << alpha
                  << ", ||r||^2 = "
                  << currentGamma
                  //                 << "\nx =\n" << estimate
                  //                 << "\nr =\n" << residual
                  << std::endl;

      if (currentGamma == 0.0 ||
          (currentGamma <= threshold && stats.iterations >= iMinIterations)) {
        stats.terminationReason =
            ConjugateGradientTerminationReason::kConverged;
        break;
      }

      // Update the conjugate search direction for the next iteration.
      const double beta = currentGamma / previousGamma;
      if (!std::isfinite(beta)) {
        stats.terminationReason =
            ConjugateGradientTerminationReason::kNumericalBreakdown;
        break;
      }
      system.rightPrecondition(residual, q1); /* q1 = L^{-T} r */
      system.scal(beta, direction);
      system.axpy(1.0, q1, direction); /* p = q1 + beta * p */
    }

    if (stats.terminationReason !=
            ConjugateGradientTerminationReason::kNumericalBreakdown &&
        stats.terminationReason !=
            ConjugateGradientTerminationReason::kConverged) {
      stats.terminationReason =
          currentGamma <= threshold && stats.iterations >= iMinIterations
              ? ConjugateGradientTerminationReason::kConverged
              : ConjugateGradientTerminationReason::kMaxIterations;
    }
  }

  if (std::isfinite(currentGamma) && currentGamma >= 0.0) {
    stats.finalPreconditionedResidualNorm = std::sqrt(currentGamma);
  }

  if (parameters.verbosity() >= ConjugateGradientParameters::COMPLEXITY)
    std::cout << "[PCG] iterations = " << stats.iterations
              << ", ||r||^2 = " << currentGamma << std::endl;

  return {std::move(estimate), std::move(stats)};
}

/**
 * Solve a preconditioned linear system and return only the estimate.
 *
 * Use preconditionedConjugateGradientDetailed() when convergence diagnostics
 * are required.
 *
 * @tparam S Linear-system type providing the PCG vector operations.
 * @tparam V Vector type accepted by the system.
 * @param system Linear system and split preconditioner operations.
 * @param initial Initial estimate.
 * @param parameters Iteration limits, reset interval, and residual tolerances.
 * @return Final estimate.
 */
template <class S, class V>
V preconditionedConjugateGradient(
    const S& system, const V& initial,
    const ConjugateGradientParameters& parameters) {
  return preconditionedConjugateGradientDetailed(system, initial, parameters,
                                                 false)
      .solution;
}

}  // namespace gtsam
