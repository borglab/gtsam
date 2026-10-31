/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file Gradients.h
 * @brief Differentiate through GTSAM, for example to learn factor parameters.
 */

#pragma once

#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <optional>
#include <string>

namespace gtsam {

class GaussianBayesTree;

/**
 * A class to help differentiate through GTSAM, for example to learn parameters
 * of factors from a loss on the optimized estimates.
 *
 * Construct Gradients after solving a factor graph. Supply ordinary loss
 * gradients on the estimates in VectorValues; the API calls these state
 * cotangents. For loss 0.5 * ||x - target||^2, the cotangent is x - target.
 * parameterGradient asks the factors for contributions to a parameter vector
 * identified by a shared ParameterTag. Factors implement
 * NoiseModelFactor::parameterVjp for their unwhitened factor residuals;
 * Gradients handles noise weighting, the implicit sign and accumulation.
 *
 * For applications providing their own parameter derivatives, the lower-level
 * pullback returns loss gradients on whitened factor residuals, called factor
 * residual cotangents. The application combines these with derivatives of its
 * factor residuals with respect to its parameters. pushforward predicts how a
 * change in factor residuals changes the estimates. Queries reuse the same
 * Bayes-tree factorization.
 *
 * Factor residual quantities use Errors, with one whitened vector per original
 * factor slot. Null/inactive slots have empty vectors. State cotangents use the
 * same local tangent bases as the factor residual Jacobians; omitted known keys
 * mean zero. Results contain every active state key. Unknown keys, invalid
 * dimensions and nonfinite inputs are errors. Finite, positive-definite
 * Gaussian noise is supported; robust losses, hard constraints and Hessian-only
 * factors are not.
 *
 * Construction stores independent whitened factor residual Jacobians. A freshly
 * eliminated tree is retained directly and is independent of caller objects.
 * An externally supplied tree uses native copy semantics: topology is copied,
 * while conditional and noise-model pointers are shared. Keep those shared
 * objects unchanged while using Gradients; their lifetime extends beyond the
 * source tree's lifetime. Copies of Gradients share their stored factors and
 * tree under this precondition. Nonlinear construction also retains the source
 * factors by shared ownership and copies solution for parameter VJP calls.
 * Keep those factors, their parameters and their noise models unchanged while
 * using parameterGradient. Const queries use separate workspaces; concurrent
 * parameterGradient calls additionally require thread-safe factor callbacks.
 * Rebuild after changing states, parameters, factors, active set or noise.
 *
 * A parameter VJP holds the solved states and supplied factor residual
 * cotangent fixed while differentiating the unwhitened factor residual with
 * respect to the requested parameter. Add any direct parameter derivative of
 * the loss separately to parameterGradient's result. Diagonal and isotropic
 * noise models that carry the same ParameterTag also contribute the
 * Gauss-Newton term on σ: each residual row i adds ν_w,i * (-e_i / σ_i²).
 * Isotropic models return one shared scalar (the sum). Full Gaussian and
 * robust noise parameters are not differentiated.
 *
 * \par Advanced differentiation model
 * These derivatives use a Gauss-Newton approximation. Let J be the whitened
 * factor residual Jacobian in tangent coordinates at the supplied solution, and
 * H = J.transpose() * J. Queries apply H inverse using two sparse Bayes-tree
 * triangular solves; they never assemble a global J, H, or inverse. The caller
 * owns optimization and must establish stationarity. Both the state Hessian
 * and the mixed state/parameter derivative use their Gauss-Newton terms.
 * The omitted second-derivative terms, weighted by factor residuals, vanish
 * when factor residuals are zero or jointly affine. Convergence alone does not
 * make them vanish. No damping or pseudoinverse fallback is applied.
 */
class GTSAM_EXPORT Gradients {
 public:
  /// CHOLESKY is faster; QR avoids forming local normal equations.
  enum Factorization { CHOLESKY, QR };

  /**
   * Construct from factor residuals exposed by NoiseModelFactor at solution,
   * using COLAMD. Obtains unwhitenedError and its Jacobian blocks, then creates
   * and retains whitened JacobianFactors. Factor-specific reduced linearize
   * overrides are not used. Factors without explicit factor residual Jacobians
   * are unsupported. The caller must ensure this squared-error model matches
   * the optimized objective if a factor overrides error(). Extra Values not
   * used by active factors do not create state variables.
   */
  Gradients(const NonlinearFactorGraph& graph, const Values& solution,
            Factorization factorization = CHOLESKY);

  /**
   * Construct from nonlinear factor residuals with each active key in ordering
   * once.
   */
  Gradients(const NonlinearFactorGraph& graph, const Values& solution,
            const Ordering& ordering, Factorization factorization = CHOLESKY);

  /**
   * Construct from independent copies of explicit JacobianFactor rows, using
   * COLAMD. Their whitened rows define the factor residual coordinates; b does
   * not affect derivative queries. JacobianFactor subclasses are supported
   * through their stored factor residual rows. Projected rows define projected
   * factor residual coordinates, not necessarily the original nonlinear factor
   * residuals. HessianFactors are rejected.
   */
  Gradients(const GaussianFactorGraph& graph,
            Factorization factorization = CHOLESKY);

  /**
   * Construct from Gaussian factor residuals with each active key in ordering
   * once.
   */
  Gradients(const GaussianFactorGraph& graph, const Ordering& ordering,
            Factorization factorization = CHOLESKY);

  /**
   * Reuse a tree and the explicit factor residual graph it factors, without
   * elimination. Copies and whitens the factor residual rows. Native tree
   * copying creates new topology but shares conditional and noise-model
   * pointers. The source tree may be destroyed; shared numerical data must
   * remain unchanged during use. The caller must supply a valid nonsingular
   * tree with matching keys and tangent dimensions that factors the same
   * undamped J.transpose() * J. Compatibility checks cannot establish numerical
   * agreement or detect a stale partially relinearized iSAM2 tree. Conditional
   * right-hand sides are ignored.
   */
  Gradients(const GaussianFactorGraph& linearizedGraph,
            const GaussianBayesTree& bayesTree);

  /** Print the state-block, factor-slot and factor residual counts. */
  void print(const std::string& prefix = "Gradients: ") const;

  /**
   * Return the intermediate vector lambda satisfying H * lambda =
   * stateCotangents. parameterGradient and pullback use this state adjoint
   * internally.
   */
  VectorValues adjoint(const VectorValues& stateCotangents) const;

  /**
   * Return the accumulated gradient through the solved states for parameter.
   * Each active NoiseModelFactor receives this tag and a cotangent in
   * unwhitened factor residual coordinates through its parameterVjp method.
   * No parameter registration or factor selection is required. Missing VJPs
   * and std::nullopt contributions mean no dependency; if no factor contributes,
   * the result is std::nullopt. A supported parameter with zero gradient returns
   * an explicit zero vector. Returned blocks must be finite, nonempty and have
   * matching dimensions, otherwise the query throws std::invalid_argument.
   *
   * Requires construction from a nonlinear graph and solution; Gaussian-only
   * construction throws std::logic_error because it has no parameter VJPs.
   * Tagged diagonal or isotropic noise models add their σ VJP. This applies
   * the same Gauss-Newton approximation as pullback and excludes any direct
   * parameter dependence of the outer loss.
   * Each call reuses the factorization and performs one adjoint calculation.
   */
  std::optional<Vector> parameterGradient(
      const VectorValues& stateCotangents, const ParameterTag& parameter) const;

  /**
   * Propagate whitened factor residual changes to a change in the solved states.
   * Supply one finite vector per factor slot, with the factor's row dimension.
   * Null/inactive slots require empty vectors; incorrect blocks are errors.
   * The result is -H^{-1} J.transpose() * factorResidualDirection, with J held
   * fixed.
   */
  VectorValues pushforward(const Errors& factorResidualDirection) const;

  /**
   * Propagate state loss gradients to one whitened factor residual vector per
   * slot. stateCotangents holds the loss derivatives in the solved-state
   * tangent bases. The result is -J H^{-1} * stateCotangents and includes the
   * implicit minus sign. For raw errors e with r = W e, a parameter derivative
   * through raw errors instead receives W.transpose() times each returned
   * factor residual cotangent.
   */
  Errors pullback(const VectorValues& stateCotangents) const;

 private:
  GaussianFactorGraph graph_;
  VectorValues::Dims dimensions_;
  std::shared_ptr<const GaussianBayesTree> bayesTree_;
  NonlinearFactorGraph nonlinearGraph_;
  std::optional<Values> solution_;

  /** Retain owned whitened Jacobian factors and eliminate them once. */
  Gradients(GaussianFactorGraph&& graph, Factorization factorization,
            const std::optional<Ordering>& ordering);

  /** Check consistency of the state block dimensions. */
  void initializeDimensions();

  /** Eliminate factors with state variables using an optional ordering. */
  void computeBayesTree(const std::optional<Ordering>& ordering,
                       Factorization factorization);
};

}  // namespace gtsam
