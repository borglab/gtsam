/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    AugmentedLagrangianOptimizer.cpp
 * @brief   Augmented Lagrangian method for nonlinear constrained optimization.
 * @author  Yetong Zhang
 * @author  Frank Dellaert (codex assisted)
 * @date    Aug 3, 2024
 */

#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/internal/AugmentedLagrangianEngine.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using std::cout, std::endl;

namespace gtsam {
namespace {

/**
 * A factor that adds a constant bias term to an original factor's unwhitened
 * error. The augmented Lagrangian uses this to represent the equality term
 *
 *   lambda^T h(x) + (rho / 2) ||h(x)||^2
 *     = (rho / 2) ||h(x) + lambda / rho||^2
 *       - ||lambda||^2 / (2 rho).
 *
 * The noise model remains attached to both the original factor and this
 * wrapper. Only this wrapper's noise model is applied to the biased residual;
 * evaluating originalFactor_->unwhitenedError does not apply the original
 * factor's noise model a second time.
 */
class BiasedFactor : public NoiseModelFactor {
 protected:
  using Base = NoiseModelFactor;
  using This = BiasedFactor;

  // Original factor whose raw error and Jacobians are reused.
  Base::shared_ptr originalFactor_;
  Vector sigmas_;
  Vector bias_;
  double penalty_ = 0.0;

 public:
  using shared_ptr = std::shared_ptr<This>;

  /// Default constructor for I/O only.
  BiasedFactor() = default;

  /**
   * Construct a biased view of an existing factor.
   *
   * @param originalFactor Original factor on x.
   * @param sigmas Constraint sigmas before penalty scaling.
   * @param lambda Multiplier in whitened constraint coordinates.
   * @param penalty Direct penalty rho.
   */
  BiasedFactor(const Base::shared_ptr& originalFactor, const Vector& sigmas,
               const Vector& lambda, double penalty)
      : Base(originalFactor->noiseModel(), originalFactor->keys()),
        originalFactor_(originalFactor),
        sigmas_(sigmas) {
    setParameters(lambda, penalty);
  }

  /// Update numerical parameters while preserving factor structure.
  void setParameters(const Vector& lambda, double penalty) {
    bias_ = (lambda / penalty).cwiseProduct(sigmas_);
    if (penalty != penalty_) {
      noiseModel_ = noiseModel::Diagonal::Sigmas(sigmas_ / std::sqrt(penalty));
      penalty_ = penalty;
    }
  }

  /**
   * Error function *without* the noise model, conventionally z-h(x). Override
   * this method to finish implementing an N-way factor. If the optional
   * argument is specified, it also computes the derivatives in `jacobians`.
   * Here the result is the original factor's unwhitened error plus the fixed
   * bias, and the Jacobians are unchanged because the bias is constant.
   */
  Vector unwhitenedError(
      const Values& values,
      gtsam::OptionalMatrixVecType jacobians = nullptr) const override {
    return originalFactor_->unwhitenedError(values, jacobians) + bias_;
  }

  /// Print the biased factor.
  void print(const std::string& label, const KeyFormatter& keyFormatter =
                                           DefaultKeyFormatter) const override {
    cout << label << "BiasedFactor " << bias_.transpose()
         << " version of:" << endl;
    originalFactor_->print(label, keyFormatter);
  }

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new This(*this)));
  }
};

/**
 * Least-squares factor for a scalar Powell--Hestenes--Rockafellar (PHR)
 * inequality augmented Lagrangian. For a whitened inequality g(x) <= 0,
 * nonnegative multiplier lambda, and direct penalty rho > 0, the PHR term is
 *
 *   [max(0, lambda + rho g(x))^2 - lambda^2] / (2 rho).
 *
 * The second term is constant in x, so LM can minimize the equivalent residual
 *
 *   r(x) = max(0, lambda + rho g(x)) / sqrt(rho).
 *
 * See Nocedal and Wright, Numerical Optimization, 2nd ed., Chapter 17, for the
 * PHR inequality augmented Lagrangian and projected multiplier update.
 */
class PhrInequalityFactor : public NoiseModelFactor {
 protected:
  using Base = NoiseModelFactor;
  using This = PhrInequalityFactor;

  NonlinearInequalityConstraint::shared_ptr constraint_;
  double lambda_;
  double penalty_;

 public:
  /// Construct a scalar PHR factor at fixed multiplier and direct penalty.
  PhrInequalityFactor(
      const NonlinearInequalityConstraint::shared_ptr& constraint,
      double lambda, double penalty)
      : Base(noiseModel::Unit::Create(1), constraint->keys()),
        constraint_(constraint),
        lambda_(lambda),
        penalty_(penalty) {
    if (constraint_->dim() != 1) {
      throw std::invalid_argument(
          "AugmentedLagrangianOptimizer supports scalar inequalities only");
    }
  }

  /// Update numerical parameters while preserving factor structure.
  void setParameters(double lambda, double penalty) {
    lambda_ = lambda;
    penalty_ = penalty;
  }

  /// Evaluate the exact shifted-ramp residual and its piecewise Jacobians.
  Vector unwhitenedError(
      const Values& values,
      gtsam::OptionalMatrixVecType jacobians = nullptr) const override {
    Vector expression;
    if (jacobians) {
      // Convert both g and dg/dx to whitened constraint coordinates so sigma
      // provides the same fixed scaling in the merit function and diagnostics.
      expression = constraint_->unwhitenedExpr(values, jacobians);
      constraint_->noiseModel()->WhitenSystem(*jacobians, expression);
    } else {
      expression = constraint_->whitenedExpr(values);
    }

    // On the inactive branch lambda + rho*g <= 0, max(0, .) and its selected
    // boundary derivative are zero.
    const auto [residual, scale] =
        internal::AlmPhrResidual(expression(0), lambda_, penalty_);
    if (jacobians)
      for (Matrix& jacobian : *jacobians) jacobian *= scale;
    return Vector1(residual);
  }

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new This(*this)));
  }
};

}  // namespace

namespace internal {

/// Original nonlinear functions; ALM penalty construction belongs to the
/// engine.
class NonlinearAlmModel {
 public:
  using Primal = Values;

  /// Cache constant constraint scales and validate supported inequality
  /// dimensions.
  explicit NonlinearAlmModel(const ConstrainedOptProblem& problem)
      : problem_(problem) {
    for (const auto& equality : problem_.eConstraints()) {
      equalitySigmas_.push_back(equality->sigmas());
      validateSigmas(equalitySigmas_.back());
    }
    for (const auto& inequality : problem_.iConstraints()) {
      if (inequality->dim() != 1)
        throw std::invalid_argument(
            "ALM supports scalar nonlinear inequalities only");
      inequalitySigmas_.push_back(inequality->sigmas());
      validateSigmas(inequalitySigmas_.back());
    }
  }

  /// Preserve original equality block boundaries.
  std::vector<size_t> equalityDimensions() const {
    std::vector<size_t> dims;
    for (const Vector& sigma : equalitySigmas_) dims.push_back(sigma.size());
    return dims;
  }
  /// Number of scalar PHR constraints.
  size_t inequalityCount() const { return inequalitySigmas_.size(); }
  /// No objective factors are explicitly cloned by this adapter.
  size_t objectiveFactorCopies() const { return 0; }
  /// Nonlinear structure preparation is counted by symbolic workspace creation.
  double preparationSeconds() const { return 0; }
  /// Nonlinear derivative matrices must be refreshed at new primal points.
  bool fixedCoefficients() const { return false; }
  /// Use manifold tangent dimensions for the shared linear algebra.
  VectorValues zeroVectors(const Values& values) const {
    return values.zeroVectors();
  }
  /// Retract in the original variable types, including manifolds and matrices.
  Values retract(const Values& values, const VectorValues& delta) const {
    return values.retract(delta);
  }

  /// Evaluate true functions; original cost and constraint Jacobians are
  /// shared.
  AlmPoint evaluate(const Values& values, bool derivatives,
                    AugmentedLagrangianStatistics* statistics) const {
    AlmPoint point;
    point.cost = problem_.costs().error(values);
    if (derivatives) {
      point.costs = *problem_.costs().linearize(values);
      ++statistics->modelLinearizations;
    }
    for (size_t i = 0; i < equalitySigmas_.size(); ++i) {
      const auto& constraint = problem_.eConstraints()[i];
      std::vector<Matrix> jacobians(constraint->keys().size());
      Vector residual = constraint->unwhitenedError(
          values, derivatives ? &jacobians : nullptr);
      residual.array() /= equalitySigmas_[i].array();
      point.equalities.push_back(residual);
      if (derivatives)
        point.equalityRows.push_back(
            row(constraint->keys(), jacobians, equalitySigmas_[i]));
    }
    for (size_t i = 0; i < inequalitySigmas_.size(); ++i) {
      const auto& constraint = problem_.iConstraints()[i];
      std::vector<Matrix> jacobians(constraint->keys().size());
      Vector residual = constraint->unwhitenedExpr(
          values, derivatives ? &jacobians : nullptr);
      point.inequalities.push_back(residual(0) / inequalitySigmas_[i](0));
      if (derivatives)
        point.inequalityRows.push_back(
            row(constraint->keys(), jacobians, inequalitySigmas_[i]));
    }
    point.derivatives = derivatives;
    return point;
  }

  /// Centered Gaussian cost factors already encode the original cost gradient.
  void addCostGradient(const Values&, const AlmPoint& point,
                       VectorValues* gradient) const {
    AlmAddCenteredGradient(point.costs, gradient);
  }
  /// Return the same objective linearization used by the stationarity test.
  const GaussianFactorGraph& linearizeCosts(const Values&,
                                            const AlmPoint& point) const {
    return point.costs;
  }

 private:
  const ConstrainedOptProblem& problem_;
  std::vector<Vector> equalitySigmas_, inequalitySigmas_;

  /// Reject undefined penalty scaling instead of silently adding a floor.
  static void validateSigmas(const Vector& sigmas) {
    if (!sigmas.allFinite() || (sigmas.array() <= 0).any())
      throw std::invalid_argument(
          "ALM constraint scales must be finite and positive");
  }
  /// Whiten original derivatives once; the engine supplies all penalty scaling.
  static JacobianFactor::shared_ptr row(const KeyVector& keys,
                                        std::vector<Matrix>& jacobians,
                                        const Vector& sigmas) {
    std::vector<std::pair<Key, Matrix>> terms;
    for (size_t j = 0; j < keys.size(); ++j) {
      jacobians[j].array().colwise() /= sigmas.array();
      terms.emplace_back(keys[j], std::move(jacobians[j]));
    }
    return std::make_shared<JacobianFactor>(terms, Vector::Zero(sigmas.size()));
  }
};

}  // namespace internal

struct AugmentedLagrangianOptimizer::Workspace {
  internal::AugmentedLagrangianEngine<internal::NonlinearAlmModel, State>
      engine;
  Workspace(const ConstrainedOptProblem& problem, const Params& params)
      : engine(internal::NonlinearAlmModel(problem), params) {}
};

void AugmentedLagrangianState::initializeLagrangeMultipliers(
    const ConstrainedOptProblem& problem) {
  std::vector<size_t> dims;
  for (const auto& constraint : problem.eConstraints())
    dims.push_back(constraint->dim());
  internal::InitializeAlmMultipliers(dims, problem.iConstraints().size(), this);
}

std::tuple<AugmentedLagrangianOptimizer::State, double, double>
AugmentedLagrangianOptimizer::iterate(const State& state, double muEq,
                                      double muIneq) const {
  std::unique_ptr<Workspace> workspace;
  return iterate(state, muEq, muIneq, workspace);
}

std::tuple<AugmentedLagrangianOptimizer::State, double, double>
AugmentedLagrangianOptimizer::iterate(
    const State& state, double muEq, double muIneq,
    std::unique_ptr<Workspace>& workspace) const {
  validateConfiguration();
  if (!workspace) workspace = std::make_unique<Workspace>(problem_, *p_);
  return workspace->engine.iterate(state, muEq, muIneq);
}

Values AugmentedLagrangianOptimizer::optimize() const {
  return optimizeWithState().values;
}

AugmentedLagrangianOptimizer::State
AugmentedLagrangianOptimizer::optimizeWithState() const {
  validateConfiguration();
  Workspace workspace(problem_, *p_);
  State state;
  state.values = initialValues_;
  return workspace.engine.optimize(std::move(state), &progress_);
}

NonlinearFactorGraph AugmentedLagrangianOptimizer::augmentedLagrangianFunction(
    const State& state, double /*epsilon*/) const {
  validateConfiguration();
  if (state.muEq <= 0.0 || state.muIneq <= 0.0) {
    throw std::invalid_argument("ALM direct penalties must be positive");
  }
  if (state.lambdaEq.size() != problem_.eConstraints().size() ||
      state.lambdaIneq.size() != problem_.iConstraints().size()) {
    throw std::invalid_argument(
        "ALM multiplier dimensions do not match constraints");
  }

  // Initialize the merit graph with the original least-squares costs.
  NonlinearFactorGraph graph = problem_.costs();

  // Add equality factors. Each graph error is
  // (rho/2)||h+lambda/rho||^2
  //   = lambda^T h + (rho/2)||h||^2 + ||lambda||^2/(2 rho).
  // The final term is independent of x and can be omitted during LM.
  const auto& equalities = problem_.eConstraints();
  for (size_t i = 0; i < equalities.size(); ++i) {
    const auto& constraint = equalities.at(i);
    graph.emplace_shared<BiasedFactor>(constraint->penaltyFactor(1.0),
                                       constraint->sigmas(),
                                       state.lambdaEq.at(i), state.muEq);
  }

  // Add exact PHR factors for scalar g<=0. Each graph error is
  // max(0,lambda+rho*g)^2/(2 rho), which differs from the mathematical PHR
  // term only by the x-independent constant lambda^2/(2 rho).
  const auto& inequalities = problem_.iConstraints();
  for (size_t i = 0; i < inequalities.size(); ++i) {
    graph.emplace_shared<PhrInequalityFactor>(
        inequalities.at(i), state.lambdaIneq.at(i), state.muIneq);
  }

  return graph;
}

void AugmentedLagrangianOptimizer::updateLagrangeMultiplier(
    const State& subproblemState, State* solvedState) const {
  internal::NonlinearAlmModel model(problem_);
  AugmentedLagrangianStatistics statistics;
  const auto point = model.evaluate(solvedState->values, false, &statistics);
  solvedState->lambdaEq = subproblemState.lambdaEq;
  solvedState->lambdaIneq = subproblemState.lambdaIneq;
  solvedState->muEq = subproblemState.muEq;
  solvedState->muIneq = subproblemState.muIneq;
  internal::UpdateAggressiveAlmMultipliers(*p_, point, solvedState);
}

std::pair<double, double> AugmentedLagrangianOptimizer::updatePenaltyParameter(
    const State& previousState, const State& solvedState) const {
  return internal::UpdateAggressiveAlmPenalty(*p_, previousState, solvedState);
}

void AugmentedLagrangianOptimizer::validateConfiguration() const {
  if (!p_)
    throw std::invalid_argument("AugmentedLagrangianParams must not be null");
  internal::ValidateAlmParams(*p_);
  internal::NonlinearAlmModel model(problem_);
}

void AugmentedLagrangianOptimizer::logInitialState(const State& state) const {
  internal::RecordAlmState(*p_, state, &progress_);
}
void AugmentedLagrangianOptimizer::logIteration(const State& state) const {
  internal::RecordAlmState(*p_, state, &progress_);
}

}  // namespace gtsam
