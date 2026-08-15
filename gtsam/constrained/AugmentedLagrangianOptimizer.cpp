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
#include <gtsam/linear/GaussianFactorGraph.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <stdexcept>

using std::cout, std::endl, std::setprecision, std::setw;

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
  Vector bias_;

 public:
  using shared_ptr = std::shared_ptr<This>;

  /// Default constructor for I/O only.
  BiasedFactor() = default;

  /**
   * Construct a biased view of an existing factor.
   *
   * @param originalFactor Original factor on x.
   * @param bias Constant added to its unwhitened error.
   */
  BiasedFactor(const Base::shared_ptr& originalFactor, const Vector& bias)
      : Base(originalFactor->noiseModel(), originalFactor->keys()),
        originalFactor_(originalFactor),
        bias_(bias) {}

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
    const double shifted = lambda_ + penalty_ * expression(0);
    if (shifted <= 0.0) {
      if (jacobians) {
        for (Matrix& jacobian : *jacobians) {
          jacobian.setZero();
        }
      }
      return Vector1::Zero();
    }

    // On the active branch r=(lambda+rho*g)/sqrt(rho), hence
    // dr/dx=sqrt(rho)*dg/dx.
    const double sqrtPenalty = std::sqrt(penalty_);
    if (jacobians) {
      for (Matrix& jacobian : *jacobians) {
        jacobian *= sqrtPenalty;
      }
    }
    return Vector1(shifted / sqrtPenalty);
  }

  /// Return a deep copy.
  NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<NonlinearFactor>(
        NonlinearFactor::shared_ptr(new This(*this)));
  }
};

struct Diagnostics {
  double generalizedConstraintViolation = 0.0;
  double primalInequalityViolation = 0.0;
  double complementarity = 0.0;
};

/* ************************************************************************* */
double InfinityNorm(const Vector& vector) {
  return vector.size() == 0 ? 0.0 : vector.cwiseAbs().maxCoeff();
}

/* ************************************************************************* */
double AugmentedLagrangianStationarity(const NonlinearFactorGraph& graph,
                                       const Values& values) {
  const VectorValues gradient = graph.linearize(values)->gradientAtZero();
  double infinityNorm = 0.0;
  for (const auto& keyGradient : gradient) {
    infinityNorm = std::max(infinityNorm, InfinityNorm(keyGradient.second));
  }
  return infinityNorm;
}

/* ************************************************************************* */
Diagnostics EvaluateDiagnostics(const ConstrainedOptProblem& problem,
                                const Values& values,
                                const AugmentedLagrangianState& subproblem) {
  Diagnostics diagnostics;

  // Equality feasibility contributes ||h(x)||_inf to theta.
  for (const auto& constraint : problem.eConstraints()) {
    diagnostics.generalizedConstraintViolation =
        std::max(diagnostics.generalizedConstraintViolation,
                 InfinityNorm(constraint->whitenedError(values)));
  }

  const auto& inequalities = problem.iConstraints();
  for (size_t i = 0; i < inequalities.size(); ++i) {
    const double expression = inequalities.at(i)->whitenedExpr(values)(0);
    const double lambda = subproblem.lambdaIneq.at(i);

    // q=max(g,-lambda/rho) makes the projected multiplier update
    // lambda^+=max(0,lambda+rho*g)=lambda+rho*q. Thus q=0 encodes primal
    // feasibility and complementarity, including inactive inequalities.
    const double projectedResidual =
        std::max(expression, -lambda / subproblem.muIneq);
    const double projectedLambda =
        std::max(0.0, lambda + subproblem.muIneq * expression);

    // BCL accepts a multiplier update using
    // theta=max(||h||_inf,||q||_inf), not merely max(g,0).
    diagnostics.generalizedConstraintViolation =
        std::max(diagnostics.generalizedConstraintViolation,
                 std::abs(projectedResidual));
    // Report primal violation and complementarity separately for diagnosis.
    diagnostics.primalInequalityViolation = std::max(
        diagnostics.primalInequalityViolation, std::max(0.0, expression));
    diagnostics.complementarity = std::max(
        diagnostics.complementarity, std::abs(projectedLambda * expression));
  }

  return diagnostics;
}

/* ************************************************************************* */
void InitializeBclSchedule(const AugmentedLagrangianParams& params,
                           double penalty, AugmentedLagrangianState* state) {
  // Conn--Gould--Toint use an inverse penalty mu. With rho=1/mu, their
  // alpha=min(mu,gamma_1) becomes alpha=min(1/rho,gamma_1).
  state->bclAlpha = std::min(1.0 / penalty, params.bclGamma1);
  state->bclOmega =
      params.bclOmega0 * std::pow(state->bclAlpha, params.bclAlphaOmega);
  state->bclEta =
      params.bclEta0 * std::pow(state->bclAlpha, params.bclAlphaEta);
}

/* ************************************************************************* */
bool AggressiveConverged(const AugmentedLagrangianState& state,
                         const AugmentedLagrangianState& previousState,
                         const AugmentedLagrangianParams& params) {
  if (state.violation() < params.absoluteViolationTolerance &&
      state.cost < params.absoluteCostTolerance) {
    return true;
  }
  return std::abs(state.violation() - previousState.violation()) <
             params.relativeViolationTolerance &&
         std::abs(state.cost - previousState.cost) <
             params.relativeCostTolerance;
}

/* ************************************************************************* */
AugmentedLagrangianUpdateType CombinedUpdateType(bool multiplierUpdated,
                                                 bool penaltyUpdated) {
  if (multiplierUpdated && penaltyUpdated) {
    return AugmentedLagrangianUpdateType::MultiplierAndPenalty;
  }
  if (multiplierUpdated) {
    return AugmentedLagrangianUpdateType::Multiplier;
  }
  if (penaltyUpdated) {
    return AugmentedLagrangianUpdateType::Penalty;
  }
  return AugmentedLagrangianUpdateType::None;
}

}  // namespace

/* ************************************************************************* */
void AugmentedLagrangianState::initializeLagrangeMultipliers(
    const ConstrainedOptProblem& problem) {
  lambdaEq.clear();
  lambdaEq.reserve(problem.eConstraints().size());
  for (const auto& constraint : problem.eConstraints()) {
    lambdaEq.push_back(Vector::Zero(constraint->dim()));
  }
  lambdaIneq.assign(problem.iConstraints().size(), 0.0);
}

/* ************************************************************************* */
std::tuple<AugmentedLagrangianOptimizer::State, double, double>
AugmentedLagrangianOptimizer::iterate(const State& state, double muEq,
                                      double muIneq) const {
  // Validate the fixed-parameter augmented-Lagrangian subproblem.
  validateConfiguration();
  if (muEq <= 0.0 || muIneq <= 0.0) {
    throw std::invalid_argument("ALM direct penalties must be positive");
  }
  if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL &&
      muEq != muIneq) {
    // Algorithm 1 of Conn--Gould--Toint has one inverse penalty mu_k for the
    // complete constraint vector c(x). In direct notation rho_k=1/mu_k, the
    // same rho must therefore weight h and the PHR inequality terms. Separate
    // penalties would require a new vector-valued schedule and a new definition
    // of alpha_k, so it would no longer be the paper's BCL update policy.
    // Constraint sigmas still provide fixed relative block scaling. The
    // Aggressive policy remains free to use separate equality/inequality rho.
    throw std::invalid_argument(
        "BCL Algorithm 1 requires muEq == muIneq (one common direct penalty "
        "rho)");
  }

  // Hold multipliers and penalties fixed while solving the inner problem.
  State subproblemState = state;
  subproblemState.muEq = muEq;
  subproblemState.muIneq = muIneq;
  if (subproblemState.lambdaEq.size() != problem_.eConstraints().size() ||
      subproblemState.lambdaIneq.size() != problem_.iConstraints().size()) {
    if (subproblemState.lambdaEq.empty() &&
        subproblemState.lambdaIneq.empty()) {
      subproblemState.initializeLagrangeMultipliers(problem_);
    } else {
      throw std::invalid_argument(
          "ALM multiplier dimensions do not match constraints");
    }
  }
  if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL &&
      (subproblemState.bclOmega <= 0.0 || subproblemState.bclEta <= 0.0)) {
    // Initialize omega_k and eta_k when iterate() is called directly.
    InitializeBclSchedule(*p_, muEq, &subproblemState);
  }

  const auto start = std::chrono::steady_clock::now();

  // Construct the merit function at (lambda_k,rho_k), then run unconstrained
  // LM from x_k. Multiplier updates happen only after x_{k+1} is available.
  const NonlinearFactorGraph augmentedLagrangian =
      augmentedLagrangianFunction(subproblemState);
  const SharedOptimizer optimizer =
      createUnconstrainedOptimizer(augmentedLagrangian, state.values);

  double stationarity =
      AugmentedLagrangianStationarity(augmentedLagrangian, optimizer->values());
  if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL) {
    // The paper stops its projected inner solve when the projected gradient is
    // at most omega_k. Because this implementation substitutes unconstrained
    // LM, it uses s_k=||grad_x L_rho(x,lambda_k)||_inf <= omega_k.
    while (stationarity > subproblemState.bclOmega &&
           optimizer->iterations() < p_->lmParams.maxIterations) {
      const size_t previousIterations = optimizer->iterations();
      optimizer->iterate();
      stationarity = AugmentedLagrangianStationarity(augmentedLagrangian,
                                                     optimizer->values());
      if (optimizer->iterations() == previousIterations) {
        break;
      }
    }
  } else {
    // Preserve the historical Aggressive behavior of letting LM use its own
    // convergence checks and iteration cap for every outer subproblem.
    optimizer->optimize();
    stationarity = AugmentedLagrangianStationarity(augmentedLagrangian,
                                                   optimizer->values());
  }

  // Evaluate objective and all policy diagnostics at the returned point
  // x_{k+1}; evaluating at x_k was the old sequencing error.
  State solvedState = subproblemState;
  solvedState.iteration = state.iteration + 1;
  solvedState.setValues(optimizer->values(), problem_);
  solvedState.unconstrainedIterations = optimizer->iterations();
  solvedState.totalUnconstrainedIterations =
      state.totalUnconstrainedIterations + solvedState.unconstrainedIterations;
  solvedState.augmentedLagrangianStationarity = stationarity;
  const Diagnostics diagnostics =
      EvaluateDiagnostics(problem_, solvedState.values, subproblemState);
  solvedState.generalizedConstraintViolation =
      diagnostics.generalizedConstraintViolation;
  solvedState.primalInequalityViolation = diagnostics.primalInequalityViolation;
  solvedState.complementarity = diagnostics.complementarity;
  solvedState.updateType = AugmentedLagrangianUpdateType::None;
  solvedState.converged = false;

  double nextMuEq = muEq;
  double nextMuIneq = muIneq;
  if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::Aggressive) {
    // Aggressive always performs projected dual ascent at x_{k+1}, then grows
    // each penalty independently if its violation did not decrease enough.
    solvedState.innerConverged = true;
    updateLagrangeMultiplier(subproblemState, &solvedState);
    std::tie(nextMuEq, nextMuIneq) = updatePenaltyParameter(state, solvedState);
    const bool multiplierUpdated =
        !problem_.eConstraints().empty() || !problem_.iConstraints().empty();
    const bool penaltyUpdated = nextMuEq != muEq || nextMuIneq != muIneq;
    solvedState.updateType =
        CombinedUpdateType(multiplierUpdated, penaltyUpdated);
  } else {
    solvedState.innerConverged = stationarity <= subproblemState.bclOmega;

    // If LM exhausts its cap before s_k<=omega_k, Algorithm 1's inner condition
    // was not met: keep both lambda and rho unchanged and return the best
    // point.
    if (solvedState.innerConverged) {
      if (solvedState.generalizedConstraintViolation <=
          subproblemState.bclEta) {
        // Successful BCL iteration (theta_k<=eta_k):
        //   lambda_h^+ = lambda_h + rho*h,
        //   lambda_g^+ = max(0,lambda_g + rho*g),
        // while rho stays fixed.
        for (size_t i = 0; i < problem_.eConstraints().size(); ++i) {
          solvedState.lambdaEq.at(i) +=
              muEq *
              problem_.eConstraints().at(i)->whitenedError(solvedState.values);
        }
        for (size_t i = 0; i < problem_.iConstraints().size(); ++i) {
          const double expression = problem_.iConstraints().at(i)->whitenedExpr(
              solvedState.values)(0);
          solvedState.lambdaIneq.at(i) =
              std::max(0.0, solvedState.lambdaIneq.at(i) + muEq * expression);
        }
        solvedState.updateType = AugmentedLagrangianUpdateType::Multiplier;

        // Tighten the next inner and feasibility targets using the unchanged
        // alpha_k, as in the accepted branch of Algorithm 1.
        solvedState.bclOmega =
            subproblemState.bclOmega *
            std::pow(subproblemState.bclAlpha, p_->bclBetaOmega);
        solvedState.bclEta = subproblemState.bclEta *
                             std::pow(subproblemState.bclAlpha, p_->bclBetaEta);
      } else {
        // Unsuccessful BCL iteration (theta_k>eta_k): hold lambda fixed,
        // increase direct rho by 1/tau, recompute alpha, and reset omega/eta.
        nextMuEq = muEq * p_->bclPenaltyIncreaseRate;
        nextMuIneq = nextMuEq;
        solvedState.updateType = AugmentedLagrangianUpdateType::Penalty;
        InitializeBclSchedule(*p_, nextMuEq, &solvedState);
      }
    }
  }

  solvedState.time =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  return {solvedState, nextMuEq, nextMuIneq};
}

/* ************************************************************************* */
Values AugmentedLagrangianOptimizer::optimize() const {
  validateConfiguration();
  progress_.clear();

  // Construct the initial primal-dual state with zero multipliers.
  State state(0, initialValues_, problem_);
  state.initializeLagrangeMultipliers(problem_);

  double muEq = p_->initialMuEq;
  double muIneq = p_->initialMuIneq;
  if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL) {
    // BCL uses one common direct penalty and the paper-derived initial
    // stationarity/feasibility schedule.
    muEq = p_->bclInitialPenalty;
    muIneq = p_->bclInitialPenalty;
    InitializeBclSchedule(*p_, muEq, &state);
  }
  state.muEq = muEq;
  state.muIneq = muIneq;
  logInitialState(state);

  // Solve fixed-parameter subproblems and apply the selected outer policy.
  while (true) {
    const State previousState = state;
    std::tie(state, muEq, muIneq) = iterate(previousState, muEq, muIneq);

    if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL) {
      state.converged = state.innerConverged &&
                        state.augmentedLagrangianStationarity <=
                            p_->absoluteStationarityTolerance &&
                        state.generalizedConstraintViolation <=
                            p_->absoluteViolationTolerance;
    } else {
      state.converged = AggressiveConverged(state, previousState, *p_);
    }
    logIteration(state);

    if (state.converged || !state.innerConverged ||
        state.iteration >= p_->maxIterations) {
      break;
    }
  }

  return state.values;
}

/* ************************************************************************* */
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
    Vector bias = state.lambdaEq.at(i) / state.muEq;
    bias = bias.cwiseProduct(constraint->sigmas());
    graph.emplace_shared<BiasedFactor>(constraint->penaltyFactor(state.muEq),
                                       bias);
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

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::updateLagrangeMultiplier(
    const State& subproblemState, State* solvedState) const {
  // Perform dual ascent on equality multipliers using h(x_{k+1}). Constraint
  // violation is the gradient of the dual function with respect to lambda.
  const auto& equalities = problem_.eConstraints();
  solvedState->lambdaEq.resize(equalities.size());
  for (size_t i = 0; i < equalities.size(); ++i) {
    const Vector violation =
        equalities.at(i)->whitenedError(solvedState->values);
    const double stepSize = std::min(
        p_->maxDualStepSizeEq, subproblemState.muEq * p_->dualStepSizeFactorEq);
    solvedState->lambdaEq.at(i) =
        subproblemState.lambdaEq.at(i) + stepSize * violation;
  }

  // Perform projected dual ascent on inequality multipliers using g(x_{k+1})
  // and projection onto lambda>=0.
  const auto& inequalities = problem_.iConstraints();
  solvedState->lambdaIneq.resize(inequalities.size());
  for (size_t i = 0; i < inequalities.size(); ++i) {
    const double violation =
        inequalities.at(i)->whitenedExpr(solvedState->values)(0);
    const double stepSize =
        std::min(p_->maxDualStepSizeIneq,
                 subproblemState.muIneq * p_->dualStepSizeFactorIneq);
    solvedState->lambdaIneq.at(i) =
        std::max(0.0, subproblemState.lambdaIneq.at(i) + stepSize * violation);
  }
}

/* ************************************************************************* */
std::pair<double, double> AugmentedLagrangianOptimizer::updatePenaltyParameter(
    const State& previousState, const State& solvedState) const {
  // Aggressive keeps separate penalties and increases a block's rho when its
  // violation has not contracted by muIncreaseThreshold.
  double muEq = solvedState.muEq;
  if (!problem_.eConstraints().empty() &&
      solvedState.eqConstraintViolation >=
          p_->muIncreaseThreshold * previousState.eqConstraintViolation) {
    muEq *= p_->muEqIncreaseRate;
  }

  double muIneq = solvedState.muIneq;
  if (!problem_.iConstraints().empty() &&
      solvedState.ineqConstraintViolation >=
          p_->muIncreaseThreshold * previousState.ineqConstraintViolation) {
    muIneq *= p_->muIneqIncreaseRate;
  }
  return {muEq, muIneq};
}

/* ************************************************************************* */
ConstrainedOptimizer::SharedOptimizer
AugmentedLagrangianOptimizer::createUnconstrainedOptimizer(
    const NonlinearFactorGraph& graph, const Values& values) const {
  // TODO(yetong): make compatible with all NonlinearOptimizers.
  return std::make_shared<LevenbergMarquardtOptimizer>(graph, values,
                                                       p_->lmParams);
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::validateConfiguration() const {
  if (!p_) {
    throw std::invalid_argument("AugmentedLagrangianParams must not be null");
  }
  if (p_->ineqConstraintPenaltyFunction) {
    throw std::invalid_argument(
        "AugmentedLagrangianOptimizer requires exact PHR inequalities; custom "
        "smoothed inequality penalties are unsupported");
  }
  for (const auto& inequality : problem_.iConstraints()) {
    if (inequality->dim() != 1) {
      throw std::invalid_argument(
          "AugmentedLagrangianOptimizer supports scalar inequalities only");
    }
  }
  if (p_->initialMuEq <= 0.0 || p_->initialMuIneq <= 0.0 ||
      p_->muEqIncreaseRate <= 0.0 || p_->muIneqIncreaseRate <= 0.0 ||
      p_->absoluteStationarityTolerance < 0.0) {
    throw std::invalid_argument("Invalid Aggressive ALM parameters");
  }
  if (p_->bclInitialPenalty <= 0.0 || p_->bclPenaltyIncreaseRate <= 1.0 ||
      p_->bclOmega0 <= 0.0 || p_->bclEta0 <= 0.0 || p_->bclGamma1 <= 0.0 ||
      p_->bclAlphaOmega <= 0.0 || p_->bclBetaOmega <= 0.0 ||
      p_->bclAlphaEta <= 0.0 || p_->bclBetaEta <= 0.0) {
    throw std::invalid_argument("Invalid BCL ALM parameters");
  }
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::logInitialState(const State& state) const {
  if (p_->verbose) {
    // Log title line.
    cout << setw(8) << "Iter"
         << "|" << setw(10) << "rhoEq"
         << "|" << setw(10) << "rhoIneq"
         << "|" << setw(10) << "cost"
         << "|" << setw(10) << "vio_e"
         << "|" << setw(10) << "vio_i"
         << "|" << setw(10) << "station"
         << "|" << setw(10) << "theta"
         << "|" << setw(10) << "lm_iters"
         << "|" << endl;

    // Log initial value line.
    cout << setw(8) << state.iteration << "|" << setw(10) << state.muEq << "|"
         << setw(10) << state.muIneq << "|" << setw(10) << setprecision(4)
         << state.cost << "|" << setw(10) << state.eqConstraintViolation << "|"
         << setw(10) << state.ineqConstraintViolation << "|" << setw(10) << "-"
         << "|" << setw(10) << "-"
         << "|" << setw(10) << "-"
         << "|" << endl;
  }
  // Store state.
  if (p_->storeOptProgress) {
    progress_.emplace_back(state);
  }
}

/* ************************************************************************* */
void AugmentedLagrangianOptimizer::logIteration(const State& state) const {
  if (p_->verbose) {
    cout << setw(8) << state.iteration << "|" << setw(10) << state.muEq << "|"
         << setw(10) << state.muIneq << "|" << setw(10) << setprecision(4)
         << state.cost << "|" << setw(10) << state.eqConstraintViolation << "|"
         << setw(10) << state.ineqConstraintViolation << "|" << setw(10)
         << state.augmentedLagrangianStationarity << "|" << setw(10)
         << state.generalizedConstraintViolation << "|" << setw(10)
         << state.unconstrainedIterations << "|" << endl;
  }
  // Store state.
  if (p_->storeOptProgress) {
    progress_.emplace_back(state);
  }
}

}  // namespace gtsam
