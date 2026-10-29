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
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtPolicy.h>

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
  double squaredEqualityViolation = 0.0;
  double squaredInequalityViolation = 0.0;
  std::vector<Vector> equalityMultipliers;
  std::vector<double> projectedMultipliers;
};

/* ************************************************************************* */
double InfinityNorm(const Vector& vector) {
  return vector.size() == 0 ? 0.0 : vector.cwiseAbs().maxCoeff();
}

/* ************************************************************************* */
double AugmentedLagrangianStationarity(const GaussianFactorGraph& linear,
                                      VectorValues* gradient) {
  // Keep the keyed gradient storage across inner and outer iterations. For
  // ordinary Jacobians, accumulate directly instead of allocating a temporary
  // VectorValues for every factor and then merging it into another map.
  gradient->setZero();
  for (const auto& factor : linear) {
    if (!factor) continue;
    if (const auto* jacobian = dynamic_cast<const JacobianFactor*>(factor.get())) {
      Vector rhs = jacobian->getb();
      if (jacobian->get_model()) jacobian->get_model()->whitenInPlace(rhs);
      jacobian->transposeMultiplyAdd(-1.0, rhs, *gradient);
    } else {
      gradient->addInPlace_(factor->gradientAtZero());
    }
  }
  double infinityNorm = 0.0;
  for (const auto& keyGradient : *gradient) {
    if (!keyGradient.second.allFinite()) {
      return std::numeric_limits<double>::infinity();
    }
    infinityNorm = std::max(infinityNorm, InfinityNorm(keyGradient.second));
  }
  return infinityNorm;
}

/* ************************************************************************* */
Diagnostics EvaluateDiagnostics(const ConstrainedOptProblem& problem,
                                const GaussianFactorGraph& linear,
                                const AugmentedLagrangianState& subproblem) {
  Diagnostics diagnostics;
  size_t slot = problem.costs().size();
  const double sqrtEqualityPenalty = std::sqrt(subproblem.muEq);
  const double sqrtInequalityPenalty = std::sqrt(subproblem.muIneq);

  // Our augmented factors produce whitened Jacobian factors with b=-r.
  // Recover h from r=sqrt(rho)*(h+lambda/rho), without evaluating h again.
  for (size_t i = 0; i < problem.eConstraints().size(); ++i) {
    const auto& factor = static_cast<const JacobianFactor&>(*linear.at(slot++));
    diagnostics.equalityMultipliers.push_back(-sqrtEqualityPenalty *
                                              factor.getb());
    const Vector residual =
        (diagnostics.equalityMultipliers.back() - subproblem.lambdaEq.at(i)) /
        subproblem.muEq;
    diagnostics.squaredEqualityViolation += residual.squaredNorm();
    diagnostics.generalizedConstraintViolation = std::max(
        diagnostics.generalizedConstraintViolation, InfinityNorm(residual));
  }

  const auto& inequalities = problem.iConstraints();
  for (size_t i = 0; i < inequalities.size(); ++i) {
    const auto& factor = static_cast<const JacobianFactor&>(*linear.at(slot++));
    const double lambda = subproblem.lambdaIneq.at(i);

    // q=max(g,-lambda/rho) makes the projected multiplier update
    // lambda^+=max(0,lambda+rho*g)=lambda+rho*q. Thus q=0 encodes primal
    // feasibility and complementarity, including inactive inequalities.
    const double projectedLambda =
        std::max(0.0, -sqrtInequalityPenalty * factor.getb()(0));
    const double projectedResidual =
        (projectedLambda - lambda) / subproblem.muIneq;
    diagnostics.projectedMultipliers.push_back(projectedLambda);

    // On the active PHR branch q=g. Otherwise g is not recoverable, but
    // primal violation and projected complementarity are both zero.
    const double expression = projectedLambda > 0.0 ? projectedResidual : 0.0;
    const double violation = std::max(0.0, expression);
    diagnostics.squaredInequalityViolation += violation * violation;

    // BCL accepts a multiplier update using
    // theta=max(||h||_inf,||q||_inf), not merely max(g,0).
    diagnostics.generalizedConstraintViolation =
        std::max(diagnostics.generalizedConstraintViolation,
                 std::abs(projectedResidual));
    // Report primal violation and complementarity separately for diagnosis.
    diagnostics.primalInequalityViolation =
        std::max(diagnostics.primalInequalityViolation, violation);
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
  state->bclOmega = std::max(
      params.absoluteStationarityTolerance,
      params.bclOmega0 * std::pow(state->bclAlpha, params.bclAlphaOmega));
  state->bclEta =
      params.bclEta0 * std::pow(state->bclAlpha, params.bclAlphaEta);
}

/* ************************************************************************* */
bool AggressiveConverged(const AugmentedLagrangianState& state,
                         double previousViolation, double previousCost,
                         const AugmentedLagrangianParams& params) {
  if (state.violation() < params.absoluteViolationTolerance &&
      state.cost < params.absoluteCostTolerance) {
    return true;
  }
  return std::abs(state.violation() - previousViolation) <
             params.relativeViolationTolerance &&
         std::abs(state.cost - previousCost) <
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
struct AugmentedLagrangianOptimizer::Workspace {
  NonlinearFactorGraph graph;
  std::unique_ptr<NonlinearMultifrontalSolver> solver;
  VectorValues gradient;

  struct Result {
    Values values;
    GaussianFactorGraph::shared_ptr linear;
    size_t iterations = 0;
    double stationarity = std::numeric_limits<double>::infinity();
  };

  explicit Workspace(NonlinearFactorGraph merit) : graph(std::move(merit)) {}

  // Only numerical parameters change between subproblems. In particular, PHR
  // activity never removes a factor or changes its keys or row count.
  void update(const ConstrainedOptProblem& problem, const State& state) {
    size_t slot = problem.costs().size();
    for (const Vector& lambda : state.lambdaEq) {
      static_cast<BiasedFactor&>(*graph.at(slot++))
          .setParameters(lambda, state.muEq);
    }
    for (double lambda : state.lambdaIneq) {
      static_cast<PhrInequalityFactor&>(*graph.at(slot++))
          .setParameters(lambda, state.muIneq);
    }
  }

  // Retry damping against the same loaded linearization. Return false when
  // no acceptable step can be found; values and error then remain unchanged.
  bool step(const LevenbergMarquardtParams& params, Values* values,
            double* error, double* damping, double* dampingFactor) {
    while (true) {
      bool stopSearching = false;
      try {
        solver->eliminateInPlace(*damping);
        const VectorValues& delta = solver->updateSolution();
        double oldLinearError = 0.0;
        const double predictedDecrease = solver->deltaError(&oldLinearError);
        if (predictedDecrease >= 0.0) {
          Values candidate = values->retract(delta);
          const double candidateError = graph.error(candidate);
          const double decrease = *error - candidateError;
          if (std::isfinite(candidateError) &&
              predictedDecrease > std::numeric_limits<double>::epsilon() *
                                      std::abs(oldLinearError)) {
            const double fidelity = decrease / predictedDecrease;
            if (fidelity > params.minModelFidelity) {
              *values = std::move(candidate);
              *error = candidateError;
              internal::decreaseLevenbergMarquardtLambda(
                  params, fidelity, damping, dampingFactor);
              return true;
            }
          }
          stopSearching = std::abs(decrease) < params.relativeErrorTol * *error;
        }
      } catch (const IndeterminateSystemException&) {
        // Retry with more damping, using the same numerical model.
      }
      if (stopSearching) return false;
      internal::increaseLevenbergMarquardtLambda(params, damping,
                                                 dampingFactor);
      // A long sequence of accepted steps can underflow zero-bounded damping.
      // A rejected step must still be able to restart the damping search.
      if (*damping == 0.0) *damping = std::numeric_limits<double>::epsilon();
      if (!std::isfinite(*damping) || *damping >= params.lambdaUpperBound) {
        return false;
      }
    }
  }

  Result solve(Values initialValues, const State& state, const Params& params) {
    const auto& lm = params.lmParams;
    const bool bcl =
        params.updatePolicy == AugmentedLagrangianUpdatePolicy::BCL;
    Result result;
    result.values = std::move(initialValues);
    double error = graph.error(result.values);
    double damping = lm.lambdaInitial;
    double dampingFactor = lm.lambdaFactor;
    bool converged = !bcl && error <= lm.errorTol;

    while (true) {
      // BCL tests the actual gradient at the current point, then uses this
      // very same linearization for the next step. The terminal model also
      // supplies the outer feasibility test and projected multiplier update.
      result.linear = graph.linearize(result.values);
      const bool stopping = converged ||
                            result.iterations >= lm.maxIterations ||
                            !std::isfinite(error);
      if (bcl || stopping) {
        result.stationarity =
            AugmentedLagrangianStationarity(*result.linear, &gradient);
      }
      if ((bcl && result.stationarity <= state.bclOmega) || stopping ||
          (bcl && !std::isfinite(result.stationarity))) {
        return result;
      }
      if (!solver) {
        const Ordering ordering =
            lm.ordering ? *lm.ordering
                        : Ordering::Create(lm.orderingType, graph);
        solver = std::make_unique<NonlinearMultifrontalSolver>(
            graph, result.values, ordering, lm.multifrontalParams,
            lm.dampingParams);
      }
      solver->load(*result.linear);
      const double previousError = error;
      if (!step(lm, &result.values, &error, &damping, &dampingFactor)) {
        // The point did not change: its stationarity/model are still valid.
        if (!bcl) {
          result.stationarity =
              AugmentedLagrangianStationarity(*result.linear, &gradient);
        }
        return result;
      }
      ++result.iterations;
      converged = !bcl && gtsam::checkConvergence(
                              lm.relativeErrorTol, lm.absoluteErrorTol,
                              lm.errorTol, previousError, error);
    }
  }
};

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
  std::unique_ptr<Workspace> workspace;
  return iterate(state, muEq, muIneq, workspace);
}

/* ************************************************************************* */
std::tuple<AugmentedLagrangianOptimizer::State, double, double>
AugmentedLagrangianOptimizer::iterate(
    const State& state, double muEq, double muIneq,
    std::unique_ptr<Workspace>& workspace) const {
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
  // Do not demand inner accuracy beyond the requested final stationarity.
  // On badly scaled problems omega can otherwise fall below rounding error
  // while feasibility still needs additional outer multiplier updates.
  subproblemState.bclOmega =
      std::max(subproblemState.bclOmega, p_->absoluteStationarityTolerance);

  const auto start = std::chrono::steady_clock::now();

  if (!workspace) {
    workspace = std::make_unique<Workspace>(
        augmentedLagrangianFunction(subproblemState));
  }
  workspace->update(problem_, subproblemState);
  Workspace::Result inner = workspace->solve(
      std::move(subproblemState.values), subproblemState, *p_);
  const double stationarity = inner.stationarity;

  // Evaluate objective and all policy diagnostics at the returned point
  // x_{k+1}; evaluating at x_k was the old sequencing error.
  // The subproblem's Values have moved into the inner solve. This copies only
  // its primal-dual metadata; keep the old multipliers for diagnostics/updates.
  State solvedState = subproblemState;
  solvedState.iteration = state.iteration + 1;
  solvedState.values = std::move(inner.values);
  solvedState.cost = problem_.costs().error(solvedState.values);
  solvedState.unconstrainedIterations = inner.iterations;
  solvedState.totalUnconstrainedIterations =
      state.totalUnconstrainedIterations + solvedState.unconstrainedIterations;
  solvedState.augmentedLagrangianStationarity = stationarity;
  const Diagnostics diagnostics =
      EvaluateDiagnostics(problem_, *inner.linear, subproblemState);
  solvedState.eqConstraintViolation =
      std::sqrt(diagnostics.squaredEqualityViolation);
  solvedState.ineqConstraintViolation =
      std::sqrt(diagnostics.squaredInequalityViolation);
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
          solvedState.lambdaEq.at(i) = diagnostics.equalityMultipliers.at(i);
        }
        for (size_t i = 0; i < problem_.iConstraints().size(); ++i) {
          solvedState.lambdaIneq.at(i) = diagnostics.projectedMultipliers.at(i);
        }
        solvedState.updateType = AugmentedLagrangianUpdateType::Multiplier;

        // Tighten the next inner and feasibility targets using the unchanged
        // alpha_k, as in the accepted branch of Algorithm 1.
        solvedState.bclOmega =
            std::max(p_->absoluteStationarityTolerance,
                     subproblemState.bclOmega *
                         std::pow(subproblemState.bclAlpha, p_->bclBetaOmega));
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
  return {std::move(solvedState), nextMuEq, nextMuIneq};
}

/* ************************************************************************* */
Values AugmentedLagrangianOptimizer::optimize() const {
  return optimizeWithState().values;
}

/* ************************************************************************* */
AugmentedLagrangianOptimizer::State
AugmentedLagrangianOptimizer::optimizeWithState() const {
  validateConfiguration();
  progress_.clear();
  // Growing this vector can copy every saved Values snapshot because Values'
  // move constructor is not noexcept. The outer iteration cap is known.
  if (p_->storeOptProgress) progress_.reserve(p_->maxIterations + 1);

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
  std::unique_ptr<Workspace> workspace;
  while (true) {
    const double previousViolation = state.violation();
    const double previousCost = state.cost;
    std::tie(state, muEq, muIneq) =
        iterate(state, muEq, muIneq, workspace);

    if (p_->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL) {
      state.converged = state.innerConverged &&
                        state.augmentedLagrangianStationarity <=
                            p_->absoluteStationarityTolerance &&
                        state.generalizedConstraintViolation <=
                            p_->absoluteViolationTolerance;
    } else {
      state.converged = AggressiveConverged(
          state, previousViolation, previousCost, *p_);
    }
    logIteration(state);

    if (state.converged || !state.innerConverged ||
        state.iteration >= p_->maxIterations) {
      break;
    }
  }

  return state;
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
void AugmentedLagrangianOptimizer::validateConfiguration() const {
  if (!p_) {
    throw std::invalid_argument("AugmentedLagrangianParams must not be null");
  }
  if (!(p_->lmParams.lambdaInitial > 0.0) ||
      !(p_->lmParams.lambdaFactor > 1.0) ||
      !(p_->lmParams.lambdaUpperBound > 0.0)) {
    throw std::invalid_argument(
        "ALM requires positive LM damping and factor > 1");
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
