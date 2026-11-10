/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#pragma once

#include <gtsam/constrained/AugmentedLagrangianSolver.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>
#include <gtsam/nonlinear/internal/LevenbergMarquardtPolicy.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>

namespace gtsam {
namespace internal {

/// Optional phase timer; disabled calls do not read the clock.
class AlmTimer {
 public:
  /// Accumulate disjoint phase time only for instrumented runs.
  AlmTimer(bool enabled, double* seconds)
      : seconds_(enabled ? seconds : nullptr) {
    if (seconds_) start_ = std::chrono::steady_clock::now();
  }
  /// Complete the timed scope.
  ~AlmTimer() {
    if (seconds_)
      *seconds_ += std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start_)
                       .count();
  }

 private:
  double* seconds_;
  std::chrono::steady_clock::time_point start_;
};

/// Exact equality merit residual in whitened constraint coordinates.
inline double AlmEqualityResidual(double residual, double lambda, double rho) {
  return std::sqrt(rho) * (residual + lambda / rho);
}

/// Exact PHR residual and its scale on the underlying constraint Jacobian.
inline std::pair<double, double> AlmPhrResidual(double residual, double lambda,
                                                double rho) {
  const double shifted = lambda + rho * residual;
  const double scale = std::sqrt(rho);
  return shifted > 0 ? std::make_pair(shifted / scale, scale)
                     : std::make_pair(0.0, 0.0);
}

/// Original model data at a point; constraint Jacobians are already whitened.
struct AlmPoint {
  double cost = 0;
  std::vector<Vector> equalities;
  std::vector<double> inequalities;
  GaussianFactorGraph costs;
  std::vector<JacobianFactor::shared_ptr> equalityRows, inequalityRows;
  bool derivatives = false;
};

/// Infinity norm, including explicit nonfinite-gradient failure detection.
inline double AlmGradientNorm(const VectorValues& gradient) {
  double norm = 0;
  for (const auto& [key, value] : gradient) {
    if (!value.allFinite()) return std::numeric_limits<double>::infinity();
    if (value.size()) norm = std::max(norm, value.cwiseAbs().maxCoeff());
  }
  return norm;
}

/// Accumulate centered Gaussian-factor gradients without a graph-wide map copy.
inline void AlmAddCenteredGradient(const GaussianFactorGraph& graph,
                                   VectorValues* gradient) {
  for (const auto& factor : graph) {
    if (!factor) continue;
    if (const auto* jacobian =
            dynamic_cast<const JacobianFactor*>(factor.get())) {
      Vector rhs = jacobian->getb();
      if (jacobian->get_model()) jacobian->get_model()->whitenInPlace(rhs);
      jacobian->transposeMultiplyAdd(-1., rhs, *gradient);
    } else {
      gradient->addInPlace_(factor->gradientAtZero());
    }
  }
}

/// Validate shared policy and damping options for either frontend.
inline void ValidateAlmParams(const AugmentedLagrangianParams& p) {
  if (!(p.lmParams.lambdaInitial > 0) || !(p.lmParams.lambdaFactor > 1) ||
      !(p.lmParams.lambdaUpperBound > 0))
    throw std::invalid_argument(
        "ALM requires positive LM damping and factor > 1");
  if (p.ineqConstraintPenaltyFunction)
    throw std::invalid_argument("ALM requires exact PHR inequalities");
  if (!(p.initialMuEq > 0) || !(p.initialMuIneq > 0) ||
      !(p.muEqIncreaseRate > 0) || !(p.muIneqIncreaseRate > 0) ||
      !(p.absoluteStationarityTolerance >= 0) ||
      !std::isfinite(p.absoluteStationarityTolerance) ||
      !(p.absoluteViolationTolerance >= 0) ||
      !std::isfinite(p.absoluteViolationTolerance))
    throw std::invalid_argument("Invalid ALM tolerances or penalties");
  if (!(p.bclInitialPenalty > 0) || !(p.bclPenaltyIncreaseRate > 1) ||
      !(p.bclOmega0 > 0) || !(p.bclEta0 > 0) || !(p.bclGamma1 > 0) ||
      !(p.bclAlphaOmega > 0) || !(p.bclBetaOmega > 0) || !(p.bclAlphaEta > 0) ||
      !(p.bclBetaEta > 0))
    throw std::invalid_argument("Invalid BCL ALM parameters");
}

/// Reset the BCL schedule for the next direct penalty.
template <class State>
void InitializeAlmSchedule(const AugmentedLagrangianParams& p, double rho,
                           State* state) {
  state->bclAlpha = std::min(1. / rho, p.bclGamma1);
  state->bclOmega =
      std::max(p.absoluteStationarityTolerance,
               p.bclOmega0 * std::pow(state->bclAlpha, p.bclAlphaOmega));
  state->bclEta = p.bclEta0 * std::pow(state->bclAlpha, p.bclAlphaEta);
}

/// Initialize only multiplier fields, preserving the supplied primal state.
template <class State>
void InitializeAlmMultipliers(const std::vector<size_t>& dimensions,
                              size_t inequalities, State* state) {
  state->lambdaEq.clear();
  for (size_t dim : dimensions) state->lambdaEq.push_back(Vector::Zero(dim));
  state->lambdaIneq.assign(inequalities, 0.);
}

/// Shared Aggressive ascent from original whitened constraint residuals.
template <class State>
void UpdateAggressiveAlmMultipliers(const AugmentedLagrangianParams& params,
                                    const AlmPoint& point, State* state) {
  double equalityStep = std::min(params.maxDualStepSizeEq,
                                 state->muEq * params.dualStepSizeFactorEq);
  double inequalityStep =
      std::min(params.maxDualStepSizeIneq,
               state->muIneq * params.dualStepSizeFactorIneq);
  for (size_t i = 0; i < state->lambdaEq.size(); ++i)
    state->lambdaEq[i] += equalityStep * point.equalities[i];
  for (size_t i = 0; i < state->lambdaIneq.size(); ++i)
    state->lambdaIneq[i] = std::max(
        0., state->lambdaIneq[i] + inequalityStep * point.inequalities[i]);
}

/// Shared independent Aggressive penalty updates.
template <class State>
std::pair<double, double> UpdateAggressiveAlmPenalty(
    const AugmentedLagrangianParams& params, const State& previous,
    const State& state) {
  double equality = state.muEq, inequality = state.muIneq;
  if (!state.lambdaEq.empty() &&
      state.eqConstraintViolation >=
          params.muIncreaseThreshold * previous.eqConstraintViolation)
    equality *= params.muEqIncreaseRate;
  if (!state.lambdaIneq.empty() &&
      state.ineqConstraintViolation >=
          params.muIncreaseThreshold * previous.ineqConstraintViolation)
    inequality *= params.muIneqIncreaseRate;
  return {equality, inequality};
}

/// Shared progress recording; frontend state types differ only in primal
/// storage.
template <class State>
void RecordAlmState(const AugmentedLagrangianParams& params, const State& state,
                    std::vector<State>* progress) {
  if (params.verbose)
    std::cout << std::setw(8) << state.iteration << " | rho=" << state.muEq
              << " | cost=" << state.cost
              << " | violation=" << state.violation()
              << " | stationarity=" << state.augmentedLagrangianStationarity
              << '\n';
  if (params.storeOptProgress) progress->push_back(state);
}

/// A single ALM implementation; Model supplies only evaluation and primal
/// algebra.
template <class Model, class State>
class AugmentedLagrangianEngine {
 public:
  using Primal = typename Model::Primal;

  /// Retain one model and numerical workspace for a complete call.
  AugmentedLagrangianEngine(Model model,
                            const AugmentedLagrangianParams& params)
      : model_(std::move(model)), params_(params) {
    ValidateAlmParams(params_);
    statistics_.objectiveFactorCopies = model_.objectiveFactorCopies();
    statistics_.preparationSeconds = model_.preparationSeconds();
  }

  /// Initialize multipliers using original equality blocks and scalar rows.
  void initializeMultipliers(State* state) const {
    InitializeAlmMultipliers(model_.equalityDimensions(),
                             model_.inequalityCount(), state);
  }

  /// Run the shared outer policy and retain snapshots only when requested.
  State optimize(State state, std::vector<State>* progress) {
    progress->clear();
    if (params_.storeOptProgress) progress->reserve(params_.maxIterations + 1);
    initializeMultipliers(&state);
    double rhoEq = params_.initialMuEq, rhoIneq = params_.initialMuIneq;
    if (isBcl()) {
      rhoEq = rhoIneq = params_.bclInitialPenalty;
      InitializeAlmSchedule(params_, rhoEq, &state);
    }
    state.muEq = rhoEq;
    state.muIneq = rhoIneq;
    const AlmPoint initial = evaluate(state.values, false);
    state.cost = initial.cost;
    double equalitySquared = 0, inequalitySquared = 0;
    for (const Vector& residual : initial.equalities)
      equalitySquared += residual.squaredNorm();
    for (double residual : initial.inequalities)
      inequalitySquared += std::pow(std::max(0., residual), 2);
    state.eqConstraintViolation = std::sqrt(equalitySquared);
    state.ineqConstraintViolation = std::sqrt(inequalitySquared);
    record(state, progress);
    while (true) {
      const double previousCost = state.cost,
                   previousViolation = state.violation();
      std::tie(state, rhoEq, rhoIneq) = iterate(state, rhoEq, rhoIneq);
      if (isBcl()) {
        state.converged = state.innerConverged &&
                          state.augmentedLagrangianStationarity <=
                              params_.absoluteStationarityTolerance &&
                          state.generalizedConstraintViolation <=
                              params_.absoluteViolationTolerance;
      } else {
        state.converged =
            state.innerConverged &&
            ((state.violation() < params_.absoluteViolationTolerance &&
              state.cost < params_.absoluteCostTolerance) ||
             (std::abs(state.violation() - previousViolation) <
                  params_.relativeViolationTolerance &&
              std::abs(state.cost - previousCost) <
                  params_.relativeCostTolerance));
      }
      if (state.converged)
        state.status = AugmentedLagrangianStatus::Converged;
      else if (state.innerConverged && state.iteration >= params_.maxIterations)
        state.status = AugmentedLagrangianStatus::OuterIterationLimit;
      record(state, progress);
      if (state.converged || !state.innerConverged ||
          state.iteration >= params_.maxIterations)
        return state;
    }
  }

  /// Solve one fixed-multiplier subproblem and perform exactly one policy
  /// update.
  std::tuple<State, double, double> iterate(const State& previous, double rhoEq,
                                            double rhoIneq) {
    if (!(rhoEq > 0) || !(rhoIneq > 0) || !std::isfinite(rhoEq) ||
        !std::isfinite(rhoIneq))
      throw std::invalid_argument(
          "ALM direct penalties must be finite and positive");
    if (isBcl() && rhoEq != rhoIneq)
      throw std::invalid_argument("BCL requires muEq == muIneq");
    const auto start = std::chrono::steady_clock::now();
    State state = previous;
    state.muEq = rhoEq;
    state.muIneq = rhoIneq;
    if (state.lambdaEq.empty() && state.lambdaIneq.empty())
      initializeMultipliers(&state);
    validateMultipliers(state);
    if (isBcl() && (state.bclOmega <= 0 || state.bclEta <= 0))
      InitializeAlmSchedule(params_, rhoEq, &state);
    state.bclOmega =
        std::max(state.bclOmega, params_.absoluteStationarityTolerance);
    state.converged = false;
    state.updateType = AugmentedLagrangianUpdateType::None;
    state.status = AugmentedLagrangianStatus::NotConverged;
    const double omega = state.bclOmega;
    const double eta = state.bclEta;
    inner(&state);
    state.iteration = previous.iteration + 1;
    state.totalUnconstrainedIterations =
        previous.totalUnconstrainedIterations + state.unconstrainedIterations;
    state.cost = point_->cost;
    std::vector<Vector> lambdaEqPlus;
    std::vector<double> lambdaIneqPlus;
    diagnostics(&state, &lambdaEqPlus, &lambdaIneqPlus);
    state.innerConverged =
        state.status == AugmentedLagrangianStatus::NotConverged &&
        (!isBcl() || state.augmentedLagrangianStationarity <= omega);
    double nextEq = rhoEq, nextIneq = rhoIneq;
    if (state.innerConverged) {
      const bool terminal = isBcl() &&
                            state.augmentedLagrangianStationarity <=
                                params_.absoluteStationarityTolerance &&
                            state.generalizedConstraintViolation <=
                                params_.absoluteViolationTolerance;
      if (!isBcl() || terminal || state.generalizedConstraintViolation <= eta) {
        if (isBcl()) {
          state.lambdaEq = std::move(lambdaEqPlus);
          state.lambdaIneq = std::move(lambdaIneqPlus);
          state.bclOmega =
              std::max(params_.absoluteStationarityTolerance,
                       omega * std::pow(state.bclAlpha, params_.bclBetaOmega));
          state.bclEta = eta * std::pow(state.bclAlpha, params_.bclBetaEta);
        } else {
          aggressiveMultipliers(&state);
        }
        state.updateType = AugmentedLagrangianUpdateType::Multiplier;
      } else {
        nextEq = nextIneq = rhoEq * params_.bclPenaltyIncreaseRate;
        state.updateType = AugmentedLagrangianUpdateType::Penalty;
        InitializeAlmSchedule(params_, nextEq, &state);
      }
      if (!isBcl()) {
        std::tie(nextEq, nextIneq) =
            UpdateAggressiveAlmPenalty(params_, previous, state);
        if (nextEq != rhoEq || nextIneq != rhoIneq)
          state.updateType =
              AugmentedLagrangianUpdateType::MultiplierAndPenalty;
      }
    }
    state.statistics = statistics_;
    state.time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    return {std::move(state), nextEq, nextIneq};
  }

 private:
  Model model_;
  const AugmentedLagrangianParams& params_;
  AugmentedLagrangianStatistics statistics_;
  std::optional<AlmPoint> point_;
  GaussianFactorGraph graph_;
  std::unique_ptr<NonlinearMultifrontalSolver> solver_;
  VectorValues gradient_, hessianDelta_;
  std::vector<JacobianFactor::shared_ptr> equalities_, inequalities_;
  std::vector<double> inequalityScales_;
  double equalityScale_ = -1;

  /// Evaluate either model behind the same optional phase accounting.
  AlmPoint evaluate(const Primal& values, bool derivatives) {
    AlmTimer timer(params_.enableProfiling, &statistics_.evaluationSeconds);
    return model_.evaluate(values, derivatives, &statistics_);
  }

  /// Whether BCL rather than Aggressive controls the outer run.
  bool isBcl() const {
    return params_.updatePolicy == AugmentedLagrangianUpdatePolicy::BCL;
  }

  /// Validate explicit multiplier state before any numerical solve.
  void validateMultipliers(const State& state) const {
    const auto dimensions = model_.equalityDimensions();
    if (state.lambdaEq.size() != dimensions.size() ||
        state.lambdaIneq.size() != model_.inequalityCount())
      throw std::invalid_argument(
          "ALM multiplier dimensions do not match constraints");
    for (size_t i = 0; i < dimensions.size(); ++i)
      if (state.lambdaEq[i].size() != static_cast<DenseIndex>(dimensions[i]) ||
          !state.lambdaEq[i].allFinite())
        throw std::invalid_argument(
            "ALM equality multiplier dimensions or values are invalid");
    for (double lambda : state.lambdaIneq)
      if (!std::isfinite(lambda) || lambda < 0)
        throw std::invalid_argument(
            "ALM inequality multipliers must be finite and nonnegative");
  }

  /// Evaluate the actual merit, including the same constant shifts as legacy
  /// ALM.
  double merit(const AlmPoint& point, const State& state) const {
    double error = point.cost;
    for (size_t i = 0; i < point.equalities.size(); ++i)
      for (DenseIndex row = 0; row < point.equalities[i].size(); ++row) {
        double residual = AlmEqualityResidual(
            point.equalities[i](row), state.lambdaEq[i](row), state.muEq);
        error += 0.5 * residual * residual;
      }
    for (size_t i = 0; i < point.inequalities.size(); ++i) {
      double residual = AlmPhrResidual(point.inequalities[i],
                                       state.lambdaIneq[i], state.muIneq)
                            .first;
      error += 0.5 * residual * residual;
    }
    return error;
  }

  /// Accumulate the true AL gradient using the original constraint Jacobians.
  double gradientAt(const Primal& values, const AlmPoint& point,
                    const State& state, VectorValues* gradient) {
    AlmTimer timer(params_.enableProfiling, &statistics_.diagnosticsSeconds);
    if (gradient->size() == 0)
      *gradient = model_.zeroVectors(values);
    else
      gradient->setZero();
    model_.addCostGradient(values, point, gradient);
    for (size_t i = 0; i < point.equalityRows.size(); ++i) {
      Vector lambda = state.lambdaEq[i] + state.muEq * point.equalities[i];
      point.equalityRows[i]->transposeMultiplyAdd(1., lambda, *gradient);
    }
    for (size_t i = 0; i < point.inequalityRows.size(); ++i) {
      double lambda = std::max(
          0., state.lambdaIneq[i] + state.muIneq * point.inequalities[i]);
      if (lambda != 0)
        point.inequalityRows[i]->transposeMultiplyAdd(1., Vector1(lambda),
                                                      *gradient);
    }
    return AlmGradientNorm(*gradient);
  }

  /// Copy augmented coefficient blocks only when the underlying matrix changes.
  void updateRow(const JacobianFactor& original, double scale, bool changed,
                 JacobianFactor::shared_ptr* cached) {
    if (!*cached) {
      *cached = std::make_shared<JacobianFactor>(original);
      changed = true;
    }
    if (changed) {
      for (auto it = original.begin(); it != original.end(); ++it) {
        size_t block = it - original.begin();
        (*cached)->getA((*cached)->begin() + block) = scale * original.getA(it);
      }
      ++statistics_.constraintCoefficientUpdates;
    }
  }

  /// Prepare the Gaussian step model in reused storage.
  void assemble(const Primal& values, const State& state) {
    {
      AlmTimer timer(params_.enableProfiling, &statistics_.assemblySeconds);
      const auto& costs = model_.linearizeCosts(values, *point_);
      graph_.resize(costs.size() + point_->equalityRows.size() +
                    point_->inequalityRows.size());
      for (size_t i = 0; i < costs.size(); ++i) graph_[i] = costs[i];
      equalities_.resize(point_->equalityRows.size());
      inequalities_.resize(point_->inequalityRows.size());
      if (inequalityScales_.size() != inequalities_.size())
        inequalityScales_.assign(inequalities_.size(), -1.);
      const double scale = std::sqrt(state.muEq);
      size_t slot = costs.size();
      for (size_t i = 0; i < equalities_.size(); ++i) {
        updateRow(*point_->equalityRows[i], scale,
                  !model_.fixedCoefficients() || equalityScale_ != scale,
                  &equalities_[i]);
        equalities_[i]->getb() =
            -scale * (point_->equalities[i] + state.lambdaEq[i] / state.muEq);
        graph_[slot++] = equalities_[i];
      }
      equalityScale_ = scale;
      for (size_t i = 0; i < inequalities_.size(); ++i) {
        auto [residual, rowScale] = AlmPhrResidual(
            point_->inequalities[i], state.lambdaIneq[i], state.muIneq);
        updateRow(
            *point_->inequalityRows[i], rowScale,
            !model_.fixedCoefficients() || inequalityScales_[i] != rowScale,
            &inequalities_[i]);
        inequalities_[i]->getb()(0) = -residual;
        inequalityScales_[i] = rowScale;
        graph_[slot++] = inequalities_[i];
      }
      ++statistics_.modelUpdates;
    }
    if (!solver_) {
      AlmTimer timer(params_.enableProfiling, &statistics_.preparationSeconds);
      Ordering ordering =
          params_.lmParams.ordering
              ? *params_.lmParams.ordering
              : Ordering::Create(params_.lmParams.orderingType, graph_);
      if (ordering.size() != graph_.keys().size() ||
          KeySet(ordering.begin(), ordering.end()) != graph_.keys())
        throw std::invalid_argument(
            "ALM ordering must cover all variable keys exactly once");
      solver_ = std::make_unique<NonlinearMultifrontalSolver>(
          graph_, ordering, params_.lmParams.multifrontalParams,
          params_.lmParams.dampingParams);
      ++statistics_.symbolicFactorizations;
    }
    AlmTimer timer(params_.enableProfiling, &statistics_.assemblySeconds);
    solver_->load(graph_);
  }

  /// Shared damping search with stable model reduction and roundoff
  /// verification.
  bool step(State* state, double* error, double* damping,
            double* dampingFactor) {
    const auto& lm = params_.lmParams;
    while (true) {
      try {
        ++statistics_.numericalEliminations;
        {
          AlmTimer timer(params_.enableProfiling,
                         &statistics_.eliminationSeconds);
          solver_->eliminateInPlace(*damping);
        }
        const VectorValues* solution;
        {
          AlmTimer timer(params_.enableProfiling,
                         &statistics_.backSubstitutionSeconds);
          solution = &solver_->updateSolution();
        }
        const VectorValues& delta = *solution;
        if (hessianDelta_.size() == 0)
          hessianDelta_ = VectorValues::Zero(delta);
        else
          hessianDelta_.setZero();
        {
          AlmTimer timer(params_.enableProfiling,
                         &statistics_.diagnosticsSeconds);
          graph_.multiplyHessianAdd(1., delta, hessianDelta_);
        }
        const double predicted =
            -gradient_.dot(delta) - 0.5 * delta.dot(hessianDelta_);
        if (std::isfinite(predicted) && predicted > 0 &&
            gradient_.dot(delta) < 0) {
          Primal candidate = model_.retract(state->values, delta);
          AlmPoint trial = evaluate(candidate, false);
          const double candidateError = merit(trial, *state);
          const double decrease = *error - candidateError;
          double fidelity = decrease / predicted;
          const double uncertainty =
              32 * std::numeric_limits<double>::epsilon() *
              std::max({1., std::abs(*error), std::abs(candidateError)});
          if (std::isfinite(candidateError) &&
              std::abs(decrease) <= uncertainty) {
            ++statistics_.roundoffChecks;
            trial = evaluate(candidate, true);
            VectorValues trialGradient;
            const double norm =
                gradientAt(candidate, trial, *state, &trialGradient);
            if (norm <= state->bclOmega ||
                norm < 0.9 * state->augmentedLagrangianStationarity)
              fidelity = 1.;
            else
              fidelity = -1.;
          }
          if (std::isfinite(candidateError) && fidelity > lm.minModelFidelity) {
            state->values = std::move(candidate);
            point_ = std::move(trial);
            *error = candidateError;
            decreaseLevenbergMarquardtLambda(lm, fidelity, damping,
                                             dampingFactor);
            return true;
          }
        }
      } catch (const IndeterminateSystemException&) {
        // Retry the same loaded model with greater temporary damping.
      }
      increaseLevenbergMarquardtLambda(lm, damping, dampingFactor);
      if (*damping == 0) *damping = std::numeric_limits<double>::epsilon();
      if (!std::isfinite(*damping) || *damping >= lm.lambdaUpperBound) {
        state->status = AugmentedLagrangianStatus::DampingLimit;
        return false;
      }
    }
  }

  /// Solve one merit subproblem, keeping the terminal point/model available.
  void inner(State* state) {
    const auto& lm = params_.lmParams;
    double damping = lm.lambdaInitial, dampingFactor = lm.lambdaFactor;
    state->unconstrainedIterations = 0;
    bool acceptedConvergence = false;
    while (true) {
      if (!point_ || !point_->derivatives)
        point_ = evaluate(state->values, true);
      state->augmentedLagrangianStationarity =
          gradientAt(state->values, *point_, *state, &gradient_);
      const double error = merit(*point_, *state);
      if (!std::isfinite(error) ||
          !std::isfinite(state->augmentedLagrangianStationarity)) {
        state->status = AugmentedLagrangianStatus::NumericalFailure;
        return;
      }
      if ((isBcl() &&
           state->augmentedLagrangianStationarity <= state->bclOmega) ||
          (!isBcl() && (acceptedConvergence || error <= lm.errorTol)))
        return;
      if (state->unconstrainedIterations >= lm.maxIterations) {
        state->status = AugmentedLagrangianStatus::InnerIterationLimit;
        return;
      }
      assemble(state->values, *state);
      double nextError = error;
      if (!step(state, &nextError, &damping, &dampingFactor)) return;
      ++state->unconstrainedIterations;
      acceptedConvergence =
          !isBcl() &&
          gtsam::checkConvergence(lm.relativeErrorTol, lm.absoluteErrorTol,
                                  lm.errorTol, error, nextError);
    }
  }

  /// Compute policy diagnostics directly from the original constraint
  /// residuals.
  void diagnostics(State* state, std::vector<Vector>* equalityPlus,
                   std::vector<double>* inequalityPlus) {
    AlmTimer timer(params_.enableProfiling, &statistics_.diagnosticsSeconds);
    double equalitySquared = 0, inequalitySquared = 0;
    state->generalizedConstraintViolation = 0;
    state->primalInequalityViolation = state->complementarity = 0;
    for (size_t i = 0; i < point_->equalities.size(); ++i) {
      const Vector& residual = point_->equalities[i];
      equalitySquared += residual.squaredNorm();
      if (residual.size())
        state->generalizedConstraintViolation =
            std::max(state->generalizedConstraintViolation,
                     residual.cwiseAbs().maxCoeff());
      equalityPlus->push_back(state->lambdaEq[i] + state->muEq * residual);
    }
    for (size_t i = 0; i < point_->inequalities.size(); ++i) {
      const double residual = point_->inequalities[i],
                   lambda = state->lambdaIneq[i];
      const double next = std::max(0., lambda + state->muIneq * residual);
      inequalityPlus->push_back(next);
      const double projected = std::max(residual, -lambda / state->muIneq);
      state->generalizedConstraintViolation =
          std::max(state->generalizedConstraintViolation, std::abs(projected));
      const double violation = std::max(0., residual);
      inequalitySquared += violation * violation;
      state->primalInequalityViolation =
          std::max(state->primalInequalityViolation, violation);
      state->complementarity =
          std::max(state->complementarity, std::abs(next * residual));
    }
    state->eqConstraintViolation = std::sqrt(equalitySquared);
    state->ineqConstraintViolation = std::sqrt(inequalitySquared);
  }

  /// Aggressive dual ascent uses the same original residuals as BCL.
  void aggressiveMultipliers(State* state) const {
    UpdateAggressiveAlmMultipliers(params_, *point_, state);
  }

  /// One progress/logging implementation for both public state types.
  void record(const State& state, std::vector<State>* progress) const {
    RecordAlmState(params_, state, progress);
  }
};

}  // namespace internal
}  // namespace gtsam
