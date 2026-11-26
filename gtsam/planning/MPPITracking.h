// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file MPPITracking.h
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#pragma once

#include <gtsam/planning/CandidateScoring.h>
#include <gtsam/planning/SamplingPlanner.h>

#include <algorithm>
#include <functional>
#include <limits>

namespace gtsam {

/**
 * Five-point quadratic Savitzky–Golay smoothing from MPPI-Generic. Columns
 * of history contain the two previous controls, oldest first. The terminal
 * control is repeated to extend the right edge of the horizon.
 */
GTSAM_EXPORT Matrix smoothControlTrajectory(const Matrix& controls,
                                            const Matrix& history);

/// Truncated Tsallis weighting from the MPPI-Generic colored controller.
class GTSAM_EXPORT TsallisMPPI : public SamplingAlgorithm {
 public:
  static constexpr bool includeIncumbent = false;
  /// Set the positive cost cutoff and shape exponent r > 1.
  TsallisMPPI(double cutoff, double shape, double inverseTemperature = 1.0);
  /// Normalize [1 - (cost-min)/cutoff]_+ raised to 1/(shape-1).
  Vector weights(const Vector& costs) const;
  Vector computeWeights(const Vector& costs,
                        const Vector& logWeightOffsets = {}) const override {
    if (logWeightOffsets.size() && logWeightOffsets.size() != costs.size())
      throw std::invalid_argument("Expected one log offset per trajectory.");
    if (logWeightOffsets.size())
      return weights(costs - logWeightOffsets / beta_);
    return weights(costs);
  }

 private:
  double cutoff_, shape_, beta_;
};

/// Nominal reference and tracked actual trajectory, or primitive and MPPI
/// solutions.
struct MPPIPlans {
  Values nominal;  ///< Selected nominal trajectory.
  Values actual;   ///< Actual-system trajectory, or colored MPPI alternative
                   ///< for primitives. On a stop, both plans contain the stop.
  size_t candidate;  ///< Nominal candidate index for robust MPPI. Tube and
                     ///< primitives use 0 for the alternative branch.
};

/**
 * Sampling configuration and random streams for MPPI-Generic tracking (ACDSLab,
 * revision abcd1ab). Each operation takes an existing SamplingPlanner. The
 * caller carries nominal/actual plans between measurement times. Graph error
 * determines trajectory cost, including factors on nonadjacent time steps.
 */
class GTSAM_EXPORT MPPITracking {
 public:
  /// Configure the sampling budget, temperature and random streams for repeated
  /// MPC steps.
  MPPITracking(const Matrix& covariance, size_t samples,
               double inverseTemperature, size_t seed = 42,
               const GaussianProposalOptions& options = {})
      : covariance_(covariance),
        samples_(samples),
        beta_(inverseTemperature),
        options_(options),
        proposal_(covariance, samples, seed, options),
        mppi_(inverseTemperature),
        generator_(seed) {
    if (!(inverseTemperature > 0.))
      throw std::invalid_argument(
          "Tracking requires a positive inverse temperature.");
  }

  /** Enable post-update smoothing using the two preceding nominal and actual
   * inputs. Supply fresh histories at each MPC step. An empty nominal history
   * disables smoothing; an empty actual history reuses the nominal history.
   */
  void setControlHistory(const Matrix& nominal, const Matrix& actual = {}) {
    const auto valid = [&](const Matrix& history) {
      return history.rows() == covariance_.rows() && history.cols() == 2 &&
             history.allFinite();
    };
    if ((nominal.size() && !valid(nominal)) ||
        (actual.size() && (!nominal.size() || !valid(actual))))
      throw std::invalid_argument("Expected two finite controls per history.");
    nominalHistory_ = nominal;
    actualHistory_ = actual.size() ? actual : nominal;
  }

  /// Run colored MPPI with exponential or positive-cutoff Tsallis weights.
  template <class State, class Control, class Dynamics>
  Values colored(const SamplingPlanner<State, Control>& planner,
                 const NonlinearFactorGraph& graph, const Values& initial,
                 const Dynamics& dynamics, double exponent, size_t iterations,
                 double tsallisCutoff = 0.0, double tsallisShape = 2.0) const {
    ColoredNoiseProposal colored(covariance_, samples_, exponent, generator_(),
                                 options_);
    Values plan =
        tsallisCutoff != 0.
            ? planner.optimize(graph, initial, dynamics, colored,
                               TsallisMPPI(tsallisCutoff, tsallisShape, beta_),
                               iterations)
            : planner.optimize(graph, initial, dynamics, colored, mppi_,
                               iterations);
    if (!nominalHistory_.size()) return plan;
    return planner.rollout(
        initial,
        smoothControlTrajectory(planner.nominalControls(plan), nominalHistory_),
        dynamics);
  }

  /**
   * Optimize from actual and nominal initial states with common perturbations.
   * Replace the reference when the actual minimum sampled cost falls below
   * the nominal minimum plus threshold. Track the winning reference using
   * feedback.
   */
  template <class State, class Control, class Dynamics, class Feedback>
  MPPIPlans tube(const SamplingPlanner<State, Control>& planner,
                 const NonlinearFactorGraph& graph, const Values& nominal,
                 const Values& actual, const Dynamics& dynamics,
                 const Feedback& feedback, double threshold,
                 size_t iterations) const {
    return tube(planner, graph, nominal, actual, dynamics, feedback, proposal_,
                threshold, iterations);
  }

  /** Draw both Tube branches from the supplied proposal. Copying
   * the proposal must replay its RNG stream. Conditioned generators receive
   * each branch's own initial Values and center.
   */
  template <class State, class Control, class Dynamics, class Feedback,
            class Proposal>
  MPPIPlans tube(const SamplingPlanner<State, Control>& planner,
                 const NonlinearFactorGraph& graph, const Values& nominal,
                 const Values& actual, const Dynamics& dynamics,
                 const Feedback& feedback, const Proposal& proposal,
                 double threshold, size_t iterations) const {
    if (!iterations || !std::isfinite(threshold))
      throw std::invalid_argument("Invalid tube MPPI parameters.");
    Values reference = nominal;
    auto real = proposal.initialize(planner.nominalControls(actual));
    auto ref = proposal.initialize(planner.nominalControls(reference));
    size_t candidate = 1;
    for (size_t i = 0; i < iterations; ++i) {
      // Identical RNG states provide common random numbers, including
      // zero-centered draws.
      auto nominalProposal = copyForReplay(proposal);
      auto realBatch = proposal.sample(real, actual, planner.dt_, false);
      auto refBatch =
          nominalProposal->sample(ref, reference, planner.dt_, false);
      Vector realCosts =
          scores(planner, graph, actual, dynamics, realBatch, real.mean);
      Vector refCosts =
          scores(planner, graph, reference, dynamics, refBatch, ref.mean);
      if (realCosts.size() != refCosts.size())
        throw std::invalid_argument(
            "Tube branches must have the same number of coupled candidates.");
      proposal.update(real, realBatch, mppi_.weights(realCosts));
      nominalProposal->update(ref, refBatch, mppi_.weights(refCosts));
      candidate =
          realCosts.minCoeff() < refCosts.minCoeff() + threshold ? 0 : 1;
      if (candidate == 0) {
        reference = actual;
        ref = real;
      }
    }
    if (nominalHistory_.size())
      ref.mean = smoothControlTrajectory(ref.mean, nominalHistory_);
    auto path = planner.rollout(reference, ref.mean, dynamics);
    auto tracked = track(planner, actual, path, ref.mean, dynamics, feedback);
    return {std::move(path), std::move(tracked), candidate};
  }

  /**
   * Choose the furthest initial-state candidate below the free energy threshold
   * along previous -> predicted -> measured state. Jointly roll out real and
   * nominal systems with shared disturbances and feedback in the real rollout.
   * feedbackPrecision weights the sum of half squared control corrections.
   * stride is the elapsed number of control steps since previousNominal.
   */
  template <class State, class Control, class Dynamics, class Feedback>
  MPPIPlans robust(const SamplingPlanner<State, Control>& planner,
                   const NonlinearFactorGraph& graph,
                   const Values& previousNominal, const Values& actual,
                   const Dynamics& dynamics, const Feedback& feedback,
                   double threshold, size_t candidates, size_t stride,
                   size_t iterations, const Matrix& feedbackPrecision) const {
    return robust(planner, graph, previousNominal, actual, dynamics, feedback,
                  proposal_, threshold, candidates, stride, iterations,
                  feedbackPrecision);
  }

  /** Use the same supplied proposal for initial-state evaluation
   * and paired nominal/actual optimization. Each candidate starts at the same
   * RNG position. During the paired optimization both systems draw around
   * the current nominal feedforward sequence, as in MPPI-Generic. The actual
   * system also applies tracking feedback. A state-feedback proposal first
   * realizes the shared feedforward controls along the nominal rollout.
   * The proposal sets the batch size.
   */
  template <class State, class Control, class Dynamics, class Feedback,
            class Proposal>
  MPPIPlans robust(const SamplingPlanner<State, Control>& planner,
                   const NonlinearFactorGraph& graph,
                   const Values& previousNominal, const Values& actual,
                   const Dynamics& dynamics, const Feedback& feedback,
                   const Proposal& proposal, double threshold,
                   size_t candidates, size_t stride, size_t iterations,
                   const Matrix& feedbackPrecision) const {
    if (candidates < 3 || !(candidates % 2) || !iterations ||
        !std::isfinite(threshold) || stride >= planner.stateKeys_.size())
      throw std::invalid_argument("Invalid robust MPPI search parameters.");
    const Matrix warm = planner.nominalControls(previousNominal);
    if (feedbackPrecision.rows() != warm.rows() ||
        feedbackPrecision.cols() != warm.rows() ||
        !feedbackPrecision.allFinite() ||
        !feedbackPrecision.isApprox(feedbackPrecision.transpose()) ||
        !Eigen::LDLT<Matrix>(feedbackPrecision).isPositive())
      throw std::invalid_argument(
          "Expected positive semidefinite feedback precision.");
    const State& previous =
        previousNominal.template at<State>(planner.stateKeys_.front());
    const State& predicted =
        previousNominal.template at<State>(planner.stateKeys_[stride]);
    const State& measured =
        actual.template at<State>(planner.stateKeys_.front());
    Matrix controls = warm;
    Values reference = actual;
    size_t selected = 0;
    size_t half = candidates / 2;
    // Use common perturbations for comparing initial states. Even with no
    // admissible candidate the earliest nominal state is a defined fallback.
    for (size_t j = 0; j < candidates; ++j) {
      const bool first = j <= half;
      const double fraction = double(first ? j : j - half) / half;
      const State& from = first ? previous : predicted;
      const State& to = first ? predicted : measured;
      const State candidate = traits<State>::Retract(
          from, fraction * traits<State>::Local(from, to));
      const size_t shift =
          first ? size_t(std::lround(fraction * stride)) : stride;
      const Matrix shifted = shiftControls(warm, shift);
      Values start = actual;
      start.insert_or_assign(planner.stateKeys_.front(), candidate);
      auto sample = [&](const auto& sampler) {
        return sampler.sample(sampler.initialize(shifted), start, planner.dt_,
                              false);
      };
      auto batch = j + 1 == candidates ? sample(proposal)
                                       : sample(*copyForReplay(proposal));
      const Vector costs =
          scores(planner, graph, start, dynamics, batch, shifted);
      const double baseline = costs.minCoeff();
      if (costs.hasNaN() ||
          baseline == -std::numeric_limits<double>::infinity())
        throw std::domain_error("Invalid initial-state candidate score.");
      const double energy =
          std::isfinite(baseline)
              ? baseline -
                    std::log(
                        (-(costs.array() - baseline) * beta_).exp().mean()) /
                        beta_
              : baseline;
      if (j == 0 || energy < threshold) {
        selected = j;
        reference = std::move(start);
        controls = shifted;
      }
    }
    auto ref = proposal.initialize(controls);
    auto real = ref;
    for (size_t i = 0; i < iterations; ++i) {
      auto batch = proposal.sample(ref, reference, planner.dt_, false);
      const auto count = batchSize(batch, ref.mean);
      Vector nominalCosts(count), actualCosts(count);
      Values path = reference;
      for (Eigen::Index j = 0; j < count; ++j) {
        planner.rolloutCandidateInPlace(path, batch, j, dynamics);
        auto feedforward =
            batch.controls.middleCols(j * controls.cols(), controls.cols());
        double effort = 0;
        auto tracked = track(planner, actual, path, feedforward, dynamics,
                             feedback, &feedbackPrecision, &effort);
        const double nom = internal::candidateGraphError(graph, path);
        const double act = internal::candidateGraphError(graph, tracked);
        nominalCosts[j] =
            0.5 * nom + 0.5 * std::max(nom, std::min(act + effort, threshold));
        actualCosts[j] = act;
      }
      mppi_.apply(proposal, ref, batch, nominalCosts);
      mppi_.apply(proposal, real, batch, actualCosts);
    }
    if (nominalHistory_.size()) {
      ref.mean = smoothControlTrajectory(ref.mean, nominalHistory_);
      real.mean = smoothControlTrajectory(real.mean, actualHistory_);
    }
    auto path = planner.rollout(reference, ref.mean, dynamics);
    auto tracked = track(planner, actual, path, real.mean, dynamics, feedback);
    return {std::move(path), std::move(tracked), selected};
  }

  /**
   * Search piecewise-linear primitives (including zero and incumbent) and
   * colored MPPI independently. Hysteresis favors MPPI when its baseline
   * falls below the primitive baseline plus hysteresis. Optional
   * caller-supplied stopping controls take priority if all primitives exceed
   * stopThreshold. Both outputs then contain the unfiltered stopping rollout
   * and the competing MPPI search is skipped.
   */
  template <class State, class Control, class Dynamics>
  MPPIPlans primitives(const SamplingPlanner<State, Control>& planner,
                       const NonlinearFactorGraph& graph, const Values& initial,
                       const Values& previousMPPI, const Dynamics& dynamics,
                       size_t segments, double exponent, double hysteresis,
                       size_t primitiveIterations, size_t mppiIterations,
                       double stopThreshold, const Matrix& stoppingControls,
                       double tsallisCutoff = 0.0, double tsallisShape = 2.0,
                       const MotionPrimitiveOptions& mixture = {}) const {
    if ((!primitiveIterations && !mppiIterations) ||
        !std::isfinite(hysteresis) || !std::isfinite(stopThreshold) ||
        (primitiveIterations && samples_ < 2))
      throw std::invalid_argument("Invalid primitives planner parameters.");
    auto primitiveState =
        proposal_.initialize(planner.nominalControls(initial));
    // Seed each receding-horizon call from a persistent stream.
    size_t seed = generator_();
    std::optional<PiecewiseLinearProposal> linear;
    if (primitiveIterations)
      linear.emplace(covariance_, samples_, segments, mixture, seed, options_);
    double bestPrimitive = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < primitiveIterations; ++i) {
      auto batch = linear->sample(primitiveState, initial, planner.dt_, true);
      auto costs =
          scores(planner, graph, initial, dynamics, batch, primitiveState.mean);
      bestPrimitive = costs.minCoeff();
      if (stopThreshold > 0 && bestPrimitive > stopThreshold) {
        if (stoppingControls.rows() != primitiveState.mean.rows() ||
            stoppingControls.cols() != primitiveState.mean.cols() ||
            !stoppingControls.allFinite())
          throw std::invalid_argument("Expected a complete stopping sequence.");
        auto stop = planner.rollout(initial, stoppingControls, dynamics);
        return {stop, std::move(stop), 1};
      }
      linear->update(primitiveState, batch,
                     PredictiveSampling().weights(costs));
    }
    auto coloredState =
        proposal_.initialize(planner.nominalControls(previousMPPI));
    ColoredNoiseProposal colored(covariance_, samples_, exponent, ~seed,
                                 options_);
    double bestMPPI = std::numeric_limits<double>::infinity();
    std::optional<TsallisMPPI> tsallis;
    if (tsallisCutoff != 0.)
      tsallis.emplace(tsallisCutoff, tsallisShape, beta_);
    for (size_t i = 0; i < mppiIterations; ++i) {
      auto batch = colored.sample(coloredState, initial, planner.dt_, false);
      auto costs =
          scores(planner, graph, initial, dynamics, batch, coloredState.mean);
      bestMPPI = costs.minCoeff();
      colored.update(coloredState, batch,
                     tsallis ? tsallis->weights(costs) : mppi_.weights(costs));
    }
    const bool useMPPI =
        mppiIterations &&
        (!primitiveIterations || bestMPPI < bestPrimitive + hysteresis);
    Matrix selectedControls = useMPPI ? coloredState.mean : primitiveState.mean;
    if (nominalHistory_.size()) {
      selectedControls =
          smoothControlTrajectory(selectedControls, nominalHistory_);
      coloredState.mean =
          smoothControlTrajectory(coloredState.mean, actualHistory_);
    }
    auto selected = planner.rollout(initial, selectedControls, dynamics);
    return {std::move(selected),
            planner.rollout(initial, coloredState.mean, dynamics),
            useMPPI ? 0u : 1u};
  }

 private:
  Matrix covariance_;
  size_t samples_;
  double beta_;
  GaussianProposalOptions options_;
  Matrix nominalHistory_, actualHistory_;
  GaussianProposal proposal_;
  MPPI mppi_;
  mutable std::mt19937_64 generator_;

  /// Keep dynamic Gaussian-family types and copy other proposals in their
  /// native type.
  template <class Proposal>
  static std::unique_ptr<Proposal> copyForReplay(const Proposal& proposal) {
    if constexpr (std::is_convertible_v<const Proposal*,
                                        const GaussianProposal*>) {
      if (typeid(proposal) != typeid(Proposal)) {
        auto copy = proposal.clone();
        auto* typed = dynamic_cast<Proposal*>(copy.get());
        if (!typed || typeid(*typed) != typeid(proposal))
          throw std::invalid_argument(
              "A derived Gaussian proposal must clone its dynamic type for "
              "coupled sampling.");
        copy.release();
        return std::unique_ptr<Proposal>(typed);
      }
    }
    return std::make_unique<Proposal>(proposal);
  }

  /// Validate physical controls and any likelihood offsets before indexing a
  /// batch.
  template <class Batch>
  static Eigen::Index batchSize(const Batch& batch, const Matrix& mean) {
    const auto horizon = mean.cols();
    if (!horizon || batch.controls.rows() != mean.rows() ||
        !batch.controls.cols() || batch.controls.cols() % horizon)
      throw std::invalid_argument(
          "Expected a batch of complete control trajectories.");
    const auto count = batch.controls.cols() / horizon;
    if (batch.logWeightOffsets.size() && batch.logWeightOffsets.size() != count)
      throw std::invalid_argument("Expected one log offset per trajectory.");
    return count;
  }

  /// Fill the tail with the final control when advancing an MPC horizon.
  static Matrix shiftControls(const Matrix& u, size_t stride) {
    Matrix result(u.rows(), u.cols());
    for (Eigen::Index i = 0; i < u.cols(); ++i)
      result.col(i) = u.col(std::min<Eigen::Index>(i + stride, u.cols() - 1));
    return result;
  }
  /// Evaluate complete factor graphs and convert optional likelihood offsets
  /// into cost units.
  template <class State, class Control, class Dynamics, class Batch>
  Vector scores(const SamplingPlanner<State, Control>& planner,
                const NonlinearFactorGraph& graph, const Values& initial,
                const Dynamics& dynamics, Batch& batch,
                const Matrix& mean) const {
    Vector result(batchSize(batch, mean));
    Values path = initial;
    for (Eigen::Index i = 0; i < result.size(); ++i) {
      planner.rolloutCandidateInPlace(path, batch, i, dynamics);
      result[i] = internal::candidateGraphError(graph, path);
    }
    if (batch.logWeightOffsets.size()) result -= batch.logWeightOffsets / beta_;
    return result;
  }
  /// Propagate tracking corrections through the actual dynamics and optionally
  /// accumulate feedback effort.
  template <class State, class Control, class Dynamics, class Feedback>
  Values track(const SamplingPlanner<State, Control>& planner,
               const Values& actual, const Values& nominal,
               const Eigen::Ref<const Matrix>& controls,
               const Dynamics& dynamics, const Feedback& feedback,
               const Matrix* precision = nullptr,
               double* effort = nullptr) const {
    return planner.rolloutPolicy(
        actual,
        [&](size_t k, const State& x) -> Control {
          const Control u = controls.col(k);
          const Control correction = feedback(
              k, x, nominal.template at<State>(planner.stateKeys_[k]), u);
          if (correction.size() != u.size() || !correction.allFinite())
            throw std::invalid_argument("Invalid feedback correction.");
          if (precision)
            *effort += 0.5 * correction.dot((*precision) * correction);
          return u + correction;
        },
        dynamics);
  }
};
}  // namespace gtsam
