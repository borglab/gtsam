// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file PolicyOptimizationMPC.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <gtsam/inference/Symbol.h>
#include <gtsam/planning/EliteMPPI.h>
#include <gtsam_unstable/planning/PolicyOptimizationMPC.h>
#include <gtsam_unstable/planning/internal/AdamUpdate.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <random>

namespace gtsam {
namespace {
using Rng = std::mt19937_64;
using symbol_shorthand::U;
using symbol_shorthand::X;

Vector normal(Eigen::Index size, Rng& rng) {
  std::normal_distribution<double> gaussian;
  return Vector::NullaryExpr(size, [&] { return gaussian(rng); });
}

Vector concatenate(const Vector& a, const Vector& b) {
  Vector result(a.size() + b.size());
  result << a, b;
  return result;
}

// Small CPU network with analytic vector-Jacobian products and accumulated
// batch gradients.
class Network {
 public:
  Network(Eigen::Index inputs, Eigen::Index hidden, Eigen::Index outputs,
          Rng& rng)
      : input_(hidden, inputs),
        output_(outputs, hidden),
        bias_(Vector::Zero(hidden)),
        outputBias_(Vector::Zero(outputs)),
        di_(Matrix::Zero(hidden, inputs)),
        dout_(Matrix::Zero(outputs, hidden)),
        db_(Vector::Zero(hidden)),
        dob_(Vector::Zero(outputs)) {
    for (Eigen::Index j = 0; j < input_.cols(); ++j)
      input_.col(j) = normal(hidden, rng) / std::sqrt(inputs);
    for (Eigen::Index j = 0; j < output_.cols(); ++j)
      output_.col(j) = 0.05 * normal(outputs, rng) / std::sqrt(hidden);
  }

  Vector predict(const Vector& x) const {
    return output_ * hidden(x) + outputBias_;
  }

  Vector inputGradient(const Vector& x, const Vector& dy) const {
    return input_.transpose() * hiddenGradient(hidden(x), dy);
  }

  void accumulate(const Vector& x, const Vector& dy) {
    const Vector h = hidden(x), dh = hiddenGradient(h, dy);
    dout_.noalias() += dy * h.transpose();
    dob_ += dy;
    di_.noalias() += dh * x.transpose();
    db_ += dh;
  }

  void update(double rate, size_t batchSize, bool useAdam) {
    const double norm = std::sqrt(di_.squaredNorm() + dout_.squaredNorm() +
                                  db_.squaredNorm() + dob_.squaredNorm());
    const double denominator =
        std::max(static_cast<double>(batchSize), norm / 10.0);
    if (useAdam) {
      internal::adamUpdate(
          firstMoment_, secondMoment_, adamUpdates_,
          {{input_, di_}, {output_, dout_}, {bias_, db_}, {outputBias_, dob_}},
          rate, denominator);
    } else {
      const double step = rate / denominator;
      input_ -= step * di_;
      output_ -= step * dout_;
      bias_ -= step * db_;
      outputBias_ -= step * dob_;
    }
    di_.setZero();
    dout_.setZero();
    db_.setZero();
    dob_.setZero();
  }

  void blend(const Network& source, double fraction) {
    input_ += fraction * (source.input_ - input_);
    output_ += fraction * (source.output_ - output_);
    bias_ += fraction * (source.bias_ - bias_);
    outputBias_ += fraction * (source.outputBias_ - outputBias_);
  }

 private:
  Vector hidden(const Vector& x) const {
    return (input_ * x + bias_).array().tanh();
  }
  Vector hiddenGradient(const Vector& h, const Vector& dy) const {
    return (1.0 - h.array().square()) * (output_.transpose() * dy).array();
  }
  Matrix input_, output_;
  Vector bias_, outputBias_;
  Matrix di_, dout_;
  Vector db_, dob_;
  Vector firstMoment_, secondMoment_;
  size_t adamUpdates_ = 0;
};

struct Gaussian {
  Vector mean, sd;
  Vector sample(Rng& rng) const {
    return mean + sd.cwiseProduct(normal(mean.size(), rng));
  }
};

struct Policy {
  Network net;
  const double logMid, logRadius;

  Policy(Eigen::Index n, Eigen::Index m, const PolicyOptimizationMPCParams& p,
         Rng& rng)
      : net(n, static_cast<Eigen::Index>(p.hiddenSize), 2 * m, rng),
        logMid(0.5 * (std::log(p.maxStddev) + std::log(p.minStddev))),
        logRadius(0.5 * (std::log(p.maxStddev) - std::log(p.minStddev))) {}

  Gaussian distribution(const Vector& x) const {
    const Vector y = net.predict(x);
    const auto m = y.size() / 2;
    return {y.head(m), (logMid + logRadius * y.tail(m).array().tanh()).exp()};
  }

  void accumulate(const Vector& x, const Vector& dMean, const Vector& dLog,
                  double weight) {
    const Vector y = net.predict(x);
    const Vector dScale = dLog.array() * logRadius *
                          (1.0 - y.tail(dLog.size()).array().tanh().square());
    net.accumulate(x, weight * concatenate(dMean, dScale));
  }
};

double diagonalKL(const Gaussian& policy, const Gaussian& prior) {
  const auto ratio = policy.sd.array() / prior.sd.array();
  const auto diff = (policy.mean - prior.mean).array() / prior.sd.array();
  return (0.5 * (ratio.square() + diff.square() - 1.0) - ratio.log()).sum();
}

struct Critic {
  Network net;
  Vector bins;

  Critic(Eigen::Index inputs, const PolicyOptimizationMPCParams& p, Rng& rng)
      : net(inputs, static_cast<Eigen::Index>(p.hiddenSize),
            static_cast<Eigen::Index>(p.valueBins), rng),
        bins(Vector::LinSpaced(static_cast<Eigen::Index>(p.valueBins),
                               -p.valueLimit, p.valueLimit)) {}

  Vector probabilities(const Vector& x) const {
    const Vector logits = net.predict(x);
    Vector probability = (logits.array() - logits.maxCoeff()).exp();
    return probability / probability.sum();
  }

  double value(const Vector& x) const { return probabilities(x).dot(bins); }

  Vector gradient(const Vector& x) const {
    const Vector probs = probabilities(x);
    return net.inputGradient(x,
                             probs.array() * (bins.array() - probs.dot(bins)));
  }

  void accumulate(const Vector& x, double target, double weight) {
    Vector error = probabilities(x);
    const double index = std::clamp((target - bins[0]) / (bins[1] - bins[0]),
                                    0., static_cast<double>(bins.size() - 1));
    const auto left = static_cast<Eigen::Index>(index);
    const double frac = index - static_cast<double>(left);
    error[left] -= 1. - frac;
    if (left + 1 < bins.size()) error[left + 1] -= frac;
    net.accumulate(x, weight * error);
  }
};

void checkVector(const Vector& v, size_t dim) {
  if (static_cast<size_t>(v.size()) != dim || !v.allFinite())
    throw std::invalid_argument(
        "PO-MPC input vector has invalid dimension or nonfinite entries");
}

double robustScale(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const double last = static_cast<double>(values.size() - 1);
  return std::max(1., values[static_cast<size_t>(0.95 * last)] -
                          values[static_cast<size_t>(0.05 * last)]);
}

}  // namespace

struct PolicyOptimizationMPC::Impl {
  struct Transition {
    Vector x, u, next;
    double reward;
    bool terminal;
    Gaussian plan;
    std::shared_ptr<const NonlinearFactorGraph> taskCost;
    bool endsEpisode = false;
  };

  const Eigen::Index n, m, horizon;
  const PolicyOptimizationMPCParams p;
  Rng rng;
  Network world;
  Policy actor, prior;
  Critic q, regularizedQ, targetQ, targetRegularizedQ;
  std::deque<Transition> replay;
  Matrix warm;
  size_t observed = 0, updates = 0;

  Impl(Eigen::Index n, Eigen::Index m,
       const PolicyOptimizationMPCParams& params)
      : n(n),
        m(m),
        horizon(static_cast<Eigen::Index>(params.horizon)),
        p(params),
        rng(p.seed),
        world(n + m, static_cast<Eigen::Index>(p.hiddenSize), n + 1, rng),
        actor(n, m, p, rng),
        prior(n, m, p, rng),
        q(n + m, p, rng),
        regularizedQ(n + m, p, rng),
        targetQ(q),
        targetRegularizedQ(regularizedQ),
        warm(Matrix::Zero(m, horizon)) {}

  PolicyOptimizationPlan search(
      const Vector& x, const Matrix& initial,
      const std::shared_ptr<const NonlinearFactorGraph>& graph) {
    GaussianProposal proposal(
        p.maxStddev * p.maxStddev * Matrix::Identity(m, m), p.samples, rng());
    return search(x, initial, graph, proposal, p.policySamples,
                  EliteMPPI(p.elites, 1.0 / p.temperature, p.minStddev),
                  p.iterations);
  }

  PolicyOptimizationPlan search(
      const Vector& x, const Matrix& initial,
      const std::shared_ptr<const NonlinearFactorGraph>& graph,
      const GaussianProposal& proposal, size_t policySamples,
      const EliteMPPI& algorithm, size_t iterations) {
    auto distribution = proposal.initialize(initial);
    Vector cost(static_cast<Eigen::Index>(proposal.sampler().numSamples())),
        weights;
    ControlBatch batch;
    PolicyOptimizationPlan result;
    for (size_t iteration = 0; iteration < iterations; ++iteration) {
      batch = proposal.sample(distribution, {}, 1., false);
      for (Eigen::Index i = 0; i < cost.size(); ++i) {
        Vector state = x;
        Values values;
        if (graph) values.insert(X(0), state);
        double total = 0., gamma = 1.;
        auto trajectory = controlTrajectory(batch.controls, i, horizon);
        for (Eigen::Index k = 0; k < horizon; ++k) {
          auto u = trajectory.col(k);
          if (i < static_cast<Eigen::Index>(policySamples))
            u = actor.distribution(state).sample(rng);
          if (std::isfinite(p.actionLimit)) {
            u = u.cwiseMax(-p.actionLimit).cwiseMin(p.actionLimit);
          }
          const Vector prediction = world.predict(concatenate(state, u));
          state += prediction.head(n);
          total += gamma * prediction[n];
          gamma *= p.discount;
          if (graph) {
            values.insert(U(k), Vector(u));
            values.insert(X(k + 1), state);
          }
        }
        const Vector last = actor.distribution(state)
                                .sample(rng)
                                .cwiseMax(-p.actionLimit)
                                .cwiseMin(p.actionLimit);
        total += gamma * q.value(concatenate(state, last));
        cost[i] = (graph ? graph->error(values) : 0.0) - total;
      }
      result.modelRollouts += static_cast<size_t>(cost.size());
      weights = algorithm.apply(proposal, distribution, batch, cost);
      distribution.stddev = distribution.stddev.array().min(p.maxStddev);
    }
    std::discrete_distribution<Eigen::Index> pick(
        weights.data(), weights.data() + weights.size());
    result.action = batch.controls.col(pick(rng) * horizon);
    result.mean = std::move(distribution.mean);
    result.stddev = std::move(distribution.stddev);
    result.taskCost_ = graph;
    return result;
  }

  PolicyOptimizationPlan plan(
      const Vector& x,
      const std::shared_ptr<const NonlinearFactorGraph>& graph) {
    checkVector(x, n);
    if (observed < p.warmupSteps) {
      PolicyOptimizationPlan result;
      result.mean = Matrix::Zero(m, horizon);
      result.stddev = Matrix::Constant(m, horizon, p.maxStddev);
      result.action = actor.distribution(x)
                          .sample(rng)
                          .cwiseMax(-p.actionLimit)
                          .cwiseMin(p.actionLimit);
      result.taskCost_ = graph;
      return result;
    }
    PolicyOptimizationPlan result = search(x, warm, graph);
    if (p.horizon > 1)
      warm.leftCols(p.horizon - 1) = result.mean.rightCols(p.horizon - 1);
    warm.rightCols(1).setZero();
    return result;
  }

  struct Example {
    size_t index;
    Vector x;
    double weight;
    Gaussian reference, nextReference;
    Vector action;
    double kl, value;
  };

  Gaussian freshPrior(const Vector& state,
                      const std::shared_ptr<const NonlinearFactorGraph>& cost) {
    const auto plan = search(state, Matrix::Zero(m, horizon), cost);
    return {plan.mean.col(0), plan.stddev.col(0)};
  }

  std::vector<Example> batch(bool pretrain) {
    std::uniform_int_distribution<size_t> pick(0, replay.size() - 1);
    std::vector<Example> examples;
    for (size_t b = 0; b < p.batchSize; ++b) {
      const size_t start = pick(rng);
      // Reanalyze the first trajectory in this minibatch at the chosen
      // interval.
      const bool reanalyze = !pretrain && b == 0 && p.reanalyzeInterval &&
                             updates % p.reanalyzeInterval == 0;
      Vector x = replay[start].x;
      double weight = 1.;
      for (size_t k = 0; k < p.horizon && start + k < replay.size(); ++k) {
        const size_t i = start + k;
        auto& sample = replay[i];
        const bool hasSuccessor = !sample.endsEpisode && i + 1 < replay.size();
        if (reanalyze) sample.plan = freshPrior(sample.x, sample.taskCost);
        Gaussian reference =
            p.learnedPrior ? prior.distribution(x) : sample.plan;
        Gaussian nextReference = reference;
        if (!pretrain && !sample.terminal) {
          if (p.learnedPrior)
            nextReference = prior.distribution(sample.next);
          else if (hasSuccessor)
            nextReference = replay[i + 1].plan;
          else
            nextReference = freshPrior(sample.next, sample.taskCost);
        }
        const auto distribution = actor.distribution(x);
        const Vector action = distribution.sample(rng)
                                  .cwiseMax(-p.actionLimit)
                                  .cwiseMin(p.actionLimit);
        examples.push_back({i, x, weight, std::move(reference),
                            std::move(nextReference), action, 0.,
                            regularizedQ.value(concatenate(x, action))});
        examples.back().kl =
            diagonalKL(distribution, examples.back().reference);
        if (!hasSuccessor) break;
        x += world.predict(concatenate(x, sample.u)).head(n);
        weight *= p.temporalDecay;
      }
    }
    return examples;
  }

  void trainOne(bool pretrain) {
    ++updates;
    const auto examples = batch(pretrain);
    std::vector<double> kls, values, priorLosses;
    for (const auto& e : examples) {
      kls.push_back(e.kl);
      values.push_back(e.value);
      const auto prediction = prior.distribution(e.x);
      const auto& target = replay[e.index].plan;
      priorLosses.push_back(p.forwardPriorKL ? diagonalKL(target, prediction)
                                             : diagonalKL(prediction, target));
    }
    const double klScale = robustScale(kls), qScale = robustScale(values);
    const double priorScale = robustScale(priorLosses);
    for (const auto& e : examples) {
      const auto& t = replay[e.index];
      const Vector xu = concatenate(e.x, t.u);
      const Vector modelTarget = concatenate(t.next - e.x, Vector1{t.reward});
      world.accumulate(xu, e.weight * (world.predict(xu) - modelTarget));
      const auto nextPolicy = actor.distribution(t.next);
      const Vector next = concatenate(t.next, nextPolicy.sample(rng)
                                                  .cwiseMax(-p.actionLimit)
                                                  .cwiseMin(p.actionLimit));
      const double discount = t.terminal ? 0. : p.discount;
      q.accumulate(xu, t.reward + discount * targetQ.value(next), e.weight);
      if (pretrain) continue;
      regularizedQ.accumulate(
          xu,
          t.reward +
              discount * (targetRegularizedQ.value(next) -
                          p.klWeight * diagonalKL(nextPolicy, e.nextReference) /
                              klScale),
          e.weight);
      if (p.learnedPrior) {
        const auto current = prior.distribution(e.x);
        const auto& target = t.plan;
        const Vector delta = current.mean - target.mean;
        Vector dm, dl;
        if (p.forwardPriorKL) {
          dm = delta.array() / current.sd.array().square();
          dl = 1. - (target.sd.array().square() + delta.array().square()) /
                        current.sd.array().square();
        } else {
          dm = delta.array() / target.sd.array().square();
          dl = current.sd.array().square() / target.sd.array().square() - 1.;
        }
        prior.accumulate(e.x, dm, dl, e.weight / priorScale);
      }
      const auto current = actor.distribution(e.x);
      const auto& reference = e.reference;
      const Vector dq =
          regularizedQ.gradient(concatenate(e.x, e.action)).tail(m) / qScale;
      const Vector dm = p.klWeight / klScale *
                            (current.mean - reference.mean).array() /
                            reference.sd.array().square() -
                        dq.array();
      const Vector dl =
          p.klWeight / klScale *
              (current.sd.array().square() / reference.sd.array().square() -
               1.) -
          dq.array() * (e.action - current.mean).array() - p.entropyWeight;
      actor.accumulate(e.x, dm, dl, e.weight);
    }
    world.update(p.learningRate, examples.size(), p.useAdam);
    q.net.update(p.learningRate, examples.size(), p.useAdam);
    targetQ.net.blend(q.net, p.targetMix);
    if (!pretrain) {
      actor.net.update(p.learningRate, examples.size(), p.useAdam);
      if (p.learnedPrior)
        prior.net.update(p.learningRate, examples.size(), p.useAdam);
      regularizedQ.net.update(p.learningRate, examples.size(), p.useAdam);
      targetRegularizedQ.net.blend(regularizedQ.net, p.targetMix);
    }
  }
};

PolicyOptimizationMPC::PolicyOptimizationMPC(
    size_t n, size_t m, const PolicyOptimizationMPCParams& params) {
  const auto& p = params;
  constexpr auto maximum =
      static_cast<size_t>(std::numeric_limits<Eigen::Index>::max());
  if (!n || !m || n >= maximum || m > maximum - n || m > maximum / 2 ||
      p.hiddenSize > maximum || p.valueBins > maximum ||
      (p.horizon && p.samples > maximum / p.horizon) || !p.horizon ||
      !p.samples || !p.iterations || !p.hiddenSize || p.valueBins < 2 ||
      !p.replayCapacity || !p.batchSize || !p.elites || !(p.actionLimit > 0) ||
      p.elites > p.samples || p.policySamples > p.samples ||
      !(p.discount >= 0 && p.discount <= 1) ||
      !(p.temporalDecay >= 0 && p.temporalDecay <= 1) ||
      !(p.targetMix > 0 && p.targetMix <= 1) ||
      !(p.minStddev > 0 && p.maxStddev >= p.minStddev) ||
      !std::isfinite(p.maxStddev) || !(p.klWeight >= 0) ||
      !std::isfinite(p.klWeight) || !(p.entropyWeight >= 0) ||
      !std::isfinite(p.entropyWeight) || !(p.valueLimit > 0) ||
      !std::isfinite(p.valueLimit) || !(p.temperature > 0) ||
      !std::isfinite(p.temperature) || !(p.learningRate > 0) ||
      !std::isfinite(p.learningRate))
    throw std::invalid_argument("Invalid PO-MPC dimensions or hyperparameters");
  impl_ = std::make_unique<Impl>(static_cast<Eigen::Index>(n),
                                 static_cast<Eigen::Index>(m), p);
}

PolicyOptimizationMPC::~PolicyOptimizationMPC() = default;

PolicyOptimizationPlan PolicyOptimizationMPC::plan(const Vector& state) {
  return impl_->plan(state, nullptr);
}

PolicyOptimizationPlan PolicyOptimizationMPC::plan(
    const Vector& state, const NonlinearFactorGraph& graph) {
  return impl_->plan(state, std::make_shared<NonlinearFactorGraph>(graph));
}

PolicyOptimizationPlan PolicyOptimizationMPC::planFrom(const Vector& state,
                                                       const Matrix& nominal) {
  if (nominal.rows() != static_cast<Eigen::Index>(impl_->m) ||
      nominal.cols() != static_cast<Eigen::Index>(impl_->p.horizon) ||
      !nominal.allFinite())
    throw std::invalid_argument("Invalid nominal PO-MPC control sequence");
  checkVector(state, impl_->n);
  impl_->warm = nominal;
  return impl_->plan(state, nullptr);
}

PolicyOptimizationPlan PolicyOptimizationMPC::searchFrom(
    const Vector& state, const Matrix& nominal,
    const GaussianNoiseSampler& sampler, size_t policySamples,
    const EliteMPPI& update, size_t iterations) {
  checkVector(state, impl_->n);
  const size_t maximum =
      static_cast<size_t>(std::numeric_limits<Eigen::Index>::max());
  if (nominal.rows() != impl_->m || nominal.cols() != impl_->horizon ||
      !nominal.allFinite() || sampler.dim() != static_cast<size_t>(impl_->m) ||
      !sampler.numSamples() ||
      sampler.numSamples() > maximum / impl_->p.horizon ||
      policySamples > sampler.numSamples() || !iterations)
    throw std::invalid_argument("Invalid PO-MPC search configuration");
  return impl_->search(state, nominal, nullptr, GaussianProposal(sampler),
                       policySamples, update, iterations);
}

std::shared_ptr<PolicyOptimizationMPC> PolicyOptimizationMPC::fork(
    size_t seed) const {
  auto copy =
      std::make_shared<PolicyOptimizationMPC>(impl_->n, impl_->m, impl_->p);
  auto& dest = *copy->impl_;
  dest.world = impl_->world;
  dest.actor.net = impl_->actor.net;
  dest.prior.net = impl_->prior.net;
  dest.q = impl_->q;
  dest.regularizedQ = impl_->regularizedQ;
  dest.targetQ = impl_->targetQ;
  dest.targetRegularizedQ = impl_->targetRegularizedQ;
  dest.observed = impl_->observed;
  dest.rng.seed(seed);
  return copy;
}

Vector PolicyOptimizationMPC::predict(const Vector& state,
                                      const Vector& action) const {
  checkVector(state, impl_->n);
  checkVector(action, impl_->m);
  Vector predicted = impl_->world.predict(concatenate(state, action));
  predicted.head(state.size()) += state;
  return predicted;
}

void PolicyOptimizationMPC::observe(const Vector& state,
                                    const PolicyOptimizationPlan& plan,
                                    double reward, const Vector& nextState,
                                    bool terminal) {
  auto& self = *impl_;
  checkVector(state, self.n);
  checkVector(nextState, self.n);
  checkVector(plan.action, self.m);
  if (!std::isfinite(reward) ||
      plan.mean.rows() != static_cast<Eigen::Index>(self.m) ||
      plan.mean.cols() != static_cast<Eigen::Index>(self.p.horizon) ||
      plan.stddev.rows() != plan.mean.rows() ||
      plan.stddev.cols() != plan.mean.cols() || !plan.mean.allFinite() ||
      !plan.stddev.allFinite() || (plan.stddev.array() <= 0.).any())
    throw std::invalid_argument(
        "Invalid observed reward or planning distribution");
  self.replay.push_back({state,
                         plan.action,
                         nextState,
                         reward,
                         terminal,
                         {plan.mean.col(0), plan.stddev.col(0)},
                         plan.taskCost_});
  if (self.replay.size() > self.p.replayCapacity) self.replay.pop_front();
  ++self.observed;
  if (terminal) reset();
  if (self.observed == self.p.warmupSteps)
    for (size_t i = 0; i < self.p.warmupSteps; ++i) self.trainOne(true);
  if (self.observed > self.p.warmupSteps && self.p.updateInterval &&
      self.observed % self.p.updateInterval == 0)
    train(1);
}

void PolicyOptimizationMPC::train(size_t updates) {
  if (updates && impl_->replay.empty())
    throw std::logic_error("PO-MPC training requires an observed transition");
  for (size_t i = 0; i < updates; ++i) impl_->trainOne(false);
}
void PolicyOptimizationMPC::reset() {
  impl_->warm.setZero();
  if (!impl_->replay.empty()) impl_->replay.back().endsEpisode = true;
}
size_t PolicyOptimizationMPC::replaySize() const {
  return impl_->replay.size();
}
Vector PolicyOptimizationMPC::policyMean(const Vector& state) const {
  checkVector(state, impl_->n);
  return impl_->actor.distribution(state).mean;
}
double PolicyOptimizationMPC::bootstrapValue(const Vector& state,
                                             const Vector& action) const {
  checkVector(state, impl_->n);
  checkVector(action, impl_->m);
  return impl_->q.value(concatenate(state, action));
}
double PolicyOptimizationMPC::regularizedValue(const Vector& state,
                                               const Vector& action) const {
  checkVector(state, impl_->n);
  checkVector(action, impl_->m);
  return impl_->regularizedQ.value(concatenate(state, action));
}

}  // namespace gtsam
