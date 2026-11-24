// Sampling-based planning. Authors: Frank Dellaert and Akash Ratheesh.
namespace gtsam {

#include <gtsam/planning/GaussianNoiseSampler.h>

class GaussianNoiseSampler {
  GaussianNoiseSampler(const gtsam::Matrix& covariance, size_t numSamples, uint64_t seed);
  size_t dim() const;
  size_t numSamples() const;
  uint64_t seed() const;
  uint64_t stepCounter() const;
  // The sampler reuses its buffers on the next draw. Give callers an owned copy.
  @pybind_lambda
  gtsam::Matrix sampleNoise(size_t horizon) const;
  @pybind_lambda
  gtsam::Matrix sampleControls(const gtsam::Matrix& nominalControls) const;
};

#include <gtsam/planning/ControlProposals.h>

class ControlDistribution {
  gtsam::Matrix mean;
  gtsam::Matrix stddev;
  size_t iteration;
};

class ControlBatch {
  gtsam::Matrix controls;
};

class GaussianProposalOptions {
  GaussianProposalOptions();
  bool includeNominal;
};

class GaussianProposal {
  GaussianProposal(const gtsam::Matrix& covariance, size_t samples, uint64_t seed = 42);
  GaussianProposal(const gtsam::Matrix& covariance, size_t samples, uint64_t seed,
                   gtsam::GaussianProposalOptions options);
  @pybind_lambda
  gtsam::ControlDistribution initialize(const gtsam::Matrix& nominal) const;
  gtsam::ControlBatch sample(const gtsam::ControlDistribution& state, const gtsam::Values& initial,
                             double dt, bool includeIncumbent = false) const;
  void update(gtsam::ControlDistribution& state, const gtsam::ControlBatch& batch,
              const gtsam::Vector& weights) const;
};

#include <gtsam/planning/SamplingAlgorithms.h>

class MPPI {
  MPPI(double inverseTemperature = 1.0);
  gtsam::Vector weights(const gtsam::Vector& costs) const;
  @pybind_lambda
  void apply(const gtsam::GaussianProposal& proposal, gtsam::ControlDistribution& search,
             const gtsam::ControlBatch& batch, const gtsam::Vector& costs) const;
};

class CEM {
  CEM(size_t elites, double minStddev = 1e-3);
  gtsam::Vector weights(const gtsam::Vector& costs) const;
};

class PredictiveSampling {
  PredictiveSampling();
  gtsam::Vector weights(const gtsam::Vector& costs) const;
};

#include <gtsam/planning/wrap/PlannerBindings.h>

class VectorField {};
class Pose2Transition {};

// Each native model supplies a different callable signature. Spell out the
// overloads so MATLAB can dispatch on its concrete dynamics object.
template<State={gtsam::Vector}, Control={gtsam::Vector}>
class SamplingPlanner {
  SamplingPlanner(gtsam::KeyVector stateKeys, gtsam::KeyVector controlKeys, double dt,
                  bool parallelCandidateScoring = false);
  gtsam::Matrix nominalControls(const gtsam::Values& initial) const;
  @pybind_lambda
  gtsam::Values rollout(const gtsam::Values& initial, const gtsam::Matrix& controls,
                        const gtsam::VectorField& dynamics) const;
  @pybind_lambda
  gtsam::Vector rolloutAndScore(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                                const gtsam::Matrix& controls, const gtsam::VectorField& dynamics) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::MPPI& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::MPPI& algorithm, size_t iterations) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::CEM& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::CEM& algorithm, size_t iterations) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::PredictiveSampling& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::PredictiveSampling& algorithm, size_t iterations) const;
};

template<State={gtsam::Pose2}, Control={gtsam::Vector}>
class SamplingPlanner {
  SamplingPlanner(gtsam::KeyVector stateKeys, gtsam::KeyVector controlKeys, double dt,
                  bool parallelCandidateScoring = false);
  gtsam::Matrix nominalControls(const gtsam::Values& initial) const;
  @pybind_lambda
  gtsam::Values rollout(const gtsam::Values& initial, const gtsam::Matrix& controls,
                        const gtsam::Pose2Transition& dynamics) const;
  @pybind_lambda
  gtsam::Vector rolloutAndScore(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                                const gtsam::Matrix& controls, const gtsam::Pose2Transition& dynamics) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::MPPI& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::MPPI& algorithm, size_t iterations) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::CEM& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::CEM& algorithm, size_t iterations) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::PredictiveSampling& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::PredictiveSampling& algorithm, size_t iterations) const;
};

}  // namespace gtsam
