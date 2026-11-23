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
};

class ControlBatch {
  gtsam::Matrix controls;
};

class GaussianProposal {
  GaussianProposal(const gtsam::Matrix& covariance, size_t samples, uint64_t seed = 42);
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

#include <gtsam/planning/wrap/PlannerBindings.h>

template<State, Control>
class SamplingPlanner {
  SamplingPlanner(gtsam::KeyVector stateKeys, gtsam::KeyVector controlKeys, double dt,
                  bool parallelCandidateScoring = false);
  gtsam::Matrix nominalControls(const gtsam::Values& initial) const;
};

typedef gtsam::SamplingPlanner<gtsam::Vector, gtsam::Vector> SamplingPlannerVectorVector;


class Vector4Point2Field {};
class Pose2Point2Transition {};

template<State, Control, Dynamics>
class WrappedSamplingPlanner {
  WrappedSamplingPlanner(gtsam::KeyVector stateKeys, gtsam::KeyVector controlKeys, double dt,
                         bool parallelCandidateScoring = false);
  gtsam::Matrix nominalControls(const gtsam::Values& initial) const;
  @pybind_lambda
  gtsam::Values rollout(const gtsam::Values& initial, const gtsam::Matrix& controls,
                        const Dynamics& dynamics) const;
  @pybind_lambda
  gtsam::Vector rolloutAndScore(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                                const gtsam::Matrix& controls, const Dynamics& dynamics) const;
  @pybind_lambda
  void iterate(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
               const Dynamics& dynamics, const gtsam::GaussianProposal& proposal,
               const gtsam::MPPI& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const Dynamics& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::MPPI& algorithm, size_t iterations) const;
};

typedef gtsam::WrappedSamplingPlanner<gtsam::Vector4, gtsam::Point2, gtsam::Vector4Point2Field>
    SamplingPlannerVector4Point2;
typedef gtsam::WrappedSamplingPlanner<gtsam::Pose2, gtsam::Point2, gtsam::Pose2Point2Transition>
    SamplingPlannerPose2Point2;

#include <gtsam/planning/VanillaMPPIController.h>

class VanillaMPPIParams {
  VanillaMPPIParams();
  size_t numSamples;
  gtsam::Matrix controlCovariance;
  uint64_t seed;
  size_t iterations;
  double inverseTemperature;
  bool parallelCandidateScoring;
};

}  // namespace gtsam
