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
  gtsam::Vector logWeightOffsets;
};

class GaussianProposalOptions {
  GaussianProposalOptions();
  gtsam::Matrix stddev;
  double decay;
  double zeroMeanFraction;
  bool includeNominal;
  gtsam::Vector likelihoodCoefficients;
};

virtual class GaussianProposal {
  GaussianProposal(const gtsam::Matrix& covariance, size_t samples, size_t seed = 42);
  GaussianProposal(const gtsam::Matrix& covariance, size_t samples, size_t seed,
                   gtsam::GaussianProposalOptions options);
  @pybind_lambda
  gtsam::ControlDistribution initialize(const gtsam::Matrix& nominal) const;
  gtsam::ControlBatch sample(const gtsam::ControlDistribution& state, const gtsam::Values& initial,
                             double dt, bool includeIncumbent = false) const;
  void update(gtsam::ControlDistribution& state, const gtsam::ControlBatch& batch,
              const gtsam::Vector& weights) const;
};

virtual class ColoredNoiseProposal : gtsam::GaussianProposal {
  ColoredNoiseProposal(const gtsam::Matrix& covariance, size_t samples, double exponent,
                       size_t seed = 42);
  ColoredNoiseProposal(const gtsam::Matrix& covariance, size_t samples, double exponent,
                       size_t seed, gtsam::GaussianProposalOptions options);
};

virtual class NormalLognormalProposal : gtsam::GaussianProposal {
  NormalLognormalProposal(const gtsam::Matrix& covariance, size_t samples, double logStddev,
                          size_t seed = 42);
  NormalLognormalProposal(const gtsam::Matrix& covariance, size_t samples, double logStddev,
                          size_t seed, gtsam::GaussianProposalOptions options);
};

class MotionPrimitiveOptions {
  MotionPrimitiveOptions();
  gtsam::Vector amplitude;
  double coloredFraction;
  double shiftedFraction;
  double coloredScale;
  double shiftedScale;
  double exponent;
  size_t fixedPrefix;
};

virtual class PiecewiseLinearProposal : gtsam::ColoredNoiseProposal {
  PiecewiseLinearProposal(const gtsam::Matrix& covariance, size_t samples, size_t segments, size_t seed = 42);
  PiecewiseLinearProposal(const gtsam::Matrix& covariance, size_t samples, size_t segments, size_t seed,
                          gtsam::GaussianProposalOptions options);
  PiecewiseLinearProposal(const gtsam::Matrix& covariance, size_t samples, size_t segments,
                          gtsam::MotionPrimitiveOptions mixture, size_t seed = 42);
};

class SmoothControlProposalState : gtsam::ControlDistribution {
  gtsam::Matrix derivativeMean;
};

class SmoothControlProposal {
  SmoothControlProposal(const gtsam::Matrix& covariance, size_t samples, size_t seed = 42);
  SmoothControlProposal(const gtsam::Matrix& covariance, size_t samples, size_t seed,
                        gtsam::GaussianProposalOptions options);
  @pybind_lambda
  gtsam::SmoothControlProposalState initialize(const gtsam::Matrix& nominal) const;
};

#include <gtsam/planning/SamplingAlgorithms.h>

double valueAtRisk(const gtsam::Vector& costs, double alpha);
double conditionalValueAtRisk(const gtsam::Vector& costs, double alpha);

virtual class SamplingAlgorithm {
  bool includesIncumbent() const;
  bool fitsSpread() const;
  double minStddev() const;
  @pybind_lambda
  void apply(const gtsam::GaussianProposal& proposal, gtsam::ControlDistribution& search,
             const gtsam::ControlBatch& batch, const gtsam::Vector& costs) const;
};

virtual class MPPI : gtsam::SamplingAlgorithm {
  MPPI(double inverseTemperature = 1.0);
  @pybind_lambda
  gtsam::Vector weights(const gtsam::Vector& costs) const;
  gtsam::Vector weights(const gtsam::Vector& costs, const gtsam::Vector& logWeightOffsets) const;
};

virtual class PredictiveSampling : gtsam::SamplingAlgorithm {
  PredictiveSampling();
  gtsam::Vector weights(const gtsam::Vector& costs) const;
};

virtual class CEM : gtsam::SamplingAlgorithm {
  CEM(size_t elites, double minStddev = 0.001);
  gtsam::Vector weights(const gtsam::Vector& costs) const;
};

#include <gtsam/planning/MPPITracking.h>

// The callable tracking operations are bound with the Python dynamics callbacks.
class MPPITracking {
  MPPITracking(const gtsam::Matrix& covariance, size_t samples, double inverseTemperature, size_t seed = 42);
  MPPITracking(const gtsam::Matrix& covariance, size_t samples, double inverseTemperature, size_t seed,
               const gtsam::GaussianProposalOptions& options);
  @pybind_lambda
  void setControlHistory(const gtsam::Matrix& nominal);
  void setControlHistory(const gtsam::Matrix& nominal, const gtsam::Matrix& actual);
};

virtual class TsallisMPPI : gtsam::SamplingAlgorithm {
  TsallisMPPI(double cutoff, double shape, double inverseTemperature = 1.0);
  gtsam::Vector weights(const gtsam::Vector& costs) const;
};

class MPPIPlans {
  gtsam::Values nominal;
  gtsam::Values actual;
  size_t candidate;
};

gtsam::Matrix smoothControlTrajectory(const gtsam::Matrix& controls, const gtsam::Matrix& history);

#include <gtsam/planning/TrackingFeedback.h>

class DDPTracking {
  DDPTracking(const gtsam::Matrix& statePrecision, const gtsam::Matrix& controlPrecision,
              const gtsam::Matrix& terminalPrecision);
  gtsam::Matrix optimize(const gtsam::Vector& initial, const gtsam::Matrix& targetStates,
                         const gtsam::Matrix& targetControls,
                         const gtsam::DDPTracking::Transition& transition,
                         size_t iterations,
                         const gtsam::Vector& lower = gtsam::Vector(),
                         const gtsam::Vector& upper = gtsam::Vector());
  // A subsequent solve can resize or replace the stored trajectory and gains.
  @pybind_lambda
  gtsam::Matrix states() const;
  @pybind_lambda
  gtsam::Matrix gain(size_t step) const;
};

gtsam::Vector contractionFeedback(const gtsam::Vector& error, const gtsam::Matrix& metric,
                                  const gtsam::Matrix& inputJacobian,
                                  const gtsam::Vector& driftDifference, double rate);

#include <gtsam/planning/EliteMPPI.h>

virtual class EliteMPPI : gtsam::SamplingAlgorithm {
  EliteMPPI(size_t elites, double inverseTemperature = 1.0, double minStddev = 0.001);
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
               const gtsam::SamplingAlgorithm& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::VectorField& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::SamplingAlgorithm& algorithm, size_t iterations) const;
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
               const gtsam::SamplingAlgorithm& algorithm, gtsam::ControlDistribution& search) const;
  @pybind_lambda
  gtsam::Values optimize(const gtsam::NonlinearFactorGraph& cost, const gtsam::Values& initial,
                         const gtsam::Pose2Transition& dynamics, const gtsam::GaussianProposal& proposal,
                         const gtsam::SamplingAlgorithm& algorithm, size_t iterations) const;
};

template<State={gtsam::Pose3}, Control={gtsam::Vector}>
class SamplingPlanner {
  SamplingPlanner(gtsam::KeyVector stateKeys, gtsam::KeyVector controlKeys, double dt,
                  bool parallelCandidateScoring = false);
  gtsam::Matrix nominalControls(const gtsam::Values& initial) const;
};

}  // namespace gtsam
