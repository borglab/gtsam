/**
 * @file  MEKFScenarioExample.cpp
 * @brief MEKF on a simulated IMU trajectory with optional position, DVL and
 *        depth aiding. Pick the trajectory with --scenario (Vertical,
 *        TypicalNavigation or WaypointSpline).
 *
 * The run loop, CLI, IMU simulation, CSV format and error reporting are shared
 * with the other filter examples (scenario_harness/FilterScenarioRunner.h); only
 * what is MEKF-specific lives here.
 */

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam_unstable/examples/scenario_harness/FilterScenarioRunner.h>
#include <gtsam_unstable/examples/scenario_harness/ImuProcessNoise.h>
#include <gtsam_unstable/examples/scenario_harness/ScenarioHarness.h>
#include <gtsam_unstable/examples/scenario_harness/ScenarioMain.h>
#include <gtsam_unstable/examples/scenario_harness/TrajectoryCsv.h>
#include <gtsam_unstable/navigation/MEKF.h>

#include <ostream>
#include <string>

namespace gtsam::mekf::examples {

// Run configuration and reporting are shared with the other filter examples.
using imu_scenarios::parseRunOptions;
using imu_scenarios::RunOptions;
using imu_scenarios::RunSummary;

/// Satisfies the adapter contract in scenario_harness/FilterScenarioRunner.h.
class ScenarioAdapter {
 public:
  static constexpr int kDim = MEKFState::dimension;
  using Filter = MEKF;
  using TrueState = MEKFState;

  explicit ScenarioAdapter(const RunOptions& opts)
      : noise_(imu_scenarios::toFilterImuNoise<ImuNoise>(
            imu_scenarios::resolvedProcessNoise(opts))),
        gravity_(imu_scenarios::navGravity()),
        depth_direct_(opts.depth_direct),
        cov_format_(opts.log_full_covariance
                        ? imu_scenarios::CovarianceFormat::FullUpperTriangle
                        : imu_scenarios::CovarianceFormat::BlockDiagonal) {}

  std::string csvHeader() const {
    return imu_scenarios::trajectoryCsvHeader(
        {"v", "p", "bg", "ba"}, {"v", "p", "bg", "ba"},
        {"R", "v", "p", "bg", "ba"}, cov_format_);
  }

  /// Start at the perturbed initial belief, biases included. Every MEKF state
  /// is physical, so P0 is used as given.
  Filter makeFilter(const imu_scenarios::InitialEstimate& initial,
                    const imu_scenarios::PhysicalStateCovariance& P0) const {
    const MEKFState X0(initial.nav.attitude(), initial.nav.velocity(),
                       initial.nav.position(), initial.bias_gyro,
                       initial.bias_accel);
    return Filter(X0, P0);
  }

  TrueState trueState(const gtsam::NavState& gt, const Eigen::Vector3d& true_bg,
                      const Eigen::Vector3d& true_ba) const {
    return MEKFState(gt.attitude(), gt.velocity(), gt.position(), true_bg,
                     true_ba);
  }

  void propagate(Filter& filter, const imu_scenarios::ImuMeasurement& imu,
                 double dt) const {
    filter.propagate(imu.omega, imu.accel, gravity_, noise_, dt);
  }

  void updatePosition(Filter& filter,
                      const imu_scenarios::PositionMeasurement& z) const {
    filter.updatePosition(z.position, z.covariance);
  }

  void updateDvl(Filter& filter, const imu_scenarios::DvlMeasurement& z) const {
    filter.updateDvl(z.body_velocity, z.covariance);
  }

  void updateDepth(Filter& filter,
                   const imu_scenarios::DepthMeasurement& z) const {
    if (depth_direct_) {
      filter.updateDepthDirect(z.depth, z.covariance);
    } else {
      filter.updateDepth(z.depth, z.covariance,
                         imu_scenarios::kDepthHorizontalVariance);
    }
  }

  /// Truth, estimate, tangent error and covariance, all in the MEKF's local
  /// chart (right/body multiplicative attitude, additive vectors).
  void writeCsvRow(std::ostream& out, double t, const TrueState& X_true,
                   const Filter& filter) const {
    const MEKFState& X_hat = filter.state();
    imu_scenarios::CsvRow row(out, t);
    row.quat(X_true.R)
        .v3(X_true.v)
        .v3(X_true.p)
        .v3(X_true.b_gyro)
        .v3(X_true.b_accel);
    row.quat(X_hat.R)
        .v3(X_hat.v)
        .v3(X_hat.p)
        .v3(X_hat.b_gyro)
        .v3(X_hat.b_accel);
    row.tangent<kDim>(filter.errorStateVector(X_true));
    if (cov_format_ == imu_scenarios::CovarianceFormat::FullUpperTriangle) {
      row.covarianceFullUpper<kDim>(filter.covariance());
    } else {
      row.covarianceBlocks<kDim>(filter.covariance());
    }
    row.end();
  }

 private:
  ImuNoise noise_;
  Eigen::Vector3d gravity_;
  bool depth_direct_;
  imu_scenarios::CovarianceFormat cov_format_;
};

inline RunSummary runScenario(const gtsam::Scenario& scenario,
                              const RunOptions& opts,
                              const std::string& scenario_name) {
  return imu_scenarios::runFilterScenario(ScenarioAdapter(opts), scenario, opts,
                                          scenario_name);
}

}  // namespace gtsam::mekf::examples

int main(int argc, char* argv[]) {
  return imu_scenarios::exampleMain(argc, argv, "MEKFScenarioExample", "mekf",
                                    gtsam::mekf::examples::runScenario);
}
