/**
 * @file  TGEqFScenarioExample.h
 * @brief TG-EqF adapter for the shared trajectory-example runner.
 *
 * The run loop, CLI, IMU simulation, CSV format and error reporting are shared
 * with the other filter examples (examples_common/FilterScenarioRunner.h); only
 * what is TG-EqF-specific lives here. Process noise comes from
 * examples_common/ImuProcessNoise.h and inputNoiseCov().
 */
#pragma once

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam_unstable/examples_common/FilterScenarioRunner.h>
#include <gtsam_unstable/examples_common/ImuProcessNoise.h>
#include <gtsam_unstable/examples_common/ScenarioHarness.h>
#include <gtsam_unstable/examples_common/TrajectoryCsv.h>
#include <gtsam_unstable/tg_eqf/TGEqF.h>

#include <ostream>
#include <string>

namespace gtsam::tgeqf::examples {

// Run configuration and reporting are shared with the other filter examples.
// The shared options only describe the physical state: bv has no physical
// truth, so the initial estimate leaves it at zero (the last 3 entries of
// State::b) and its initial covariance comes from the filter itself.
using imu_scenarios::parseRunOptions;
using imu_scenarios::RunOptions;
using imu_scenarios::RunSummary;

/// Satisfies the adapter contract in examples_common/FilterScenarioRunner.h.
class ScenarioAdapter {
 public:
  static constexpr int kDim = 18;
  using Filter = TGEqF;
  using TrueState = State;

  explicit ScenarioAdapter(const RunOptions& opts)
      : noise_(imu_scenarios::toFilterImuNoise<ImuNoise>(
            imu_scenarios::resolvedProcessNoise(opts))),
        gravity_(imu_scenarios::navGravity()),
        reset_step_(opts.tg_eqf_reset_step),
        depth_direct_(opts.tg_eqf_depth_direct),
        dvl_c0_(opts.tg_eqf_dvl_c0),
        cov_format_(opts.log_full_covariance
                        ? imu_scenarios::CovarianceFormat::FullUpperTriangle
                        : imu_scenarios::CovarianceFormat::BlockDiagonal) {}

  /// The virtual bias bv is estimate-only: it has no ground-truth counterpart.
  std::string csvHeader() const {
    return imu_scenarios::trajectoryCsvHeader(
        {"v", "p", "bg", "ba"}, {"v", "p", "bg", "ba", "bv"},
        {"R", "v", "p", "bg", "ba", "bv"}, cov_format_);
  }

  /// Pin the origin xi0 at the manifold identity, so the error dynamics are
  /// always linearized about the same point, and carry the perturbed initial
  /// belief in the group element: phi(X0, xi0) is that belief. P0 covers the
  /// physical state only; TGEqF::initialCovariance() adds the virtual-bias
  /// block.
  Filter makeFilter(const imu_scenarios::InitialEstimate& initial,
                    const imu_scenarios::PhysicalStateCovariance& P0) const {
    Eigen::Matrix<double, 3, 2> vp;
    vp.col(0) = initial.nav.velocity();
    vp.col(1) = initial.nav.position();
    State xi_hat0;
    xi_hat0.T = ExtendedPose3<2>(initial.nav.attitude(), vp);
    xi_hat0.b.segment<3>(0) = initial.bias_gyro;
    xi_hat0.b.segment<3>(3) = initial.bias_accel;
    // b_v stays zero: the virtual bias has no physical truth to perturb.
    Filter filter(phiInverse(State::identity(), xi_hat0),
                  Filter::initialCovariance(P0));
    filter.set_reset_step(reset_step_);
    filter.set_dvl_jacobian(dvl_c0_ ? Filter::DvlJacobian::C0
                                    : Filter::DvlJacobian::Cstar);
    return filter;
  }

  TrueState trueState(const gtsam::NavState& gt, const Vector3& true_bg,
                      const Vector3& true_ba) const {
    Eigen::Matrix<double, 3, 2> vp;
    vp.col(0) = gt.velocity();
    vp.col(1) = gt.position();
    State xi;
    xi.T = ExtendedPose3<2>(gt.attitude(), vp);
    xi.b.segment<3>(0) = true_bg;
    xi.b.segment<3>(3) = true_ba;
    // b_v (segment<3>(6)) stays zero: no physical truth.
    return xi;
  }

  void propagate(Filter& filter, const imu_scenarios::ImuMeasurement& imu,
                 double dt) const {
    filter.propagate(imu.omega, imu.accel, gravity_, noise_, dt);
  }

  void updatePosition(Filter& filter,
                      const imu_scenarios::PositionMeasurement& z) const {
    filter.update_position(z.position, z.covariance);
  }

  void updateDvl(Filter& filter, const imu_scenarios::DvlMeasurement& z) const {
    filter.update_dvl(z.body_velocity, z.covariance);
  }

  void updateDepth(Filter& filter,
                   const imu_scenarios::DepthMeasurement& z) const {
    if (depth_direct_) {
      filter.update_depth_direct(z.depth, z.covariance);
    } else {
      filter.update_depth(z.depth, z.covariance);
    }
  }

  /// Truth, estimate, tangent error and covariance. The error is
  /// eps = Local(xi_ref, phi(g^{-1}, xi_true)) and the covariance is the
  /// matching errorCovariance(), both in the fixed origin chart.
  void writeCsvRow(std::ostream& out, double t, const TrueState& xi_true,
                   const Filter& filter) const {
    const State& xi_hat = filter.state();
    imu_scenarios::CsvRow row(out, t);
    row.quat(xi_true.T.rotation())
        .v3(xi_true.T.x(0))
        .v3(xi_true.T.x(1))
        .v3(xi_true.b.segment<3>(0))
        .v3(xi_true.b.segment<3>(3));
    row.quat(xi_hat.T.rotation())
        .v3(xi_hat.T.x(0))
        .v3(xi_hat.T.x(1))
        .v3(xi_hat.b.segment<3>(0))
        .v3(xi_hat.b.segment<3>(3))
        .v3(xi_hat.b.segment<3>(6));
    row.tangent<kDim>(filter.errorStateVector(xi_true));
    if (cov_format_ == imu_scenarios::CovarianceFormat::FullUpperTriangle) {
      row.covarianceFullUpper<kDim>(filter.errorCovariance());
    } else {
      row.covarianceBlocks<kDim>(filter.errorCovariance());
    }
    row.end();
  }

 private:
  ImuNoise noise_;
  gtsam::Vector3 gravity_;
  bool reset_step_;
  bool depth_direct_;
  bool dvl_c0_;
  imu_scenarios::CovarianceFormat cov_format_;
};

inline RunSummary runScenario(const gtsam::Scenario& scenario,
                              const RunOptions& opts,
                              const std::string& scenario_name) {
  return imu_scenarios::runFilterScenario(ScenarioAdapter(opts), scenario, opts,
                                          scenario_name);
}

}  // namespace gtsam::tgeqf::examples
