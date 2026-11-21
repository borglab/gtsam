/**
 * @file  FilterScenarioRunner.h
 * @brief The one trajectory-example loop, shared by tg_eqf / tfg_inekf / mekf.
 *
 * The loop, the RNG draw order and the logged quantities are identical for
 * every filter; only construction, propagation and the update calls differ.
 * Those live in a small per-filter adapter (see <filter>/examples/
 * <Filter>ScenarioExample.h), which stays next to its filter so this header
 * remains filter-agnostic.
 *
 * Adapter contract:
 * @code
 *   static constexpr int kDim;   // tangent dimension (15, 18, ...)
 *   using Filter;                // the filter type
 *   using TrueState;             // state type holding ground truth + biases
 *
 *   std::string csvHeader() const;
 *   Filter makeFilter(const InitialEstimate& initial,
 *                     const PhysicalStateCovariance& P0) const;
 *   TrueState trueState(const gtsam::NavState& gt, const Eigen::Vector3d& bg,
 *                       const Eigen::Vector3d& ba) const;
 *   void propagate(Filter&, const ImuMeasurement&, double dt) const;
 *   void updatePosition(Filter&, const PositionMeasurement&) const;
 *   void updateDvl(Filter&, const DvlMeasurement&) const;
 *   void updateDepth(Filter&, const DepthMeasurement&) const;
 *   void writeCsvRow(std::ostream&, double t, const TrueState&,
 *                    const Filter&) const;
 * @endcode
 * The adapter is constructed from RunOptions, so anything it needs per run
 * (process noise, gravity) is set up once before the loop starts. P0 covers
 * only the 15 physical states; a filter with extra estimate-only states
 * (kDim > 15) fills those blocks in itself.
 */
#pragma once

#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/navigation/ScenarioRunner.h>
#include <gtsam_unstable/examples_common/IMUScenarios.h>
#include <gtsam_unstable/examples_common/ScenarioHarness.h>

#include <cmath>
#include <cstddef>
#include <fstream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

namespace imu_scenarios {

/// Drive `scenario` through the adapter's filter, logging the trajectory CSV to
/// opts.output_path and returning the run's error summary.
template <class Adapter>
inline RunSummary runFilterScenario(const Adapter& adapter,
                                    const gtsam::Scenario& scenario,
                                    const RunOptions& opts,
                                    const std::string& scenario_name) {
  validateRunOptions(opts);

  std::ofstream csv(opts.output_path);
  if (!csv) {
    throw std::runtime_error("Cannot open output file: " + opts.output_path);
  }
  csv << adapter.csvHeader();

  auto params = std::make_shared<gtsam::PreintegrationParams>(navGravity());
  std::mt19937 rng(opts.seed);

  // Ground truth is the clean scenario: `scenario` is what the IMU is driven
  // through and what `gt` refers to below. The filter instead starts away from
  // it, at a belief drawn from the init-*-sigma options, so P0 describes an
  // error that actually occurs. The true biases stay at their configured
  // initial values and random-walk from there; the filter knows none of this.
  const gtsam::ScenarioRunner runner(scenario, params, opts.dt);
  const InitialEstimate initial =
      sampleInitialEstimate(rng, opts, scenario.navState(0.0));
  ImuSimulator sim(opts, rng, opts.gyro_bias, opts.accel_bias);

  auto filter = adapter.makeFilter(initial, initialCovariance(opts));

  PeriodicTrigger pos_updates(opts.pos_rate, opts.aiding_start_time);
  PeriodicTrigger dvl_updates(opts.dvl_rate, opts.aiding_start_time);
  PeriodicTrigger depth_updates(opts.depth_rate, opts.aiding_start_time);
  RunSummaryAccumulator errors;

  const size_t num_steps =
      static_cast<size_t>(std::llround(opts.duration / opts.dt));
  double t = 0.0;

  // The filter state is at time t throughout the loop body, matching gt: update
  // and log first, propagate to t + dt last.
  for (size_t k = 0; k <= num_steps; ++k) {
    const gtsam::NavState gt = scenario.navState(t);
    const ImuMeasurement imu = sim.sample(runner, t);

    if (pos_updates.due(t)) {
      adapter.updatePosition(filter,
                             sim.samplePosition(gt, opts.pos_noise_sigma));
    }
    if (dvl_updates.due(t)) {
      adapter.updateDvl(filter, sim.sampleDvl(gt, opts.dvl_noise_sigma));
    }
    if (depth_updates.due(t)) {
      adapter.updateDepth(filter, sim.sampleDepth(gt, opts.depth_noise_sigma));
    }

    // Log every Nth step (and always the final one) to bound MC file sizes.
    if (k % static_cast<size_t>(opts.log_decim) == 0 || k == num_steps) {
      adapter.writeCsvRow(
          csv, t, adapter.trueState(gt, sim.gyroBias(), sim.accelBias()),
          filter);
    }
    errors.add(gt, filter.attitude(), filter.velocity(), filter.position());

    if (k == num_steps) {
      break;
    }

    adapter.propagate(filter, imu, opts.dt);
    sim.randomWalkBias();
    t += opts.dt;
  }

  const RunSummary summary = errors.summary();
  printRunSummary(summary, scenario_name, opts.output_path);
  return summary;
}

}  // namespace imu_scenarios
