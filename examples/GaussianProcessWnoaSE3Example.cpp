/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianProcessWnoaSE3Example.cpp
 * @brief Gaussian Process interpolation example on SE(3) with WNOA factors.
 * Based on the python example
 * python/examples/GaussianProcessWnoaInterpolationSE3.ipynb
 * @date May 9 2026
 * @authors Connor Holmes
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaFactorGraph.h>
#include <gtsam/nonlinear/WnoaInterpFactor.h>
#include <gtsam/nonlinear/WnoaStateData.h>

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using Pose = gtsam::Pose3;
using Velocity = Eigen::Matrix<double, 6, 1>;
using State = gtsam::StateData;
using WnoaFactorGraph = gtsam::WnoaFactorGraph<Pose>;

constexpr int kDoF = 6;

State makeState(int index, double time) {
  return {gtsam::Symbol('x', index), gtsam::Symbol('v', index), time};
}

Velocity makeTwist(double time, double total_time) {
  Velocity twist;
  twist << 0.0, 0.0, -0.3 * std::sin(2.0 * M_PI * time / total_time), -1.0, 0.0,
      0.0;
  return twist;
}

Velocity sampleGaussianDiagonal(const Eigen::Matrix<double, kDoF, kDoF>& cov,
                                std::mt19937& rng) {
  std::normal_distribution<double> standard_normal(0.0, 1.0);
  Velocity sample;
  for (int i = 0; i < kDoF; ++i) {
    sample(i) = std::sqrt(cov(i, i)) * standard_normal(rng);
  }
  return sample;
}

Velocity poseErrorVector(const Pose& estimate, const Pose& truth) {
  return Pose::Logmap(truth.between(estimate));
}

double rmsPoseError(const gtsam::Values& estimate,
                    const gtsam::Values& ground_truth,
                    const std::vector<State>& states) {
  double sum_squared_error = 0.0;
  for (const State& state : states) {
    const Velocity error = poseErrorVector(estimate.at<Pose>(state.pose),
                                           ground_truth.at<Pose>(state.pose));
    sum_squared_error += error.squaredNorm();
  }
  return std::sqrt(sum_squared_error / static_cast<double>(states.size()));
}

gtsam::Values optimizeGraph(const gtsam::NonlinearFactorGraph& graph,
                            const gtsam::Values& initial) {
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosity("ERROR");
  return gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
}

void printStateSummary(const std::string& label, const gtsam::Values& result,
                       const gtsam::Values& ground_truth,
                       const std::vector<State>& states) {
  std::cout << label << "\n";
  std::cout << "  states: " << states.size() << "\n";
  std::cout << "  rms pose error: "
            << rmsPoseError(result, ground_truth, states) << "\n";
}

}  // namespace

int main() {
  // Setup: Define trajectory parameters for ground truth generation.
  // We estimate at K=7 intervals but interpolate at K_sample=5 times higher
  // rate.
  constexpr int kIntervals = 7;
  constexpr int kSample = 5;
  constexpr double kTotalTime = 10.0;
  constexpr double kDt = kTotalTime / (kIntervals * kSample);
  constexpr int kAllIntervals = kIntervals * kSample;

  std::mt19937 rng(42);

  // Define measurement noise model and WNOA process noise covariance.
  // These define the prior belief about measurement and process uncertainties.
  Eigen::Matrix<double, kDoF, kDoF> measurement_cov =
      1e-2 * (Eigen::Matrix<double, kDoF, 1>() << 0.1, 0.1, 0.1, 1.0, 1.0, 1.0)
                 .finished()
                 .asDiagonal();
  const auto noise_model =
      gtsam::noiseModel::Gaussian::Covariance(measurement_cov);

  Eigen::Matrix<double, kDoF, 1> q_psd_diag_fixed;
  q_psd_diag_fixed << 0.008 * 0.1, 0.008 * 0.1, 0.008 * 0.1, 0.008 * 1.0,
      0.008 * 1.0, 0.008 * 1.0;
  const gtsam::Vector q_psd_diag = q_psd_diag_fixed;

  // Generate ground truth trajectory at full interpolation rate.
  // States are separated into estimated and interpolated sets for later use.
  gtsam::Values values_gt;
  std::vector<State> all_states;
  std::vector<State> estimated_states_vec;
  std::vector<State> interpolated_states_vec;
  std::set<State> estimated_states;
  std::set<State> interpolated_states;

  Pose pose = Pose::Expmap(Velocity::Zero());
  for (int k = 0; k <= kAllIntervals; ++k) {
    const double time = k * kDt;
    const Velocity twist = makeTwist(time, kTotalTime);
    if (k == 0) {
      pose = Pose::Expmap(Velocity::Zero());
    } else {
      pose = Pose::Expmap(twist * kDt).compose(pose);
    }

    const State state = makeState(k, time);
    values_gt.insert(state.pose, pose);
    values_gt.insert(state.velocity, twist);
    all_states.push_back(state);

    if (k % kSample == 0) {
      estimated_states.insert(state);
      estimated_states_vec.push_back(state);
    } else {
      interpolated_states.insert(state);
      interpolated_states_vec.push_back(state);
    }
  }

  // Build factor graph with measurements and WNOA motion priors on estimated
  // states only. WNOA (White Noise on Acceleration) priors smooth the
  // trajectory with GP interpretation.
  gtsam::NonlinearFactorGraph graph;
  std::vector<Pose> measurements;
  for (size_t i = 0; i < estimated_states_vec.size(); ++i) {
    const State& state = estimated_states_vec[i];
    const Velocity noise = sampleGaussianDiagonal(measurement_cov, rng);
    const Pose measurement =
        Pose::Expmap(noise).compose(values_gt.at<Pose>(state.pose));
    measurements.push_back(measurement);
    graph.add(std::make_shared<gtsam::PriorFactor<Pose>>(
        state.pose, measurement, noise_model));
    if (i > 0) {
      graph.add(std::make_shared<gtsam::WnoaMotionFactor<Pose>>(
          estimated_states_vec[i - 1], state, q_psd_diag_fixed));
    }
  }

  // Create initial trajectory estimate using constant velocity assumption.
  // This provides a reasonable starting point for the optimizer.
  const Velocity initial_twist =
      (Velocity() << 0.0, 0.0, 0.0, -1.0, 0.0, 0.0).finished();
  gtsam::Values values_init;
  Pose running_pose = values_gt.at<Pose>(estimated_states_vec.front().pose);
  values_init.insert(estimated_states_vec.front().pose, running_pose);
  values_init.insert(estimated_states_vec.front().velocity, initial_twist);
  double previous_time = estimated_states_vec.front().time;
  for (size_t i = 1; i < estimated_states_vec.size(); ++i) {
    const State& state = estimated_states_vec[i];
    const double delta_t = state.time - previous_time;
    running_pose = Pose::Expmap(initial_twist * delta_t).compose(running_pose);
    values_init.insert(state.pose, running_pose);
    values_init.insert(state.velocity, initial_twist);
    previous_time = state.time;
  }

  // Optimize the factor graph with estimated states and WNOA priors.
  const gtsam::Values result = optimizeGraph(graph, values_init);
  std::cout << "Base graph solved.\n";
  printStateSummary("Estimated states", result, values_gt,
                    estimated_states_vec);

  // Use GP regression to interpolate states between estimated states.
  // This recovers the continuous-time trajectory from the optimized discrete
  // states. Also extract covariances for uncertainty quantification.
  auto [values_interp, covariance_map] =
      gtsam::updateInterpValuesWithCovariance<Pose>(
          graph, result, estimated_states, interpolated_states, q_psd_diag);
  std::cout << "Interpolated states: " << interpolated_states.size() << "\n";
  std::cout << "Interpolated covariance entries: " << covariance_map.size()
            << "\n";
  printStateSummary("Interpolated states", values_interp, values_gt,
                    interpolated_states_vec);

  if (!interpolated_states_vec.empty()) {
    const State& sample_state = interpolated_states_vec.front();
    const auto cov_it = covariance_map.find(sample_state.pose);
    if (cov_it != covariance_map.end()) {
      std::cout << "Sample interpolated covariance trace: "
                << cov_it->second.trace() << "\n";
    }
  }

  // Add measurements on interpolated states without directly optimizing them.
  // WnoaInterpFactor wraps the measurement factor and handles interpolation
  // internally.
  gtsam::NonlinearFactorGraph wrapped_graph(graph);
  if (!interpolated_states_vec.empty()) {
    const State& mid_state =
        interpolated_states_vec[interpolated_states_vec.size() / 2];
    const Velocity noise = sampleGaussianDiagonal(measurement_cov, rng);
    const Pose measurement =
        Pose::Expmap(noise).compose(values_gt.at<Pose>(mid_state.pose));
    auto inner_factor = std::make_shared<gtsam::PriorFactor<Pose>>(
        mid_state.pose, measurement, noise_model);
    std::set<State> wrapped_interp_states{mid_state};
    wrapped_graph.add(std::make_shared<gtsam::WnoaInterpFactor<Pose>>(
        inner_factor, estimated_states, wrapped_interp_states, q_psd_diag));
  }

  const gtsam::Values wrapped_result =
      optimizeGraph(wrapped_graph, values_init);
  std::cout << "Wrapped graph solved.\n";
  printStateSummary("Wrapped result", wrapped_result, values_gt,
                    estimated_states_vec);

  // Automatically convert a factor graph where factors are defined on both
  // estimated and interpolated states. The conversion wraps interpolated state
  // factors and removes interpolated states from optimization.
  gtsam::NonlinearFactorGraph raw_interp_graph;
  std::vector<State> raw_measurement_states = estimated_states_vec;
  raw_measurement_states.insert(raw_measurement_states.end(),
                                interpolated_states_vec.begin(),
                                interpolated_states_vec.end());
  for (const State& state : raw_measurement_states) {
    const Velocity noise = sampleGaussianDiagonal(measurement_cov, rng);
    const Pose measurement =
        Pose::Expmap(noise).compose(values_gt.at<Pose>(state.pose));
    raw_interp_graph.add(std::make_shared<gtsam::PriorFactor<Pose>>(
        state.pose, measurement, noise_model));
  }

  // Automatically wrap factors and add WNOA priors between estimated states.
  const auto auto_graph = gtsam::interpolateFactorGraph<Pose, WnoaFactorGraph>(
      raw_interp_graph, estimated_states, interpolated_states, q_psd_diag);

  // Optimize the automatically converted graph with only estimated states
  // optimized.
  const gtsam::Values auto_result = optimizeGraph(auto_graph, values_init);
  std::cout << "Automatically converted graph solved.\n";
  printStateSummary("Auto-converted result", auto_result, values_gt,
                    estimated_states_vec);

  const auto [auto_values_interp, auto_covariance_map] =
      gtsam::updateInterpValuesWithCovariance<Pose>(
          auto_graph, auto_result, estimated_states, interpolated_states,
          q_psd_diag);
  std::cout << "Auto interpolation covariance entries: "
            << auto_covariance_map.size() << "\n";
  printStateSummary("Auto-interpolated states", auto_values_interp, values_gt,
                    interpolated_states_vec);

  std::cout << "Done.\n";
  return 0;
}
