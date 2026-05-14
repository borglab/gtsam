/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  WnoaFactorGraph.cpp
 *  @brief Factor graph that handles computation of interpolated states for WNOA
 *  @author Sven Lilge
 */

#include <gtsam/config.h>  // for GTSAM_USE_TBB
#include <gtsam/nonlinear/WnoaFactorGraph.h>

#include <Eigen/Core>  // for Eigen::setNbThreads

#ifdef GTSAM_USE_TBB
#include <tbb/concurrent_unordered_map.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/partitioner.h>
#endif

#include <algorithm>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_set>

namespace gtsam {

/* ************************************************************************* */
namespace {

#ifdef GTSAM_USE_TBB
template <typename PoseType>
class _LinearizeOneFactor {
  const WnoaFactorGraph<PoseType>& nonlinearGraph_;
  const Values& linearizationPoint_;
  GaussianFactorGraph& result_;
  typename WnoaInterpFactor<PoseType>::PassedInterpData& passedInterpData_;

 public:
  // Create functor with constant parameters
  _LinearizeOneFactor(
      const WnoaFactorGraph<PoseType>& graph, const Values& linearizationPoint,
      GaussianFactorGraph& result,
      typename WnoaInterpFactor<PoseType>::PassedInterpData& passedInterpData)
      : nonlinearGraph_(graph),
        linearizationPoint_(linearizationPoint),
        result_(result),
        passedInterpData_(passedInterpData) {}
  // Operator that linearizes a given range of the factors
  void operator()(const tbb::blocked_range<size_t>& blocked_range) const {
    for (size_t i = blocked_range.begin(); i != blocked_range.end(); ++i) {
      if (nonlinearGraph_[i] && nonlinearGraph_[i]->sendable()) {
        // Attempt dynamic cast to WnoaInterpFactor
        auto wnoa_factor =
            std::dynamic_pointer_cast<WnoaInterpFactor<PoseType>>(
                nonlinearGraph_[i]);
        if (wnoa_factor) {
          result_[i] =
              wnoa_factor->linearize(linearizationPoint_, &passedInterpData_);
        } else {
          result_[i] = nonlinearGraph_[i]->linearize(linearizationPoint_);
        }
      } else {
        result_[i] = GaussianFactor::shared_ptr();
      }
    }
  }
};
#endif

}  // namespace

template <typename PoseType>
WnoaFactorGraph<PoseType>::WnoaFactorGraph(
    std::unordered_map<StateData, std::pair<StateData, StateData>> interp_map,
    const Eigen::Vector<double, dim> q_psd_diag, bool fixed_noise_model)
    : interpolator_(q_psd_diag),
      interp_to_borders_map_(std::move(interp_map)),
      fixed_noise_model_(fixed_noise_model) {
  // Collect unique keys for boundary states to avoid repeated Values::at
  // lookups.
  border_pose_keys_.reserve(interp_to_borders_map_.size() * 2);
  border_vel_keys_.reserve(interp_to_borders_map_.size() * 2);
  interp_to_borders_vec_.reserve(interp_to_borders_map_.size());
  for (const auto& kv : interp_to_borders_map_) {
    const auto& border_states = kv.second;
    const auto& left = border_states.first;
    const auto& right = border_states.second;
    border_pose_keys_.insert(left.pose);
    border_pose_keys_.insert(right.pose);
    border_vel_keys_.insert(left.velocity);
    border_vel_keys_.insert(right.velocity);
    // Store local interpolation matrices
    double tau = kv.first.time;
    double t_k = left.time;
    double t_kp1 = right.time;
    interp_to_LambdaPsi_vec_.emplace_back(
        kv.first, std::make_shared<LambdaPsiMats>(
                      interpolator_.getLambdaPsi(t_k, t_kp1, tau)));
    // Convert map to vector for optimal parallel access patterns
    interp_to_borders_vec_.emplace_back(kv);
  }

  // Build compact batch vector: for each unique border pair, list indices of
  // interp states
  struct LocalBorderHash {
    size_t operator()(const std::pair<StateData, StateData>& p) const noexcept {
      const size_t h1 = std::hash<StateData>{}(p.first);
      const size_t h2 = std::hash<StateData>{}(p.second);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };
  std::unordered_map<std::pair<StateData, StateData>, std::vector<size_t>,
                     LocalBorderHash>
      tmp;
  tmp.reserve(interp_to_borders_vec_.size());
  for (size_t idx = 0; idx < interp_to_borders_vec_.size(); ++idx) {
    const auto& borders =
        interp_to_borders_vec_[idx].second;  // pair<StateData, StateData>
    tmp[borders].push_back(idx);
  }
  border_batches_.clear();
  border_batches_.reserve(tmp.size());
  for (auto& kv : tmp) border_batches_.emplace_back(std::move(kv));
}

/* ************************************************************************* */
template <typename PoseType>
std::shared_ptr<GaussianFactorGraph> WnoaFactorGraph<PoseType>::linearize(
    const Values& linearizationPoint) const {
  // Compute values, Jacobians and conditional covariances for all interpolated
  // states

  auto passedInterpData =
      std::make_shared<typename WnoaInterpFactor<PoseType>::PassedInterpData>();

  if (fixed_noise_model_) {
    passedInterpData->values = getInterpolatedValues(
        linearizationPoint, &passedInterpData->jacobians, nullptr);
  } else {
    passedInterpData->values =
        getInterpolatedValues(linearizationPoint, &passedInterpData->jacobians,
                              &passedInterpData->condCovs);
  }

  // create an empty linear FG
  GaussianFactorGraph::shared_ptr linearFG =
      std::make_shared<GaussianFactorGraph>();

#ifdef GTSAM_USE_TBB

  linearFG->resize(size());
  TbbOpenMPMixedScope
      threadLimiter;  // Limits OpenMP threads since we're mixing TBB and OpenMP

  // First linearize all sendable factors
  tbb::parallel_for(tbb::blocked_range<size_t>(0, size()),
                    _LinearizeOneFactor<PoseType>(*this, linearizationPoint,
                                                  *linearFG, *passedInterpData),
                    tbb::auto_partitioner());

  // Linearize all non-sendable factors
  for (size_t i = 0; i < size(); i++) {
    auto& factor = (*this)[i];
    if (factor && !(factor->sendable())) {
      // Attempt dynamic cast to WnoaInterpFactor
      auto wnoa_factor =
          std::dynamic_pointer_cast<WnoaInterpFactor<PoseType>>(factor);
      if (wnoa_factor) {
        (*linearFG)[i] =
            wnoa_factor->linearize(linearizationPoint, passedInterpData.get());
      } else {
        (*linearFG)[i] = factor->linearize(linearizationPoint);
      }
    }
  }

#else

  linearFG->reserve(size());

  // linearize all factors
  for (const sharedFactor& factor : factors_) {
    // Attempt dynamic cast to WnoaInterpFactor
    if (factor) {
      auto wnoa_factor =
          std::dynamic_pointer_cast<WnoaInterpFactor<PoseType>>(factor);
      if (wnoa_factor) {
        linearFG->push_back(
            wnoa_factor->linearize(linearizationPoint, passedInterpData.get()));
      } else {
        linearFG->push_back(factor->linearize(linearizationPoint));
      }
    } else
      linearFG->push_back(GaussianFactor::shared_ptr());
  }

#endif

  return linearFG;
}

template <typename PoseType>
double WnoaFactorGraph<PoseType>::error(const Values& values) const {
  double total_error = 0.;

  // Compute values, Jacobians and conditional covariances for all interpolated
  // states

  auto passedInterpData =
      std::make_shared<typename WnoaInterpFactor<PoseType>::PassedInterpData>();
  if (fixed_noise_model_) {
    passedInterpData->values = getInterpolatedValues(values, nullptr, nullptr);
  } else {
    passedInterpData->values =
        getInterpolatedValues(values, nullptr, &passedInterpData->condCovs);
  }

  // iterate over all the factors_ to accumulate the log probabilities
  for (const sharedFactor& factor : factors_) {
    if (factor) {
      auto wnoa_factor =
          std::dynamic_pointer_cast<WnoaInterpFactor<PoseType>>(factor);
      if (wnoa_factor) {
        total_error += wnoa_factor->error(values, passedInterpData.get());
      } else {
        total_error += factor->error(values);
      }
    }
  }

  return total_error;
}
/* @brief Interpolate all interpolated states based on estimated states.
 * Put their values in a Values structure and compute their Jacobians.*/
template <typename PoseType>
Values WnoaFactorGraph<PoseType>::getInterpolatedValues(
    const Values& values,
    std::unordered_map<Key, std::array<Matrix, 4>>* InterpJacobians,
    std::unordered_map<StateData, Matrix2N>* InterpCondCovs) const {
#ifdef GTSAM_USE_TBB
  // Parallelized version using TBB
  // We parallelize over border batches
  // Each batch corresponds to a pair of bordering states the set of
  // interpolated states that lie between them This allows us to compute
  // interpolated values and Jacobians for all states in a batch together. This
  // is more efficient than parallelizing over individual interpolated states,
  // due to shared border state computations. We use TBB's parallel_for with a
  // blocked range to process batches in parallel. We use thread-local storage to
  // accumulate results before merging them into the final output containers

  TbbOpenMPMixedScope threadLimiter;
  // Cache boundary values
  std::unordered_map<Key, PoseType> border_pose_cache;
  border_pose_cache.reserve(border_pose_keys_.size());
  std::unordered_map<Key, VelocityType> border_vel_cache;
  border_vel_cache.reserve(border_vel_keys_.size());
  for (Key k : border_pose_keys_)
    border_pose_cache.emplace(k, values.at<PoseType>(k));
  for (Key k : border_vel_keys_)
    border_vel_cache.emplace(k, values.at<VelocityType>(k));

  // Use precomputed border batches stored in the graph (built in constructor)
  auto& batches = this->border_batches_;

  // Thread-local storage for accumulating interpolated values, Jacobians and
  // conditional covariances before merging
  struct TLS {
    Values local_values;
    std::vector<std::pair<Key, std::array<Matrix, 4>>> jacs;
    std::vector<std::pair<StateData, Matrix2N>> condcovs;
    std::vector<Matrix> H;
  };
  tbb::enumerable_thread_specific<TLS> tls;

  // Determine grain size for parallel_for
  // This controls the number of batches processed by each thread in one go.
  // We want to balance load and overhead.
  // Might give a minimal performance boost compared to the default
  // auto_partitioner
  unsigned nt = std::thread::hardware_concurrency();
  unsigned phys = nt ? std::max<unsigned>(1u, nt / 2) : 1u;
  size_t grain = std::max<size_t>(1, batches.size() / (8 * phys));
  {
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, batches.size(), grain),
        [&](const tbb::blocked_range<size_t>& r) {
          // Allocate space for variables based on number of batches and batch
          // size to avoid dynamic resizing in the loop
          auto& local = tls.local();
          if (local.H.empty()) local.H.resize(8);
          size_t expectedInterp = 0;
          for (size_t bi = r.begin(); bi != r.end(); ++bi)
            expectedInterp +=
                batches[bi].second.size();  // number of interpolated states in
                                            // this batch
          if (InterpJacobians)
            local.jacs.reserve(
                local.jacs.size() +
                expectedInterp *
                    2);  // 2 factors per interpolated state (pose and velocity)
          if (InterpCondCovs)
            local.condcovs.reserve(local.condcovs.size() +
                                   expectedInterp);  // 1 conditional covariance
                                                     // per interpolated state

          // Process each batch in the assigned range
          for (size_t bi = r.begin(); bi != r.end(); ++bi) {
            // Get border states for this batch from cache
            const auto& bp = batches[bi].first;
            const StateData& left = bp.first;
            const StateData& right = bp.second;

            // Get the corresponding state values from the cache
            const auto state_left = TimestampedPoseVelocity<PoseType>(
              border_pose_cache.at(left.pose),
              border_vel_cache.at(left.velocity), left.time);
            const auto state_right = TimestampedPoseVelocity<PoseType>(
              border_pose_cache.at(right.pose),
              border_vel_cache.at(right.velocity), right.time);

            // Precompute local state vars and local-global Jacobians for this
            // border pair
            std::shared_ptr<StateJacobians> stateJacobiansPreComp =
                std::make_shared<StateJacobians>();
            std::shared_ptr<LocalStateVecs> localStateVecsPreComp =
                std::make_shared<LocalStateVecs>();

            if (InterpJacobians) {
              // If we're computing Jacobians, we need to compute the local
              // state vectors (xi, xi_dot at the border) and local-global
              // Jacobians (at the border) for this border pair We can be reuse
              // those for all interpolated states in this batch (only xi_tau,
              // xi_dot_tau and the interpolation Jacobians change between
              // interpolated states in the same batch)
              *localStateVecsPreComp = interpolator_.computeLocalStateVecs(
                  state_left, state_right, stateJacobiansPreComp.get());
            } else {
              // If we're not computing Jacobians, we only need to compute the
              // local state vectors (xi, xi_dot at the border) for this border
              // pair once Again, we can reuse them for all interpolated states
              // in this batch
              *localStateVecsPreComp = interpolator_.computeLocalStateVecs(
                  state_left, state_right, nullptr);
            }

            // Run through all interpolated states in this batch
            for (size_t interpIdx : batches[bi].second) {
              // Get the actual interpolated state data (time, keys) for this
              // interpolated state index
              const StateData& interp_state =
                  interp_to_borders_vec_[interpIdx].first;
              PoseVelocity<PoseType> result;
              if (InterpJacobians) {
                // If we're computing Jacobians, we compute the interpolated
                // state value and Jacobians for this interpolated state We can
                // reuse the precomputed local state vectors and local-global
                // Jacobians for the border states Only the interpolation
                // Jacobians (xi_tau, xi_dot_tau) change between interpolated
                // states in the same batch We can also rely on the precomputed
                // Lambda and Psi interpolation matrices for this interpolated
                // state that we stored in the graph during construction (only
                // depends on tau and not the state values)
                result = interpolator_.interpolatePoseAndVelocity(
                    state_left, state_right, interp_state.time, &local.H,
                    nullptr, nullptr,
                    interp_to_LambdaPsi_vec_[interpIdx].second,
                    localStateVecsPreComp, stateJacobiansPreComp);
                std::array<Matrix, 4> Jpose = {local.H[0], local.H[1],
                                               local.H[2], local.H[3]};
                std::array<Matrix, 4> Jvel = {local.H[4], local.H[5],
                                              local.H[6], local.H[7]};
                local.jacs.emplace_back(interp_state.pose, std::move(Jpose));
                local.jacs.emplace_back(interp_state.velocity, std::move(Jvel));
              } else {
                // If we're not computing Jacobians, we only compute the
                // interpolated state value for this interpolated state We can
                // reuse the precomputed local state vectors for the border
                // states and the precomputed Lambda and Psi interpolation
                // matrices for this interpolated state that we stored in the
                // graph during construction
                result = interpolator_.interpolatePoseAndVelocity(
                    state_left, state_right, interp_state.time, nullptr,
                    nullptr, nullptr,
                    interp_to_LambdaPsi_vec_[interpIdx].second,
                    localStateVecsPreComp, nullptr);
              }
              // Insert the computed interpolated state value into the
              // thread-local Values structure
              local.local_values.insert(interp_state.pose,
                                        std::move(result.pose));
              local.local_values.insert(interp_state.velocity,
                                        std::move(result.vel));
              if (InterpCondCovs) {
                // If we're computing conditional covariances, we also compute
                // the conditional covariance for this interpolated state
                auto state_tau = TimestampedPoseVelocity<PoseType>(
                    result, interp_state.time);
                Matrix2N Sigma_tau = interpolator_.computeConditionalCov(
                    state_left, state_right, state_tau);
                local.condcovs.emplace_back(interp_state, std::move(Sigma_tau));
              }
            }
          }
        });
  }

  // Now we need to merge the thread-local results into the final output
  // containers We reserved space in the thread-local containers based on the
  // expected number of interpolated states to minimize dynamic resizing during
  // merging
  Values values_interp;
  if (InterpJacobians)
    InterpJacobians->reserve(interp_to_borders_vec_.size() * 2);
  if (InterpCondCovs) InterpCondCovs->reserve(interp_to_borders_vec_.size());
  for (auto& local : tls) {
    // Bulk-insert per-thread Values
    values_interp.insert(local.local_values);
    if (InterpJacobians)
      for (auto& kv : local.jacs)
        InterpJacobians->insert_or_assign(kv.first, std::move(kv.second));
    if (InterpCondCovs)
      for (auto& sc : local.condcovs)
        (*InterpCondCovs)[sc.first] = std::move(sc.second);
  }
  return values_interp;
#else
  // Sequential version
  // Logic is the same as in the parallel version, but we can directly insert
  // results into the final output containers without needing thread-local
  // storage or merging See comments above for more details on the logic
  std::unordered_map<Key, PoseType> border_pose_cache;
  border_pose_cache.reserve(border_pose_keys_.size());
  std::unordered_map<Key, VelocityType> border_vel_cache;
  border_vel_cache.reserve(border_vel_keys_.size());
  for (Key k : border_pose_keys_)
    border_pose_cache.emplace(k, values.at<PoseType>(k));
  for (Key k : border_vel_keys_)
    border_vel_cache.emplace(k, values.at<VelocityType>(k));
  std::vector<Matrix> H(8);
  Values values_interp;
  if (InterpJacobians)
    InterpJacobians->reserve(interp_to_borders_vec_.size() * 2);
  if (InterpCondCovs) InterpCondCovs->reserve(interp_to_borders_vec_.size());
  for (const auto& kv : border_batches_) {
    const StateData& left = kv.first.first;
    const StateData& right = kv.first.second;
    const auto state_left = TimestampedPoseVelocity<PoseType>(
      border_pose_cache.at(left.pose),
      border_vel_cache.at(left.velocity), left.time);
    const auto state_right = TimestampedPoseVelocity<PoseType>(
      border_pose_cache.at(right.pose),
      border_vel_cache.at(right.velocity), right.time);

    // Precompute local state vars and local-global Jacobians for this border
    // pair
    std::shared_ptr<StateJacobians> stateJacobiansPreComp =
        std::make_shared<StateJacobians>();
    std::shared_ptr<LocalStateVecs> localStateVecsPreComp =
        std::make_shared<LocalStateVecs>();

    if (InterpJacobians) {
      *localStateVecsPreComp = interpolator_.computeLocalStateVecs(
          state_left, state_right, stateJacobiansPreComp.get());
    } else {
      *localStateVecsPreComp =
          interpolator_.computeLocalStateVecs(state_left, state_right, nullptr);
    }

    for (size_t interpIdx : kv.second) {
      const StateData& interp_state = interp_to_borders_vec_[interpIdx].first;
      PoseVelocity<PoseType> result;
      if (InterpJacobians) {
        result = interpolator_.interpolatePoseAndVelocity(
            state_left, state_right, interp_state.time, &H, nullptr, nullptr,
            interp_to_LambdaPsi_vec_[interpIdx].second, localStateVecsPreComp,
            stateJacobiansPreComp);
      } else {
        result = interpolator_.interpolatePoseAndVelocity(
            state_left, state_right, interp_state.time, nullptr, nullptr,
            nullptr, interp_to_LambdaPsi_vec_[interpIdx].second,
            localStateVecsPreComp, nullptr);
      }
      values_interp.insert(interp_state.pose, result.pose);
      values_interp.insert(interp_state.velocity, result.vel);
      if (InterpJacobians) {
        (*InterpJacobians)[interp_state.pose] =
            std::array<Matrix, 4>{H[0], H[1], H[2], H[3]};
        (*InterpJacobians)[interp_state.velocity] =
            std::array<Matrix, 4>{H[4], H[5], H[6], H[7]};
      }
      if (InterpCondCovs) {
        auto state_tau =
            TimestampedPoseVelocity<PoseType>(result, interp_state.time);
        Matrix2N Sigma_tau = interpolator_.computeConditionalCov(
            state_left, state_right, state_tau);
        (*InterpCondCovs)[interp_state] = std::move(Sigma_tau);
      }
    }
  }
  return values_interp;
#endif
}

// Explicit template instantiation
template class GTSAM_EXPORT WnoaFactorGraph<Point1>;
template class GTSAM_EXPORT WnoaFactorGraph<Point2>;
template class GTSAM_EXPORT WnoaFactorGraph<Point3>;
template class GTSAM_EXPORT WnoaFactorGraph<Pose2>;
template class GTSAM_EXPORT WnoaFactorGraph<Pose3>;
}  // namespace gtsam
