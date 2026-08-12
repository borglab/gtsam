/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file WnoaStateData.h
 * @brief Introduces a lightweight struct for identifying states in
 * continuous-time estimation and interpolation.
 * @author Connor Holmes
 * @author Zi Cong Guo
 * @author Sven Lilge
 */

#pragma once

#include <gtsam/inference/Key.h>

#include <functional>
#include <limits>

namespace gtsam {

/**
 * @brief Lightweight container for states used for continuous-time estimation
 * and interpolation.
 *
 * `StateData` stores the keys identifying the pose and velocity variables in
 * a factor graph together with the associated timestamp. It is intentionally
 * minimal and used primarily for ordering, lookup and grouping of states when
 * constructing interpolation queries.
 */
struct GTSAM_EXPORT StateData {
  Key pose;      ///< Key of the pose variable
  Key velocity;  ///< Key of the velocity variable
  double time;   ///< Timestamp (seconds) associated with this state

  /// Default constructor.
  StateData() = default;

  /**
   * @brief Construct a StateData from explicit keys and timestamp.
   * @param pose_in Pose key
   * @param velocity_in Velocity key
   * @param time_in Timestamp in seconds
   */
  StateData(Key pose_in, Key velocity_in, double time_in)
      : pose(pose_in), velocity(velocity_in), time(time_in) {};

  /**
   * @brief Strict-weak ordering: (time, pose, velocity) ascending.
   *
   * Primarily orders states chronologically by time. When timestamps are
   * equal, pose and then velocity keys are used as tie-breakers to provide a
   * strict-weak ordering suitable for ordered containers such as std::set.
   */
  bool operator<(const StateData& other) const {
    if (this->time < other.time) return true;
    if (this->time > other.time) return false;
    if (this->pose < other.pose) return true;
    if (this->pose > other.pose) return false;
    return this->velocity < other.velocity;
  }

  /**
   * @brief Strict-weak ordering: (time, pose, velocity) descending.
   *
   * Mirror of `operator<` with all comparisons reversed.
   */
  bool operator>(const StateData& other) const {
    if (this->time > other.time) return true;
    if (this->time < other.time) return false;
    if (this->pose > other.pose) return true;
    if (this->pose < other.pose) return false;
    return this->velocity > other.velocity;
  }

  /// Returns true if this state's timestamp is before \p time.
  bool operator<(double time) const { return this->time < time; }

  /// Returns true if this state's timestamp is after \p time.
  bool operator>(double time) const { return this->time > time; }

  /**
   * @brief Equality compares the identifying keys (pose and velocity).
   *
   * Two `StateData` objects are considered equal if they refer to the same
   * pose and velocity keys. The timestamp is not used for equality to keep
   * `StateData` usable as a simple identifier in unordered containers.
   */
  bool operator==(const StateData& other) const {
    return (this->pose == other.pose) && (this->velocity == other.velocity);
  }
};

}  // namespace gtsam

/**
 * @brief Hash `StateData` using the pose and velocity keys.
 *
 * Combines the hash of the pose and velocity keys to produce a stable
 * hash suitable for `unordered_map`/`unordered_set`. The timestamp is not
 * included because `StateData` equality ignores time as well.
 */
namespace std {
template <>
struct hash<gtsam::StateData> {
  std::size_t operator()(const gtsam::StateData& p) const noexcept {
    return std::hash<uint64_t>()(p.pose) ^
           (std::hash<uint64_t>()(p.velocity) << 1);
  }
};
}  // namespace std