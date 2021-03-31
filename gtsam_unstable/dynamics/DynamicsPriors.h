/**
 * @file DynamicsPriors.h
 *
 * @brief Priors to be used with dynamic systems (Specifically PoseRTV)
 *
 * @date Nov 22, 2011
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dynamics/PoseRTV.h>
#include <gtsam_unstable/slam/PartialPriorFactor.h>

namespace gtsam {

// Indices of relevant variables in the PoseRTV tangent vector:
// [ rx ry rz tx ty tz vx vy vz ]
static const size_t kRollIndex = 0;
static const size_t kPitchIndex = 1;
static const size_t kHeightIndex = 5;
static const size_t kVelocityZIndex = 8;
static const std::vector<size_t> kVelocityIndices = { 6, 7, 8 };

/**
 * Forces the value of the height (z) in a PoseRTV to a specific value.
 * Dim: 1
 */
struct DHeightPrior : public gtsam::PartialPriorFactor<PoseRTV> {
  typedef gtsam::PartialPriorFactor<PoseRTV> Base;

  DHeightPrior(Key key, double height, const gtsam::SharedNoiseModel& model)
  : Base(key, kHeightIndex, height, model)  {}
};

/**
 * Forces the roll to a particular value - useful for flying robots
 * Implied value is zero
 * Dim: 1
 */
struct DRollPrior : public gtsam::PartialPriorFactor<PoseRTV> {
  typedef gtsam::PartialPriorFactor<PoseRTV> Base;

  /** allows for explicit roll parameterization - uses canonical coordinate */
  DRollPrior(Key key, double wx, const gtsam::SharedNoiseModel& model)
      : Base(key, kRollIndex, wx, model)  {  }

  /** Forces roll to zero */
  DRollPrior(Key key, const gtsam::SharedNoiseModel& model)
      : Base(key, kRollIndex, 0.0, model)  {  }
};

/**
 * Constrains the full velocity of a state to a particular value
 * Useful for enforcing a stationary state
 * Dim: 3
 */
struct VelocityPrior : public gtsam::PartialPriorFactor<PoseRTV> {
  typedef gtsam::PartialPriorFactor<PoseRTV> Base;

  VelocityPrior(Key key, const gtsam::Vector& vel, const gtsam::SharedNoiseModel& model)
      : Base(key, kVelocityIndices, vel, model) {}
};

/**
 * Ground constraint: forces the robot to be upright (no roll, pitch), a fixed height, and no
 * velocity in z direction
 * Dim: 4
 */
struct DGroundConstraint : public gtsam::PartialPriorFactor<PoseRTV> {
  typedef gtsam::PartialPriorFactor<PoseRTV> Base;

  /**
   * Primary constructor allows for variable height of the "floor"
   */
  DGroundConstraint(Key key, double height, const gtsam::SharedNoiseModel& model)
      : Base(key, { kHeightIndex, kVelocityZIndex, kRollIndex, kPitchIndex },
             Vector::Unit(4, 0)*height, model) {}

  /**
   * Fully specify vector - use only for debugging
   */
  DGroundConstraint(Key key, const Vector& constraint, const gtsam::SharedNoiseModel& model)
  : Base(key, { kHeightIndex, kVelocityZIndex, kRollIndex, kPitchIndex }, constraint, model) {
    assert(constraint.size() == 4);
  }
};

} // \namespace gtsam
