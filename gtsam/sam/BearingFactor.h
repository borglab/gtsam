/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BearingFactor.h
 *  @brief Serializable factor induced by a bearing measurement on a point from a pose
 *  @date July 2015
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/SerializableExpressionFactor.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

/**
 * Binary factor for a bearing measurement
 * @addtogroup SAM
 */
template <typename Pose, typename Point, typename Measured>
struct BearingFactor : public SerializableExpressionFactor<Measured> {
  GTSAM_CONCEPT_POSE_TYPE(Pose)

  /// default constructor
  BearingFactor() {}

  /// primary constructor
  BearingFactor(Key poseKey, Key pointKey, const Measured& measured, const SharedNoiseModel& model)
      : SerializableExpressionFactor<Measured>(model, measured) {
    this->keys_.push_back(poseKey);
    this->keys_.push_back(pointKey);
    this->initialize(expression());
  }

  // Return measurement expression
  virtual Expression<Measured> expression() const {
    return Expression<Measured>(Expression<Pose>(this->keys_[0]), &Pose::bearing,
                                Expression<Point>(this->keys_[1]));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "BearingFactor" << std::endl;
    SerializableExpressionFactor<Measured>::print(s, kf);
  }
};  // BearingFactor

/// traits
template <class Pose, class Point, class Measured>
struct traits<BearingFactor<Pose, Point, Measured> >
    : public Testable<BearingFactor<Pose, Point, Measured> > {};

}  // namespace gtsam
