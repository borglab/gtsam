/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BearingFactor.H
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
template <class Pose, class Point, class Measured = typename Pose::Rotation>
class BearingFactor : public SerializableExpressionFactor<Measured> {
 private:
  GTSAM_CONCEPT_TESTABLE_TYPE(Measured)
  GTSAM_CONCEPT_POSE_TYPE(Pose)

  // Return measurement expression
  virtual Expression<Measured> expression() const {
    return Expression<Measured>(Expression<Pose>(this->keys_[0]), &Pose::bearing,
                                Expression<Point>(this->keys_[1]));
  }

 public:
  /// default constructor
  BearingFactor() {}

  /// primary constructor
  BearingFactor(Key poseKey, Key pointKey, const Measured& measured,
                const SharedNoiseModel& model)
      : SerializableExpressionFactor<Measured>(model, measured) {
    this->keys_.push_back(poseKey);
    this->keys_.push_back(pointKey);
    this->initialize(expression());
  }

  /// desructor
  virtual ~BearingFactor() {}

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
