/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BearingFactor.h
 *  @brief Serializable factor induced by a bearing measurement of a point from
 *a given pose
 *  @date July 2015
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/SerializableExpressionFactor.h>
#include <gtsam/geometry/concepts.h>

namespace gtsam {

/**
 * Binary factor for a bearing measurement
 * @addtogroup SAM
 */
template <typename POSE, typename POINT, typename T>
struct BearingFactor : public SerializableExpressionFactor2<T, POSE, POINT> {
  typedef SerializableExpressionFactor2<T, POSE, POINT> Base;

  /// default constructor
  BearingFactor() {}

  /// primary constructor
  BearingFactor(Key key1, Key key2, const T& measured,
                const SharedNoiseModel& model)
      : Base(key1, key2, model, measured) {
    this->initialize(expression(key1, key2));
  }

  // Return measurement expression
  virtual Expression<T> expression(Key key1, Key key2) const {
    return Expression<T>(Expression<POSE>(key1), &POSE::bearing,
                         Expression<POINT>(key2));
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "BearingFactor" << std::endl;
    Base::print(s, kf);
  }
};  // BearingFactor

/// traits
template <class POSE, class POINT, class T>
struct traits<BearingFactor<POSE, POINT, T> >
    : public Testable<BearingFactor<POSE, POINT, T> > {};

}  // namespace gtsam
