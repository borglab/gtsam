/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BearingFactor.h
 *  @brief Serializable factor induced by a bearing measurement
 *  @date July 2015
 *  @author Frank Dellaert
 **/

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>

namespace gtsam {

// forward declaration of Bearing functor, assumed partially specified
template <typename A1, typename A2>
struct Bearing;

/**
 * Binary factor for a bearing measurement
 * Works for any two types A1,A2 for which the functor Bearing<A1,A2>() is
 * defined
 * @addtogroup SAM
 */
template <typename A1, typename A2,
          typename T = typename Bearing<A1, A2>::result_type>
struct BearingFactor : public ExpressionFactor2<T, A1, A2> {
  typedef ExpressionFactor2<T, A1, A2> Base;

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
    Expression<A1> a1_(key1);
    Expression<A2> a2_(key2);
    return Expression<T>(Bearing<A1, A2>(), a1_, a2_);
  }

  /// print
  void print(const std::string& s = "",
             const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "BearingFactor" << std::endl;
    Base::print(s, kf);
  }
};  // BearingFactor

/// traits
template <typename A1, typename A2, typename T>
struct traits<BearingFactor<A1, A2, T> >
    : public Testable<BearingFactor<A1, A2, T> > {};

}  // namespace gtsam
