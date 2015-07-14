/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file BearingRangeFactor.h
 * @date Apr 1, 2010
 * @author Kai Ni
 * @brief a single factor contains both the bearing and the range to prevent
 * handle to pair bearing and range factors
 */

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/geometry/BearingRange.h>
#include <boost/concept/assert.hpp>

namespace gtsam {

/**
 * Binary factor for a bearing/range measurement
 * @addtogroup SLAM
 */
template <typename A1, typename A2,
          typename B = typename Bearing<A1, A2>::result_type,
          typename R = typename Range<A1, A2>::result_type>
class BearingRangeFactor
    : public ExpressionFactor2<BearingRange<A1, A2>, A1, A2> {
 private:
  typedef BearingRange<A1, A2> T;
  typedef ExpressionFactor2<T, A1, A2> Base;
  typedef BearingRangeFactor<A1, A2> This;

 public:
  typedef boost::shared_ptr<This> shared_ptr;

  /// default constructor
  BearingRangeFactor() {}

  /// primary constructor
  BearingRangeFactor(Key key1, Key key2, const B& measuredBearing,
                     const R& measuredRange, const SharedNoiseModel& model)
      : Base(key1, key2, model, T(measuredBearing, measuredRange)) {
    this->initialize(expression(key1, key2));
  }

  virtual ~BearingRangeFactor() {}

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  virtual Expression<T> expression(Key key1, Key key2) const {
    return Expression<T>(T::Measure, Expression<A1>(key1),
                         Expression<A2>(key2));
  }

  /// print
  virtual void print(const std::string& s = "",
                     const KeyFormatter& kf = DefaultKeyFormatter) const {
    std::cout << s << "BearingRangeFactor" << std::endl;
    Base::print(s, kf);
  }

};  // BearingRangeFactor

/// traits
template <typename A1, typename A2, typename B, typename R>
struct traits<BearingRangeFactor<A1, A2, B, R> >
    : public Testable<BearingRangeFactor<A1, A2, B, R> > {};

}  // namespace gtsam
