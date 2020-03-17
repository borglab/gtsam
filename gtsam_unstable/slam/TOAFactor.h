/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TOAFactor.h
 *  @brief "Time of Arrival" factor
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date December 2014
 */

#pragma once

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam_unstable/geometry/Event.h>

namespace gtsam {

/// A "Time of Arrival" factor - so little code seems hardly worth it :-)
class TOAFactor : public ExpressionFactor<double> {
  typedef Expression<double> Double_;

 public:
  /**
   * Most genral constructor with two expressions
   * @param eventExpression expression yielding an event
   * @param sensorExpression expression yielding a sensor location
   * @param toaMeasurement time of arrival at sensor
   * @param model noise model
   * @param toa optional time of arrival functor
   */
  TOAFactor(const Expression<Event>& eventExpression,
            const Expression<Point3>& sensorExpression, double toaMeasurement,
            const SharedNoiseModel& model,
            const TimeOfArrival& toa = TimeOfArrival())
      : ExpressionFactor<double>(
            model, toaMeasurement,
            Double_(toa, eventExpression, sensorExpression)) {}

  /**
   * Constructor with fixed sensor
   * @param eventExpression expression yielding an event
   * @param sensor a known sensor location
   * @param toaMeasurement time of arrival at sensor
   * @param model noise model
   * @param toa optional time of arrival functor
   */
  TOAFactor(const Expression<Event>& eventExpression, const Point3& sensor,
            double toaMeasurement, const SharedNoiseModel& model,
            const TimeOfArrival& toa = TimeOfArrival())
      : TOAFactor(eventExpression, Expression<Point3>(sensor), toaMeasurement,
                  model, toa) {}
};

}  // namespace gtsam
