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

#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam_unstable/geometry/Event.h>

namespace gtsam {

/// A "Time of Arrival" factor - so little code seems hardly worth it :-)
class TOAFactor: public ExpressionFactor<double> {

  typedef Expression<double> Double_;

public:

  /**
   * Constructor
   * @param some expression yielding an event
   * @param microphone_ expression yielding a microphone location
   * @param toaMeasurement time of arrival at microphone
   * @param model noise model
   */
  TOAFactor(const Expression<Event>& eventExpression,
      const Expression<Point3>& microphone_, double toaMeasurement,
      const SharedNoiseModel& model) :
      ExpressionFactor<double>(model, toaMeasurement,
          Double_(&Event::toa, eventExpression, microphone_)) {
  }

};

} //\ namespace gtsam

