/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  Event.cpp
 *  @brief Space-time event
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date December 2014
 */

#include <gtsam_unstable/geometry/Event.h>
#include <iostream>

namespace gtsam {

/* ************************************************************************* */
void Event::print(const std::string& s) const {
  std::cout << s << "{'time':" << time_
            << ", 'location': " << location_.transpose() << "}";
}

/* ************************************************************************* */
bool Event::equals(const Event& other, double tol) const {
  return std::abs(time_ - other.time_) < tol &&
         traits<Point3>::Equals(location_, other.location_, tol);
}

/* ************************************************************************* */

}  // namespace gtsam
