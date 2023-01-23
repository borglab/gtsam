/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  Event.h
 *  @brief Space-time event
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date December 2014
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam_unstable/dllexport.h>

#include <cmath>
#include <iosfwd>
#include <string>

namespace gtsam {

/**
 * A space-time event models an event that happens at a certain 3D location, at
 * a certain time. One use for it is in sound-based or UWB-ranging tracking or
 * SLAM, where we have "time of arrival" measurements at a set of sensors. The
 * TOA functor below provides a measurement function for those applications.
 */
class Event {
  double time_;      ///< Time event was generated
  Point3 location_;  ///< Location at time event was generated

 public:
  enum { dimension = 4 };

  /// Default Constructor
  Event() : time_(0), location_(0, 0, 0) {}

  /// Constructor from time and location
  Event(double t, const Point3& p) : time_(t), location_(p) {}

  /// Constructor with doubles
  Event(double t, double x, double y, double z)
      : time_(t), location_(x, y, z) {}

  double time() const { return time_; }
  Point3 location() const { return location_; }

  // TODO(frank) we really have to think of a better way to do linear arguments
  double height(OptionalJacobian<1, 4> H = {}) const {
    static const Matrix14 JacobianZ = (Matrix14() << 0, 0, 0, 1).finished();
    if (H) *H = JacobianZ;
    return location_.z();
  }

  /** print with optional string */
  GTSAM_UNSTABLE_EXPORT void print(const std::string& s = "") const;

  /** equals with an tolerance */
  GTSAM_UNSTABLE_EXPORT bool equals(const Event& other,
                                    double tol = 1e-9) const;

  /// Updates a with tangent space delta
  inline Event retract(const Vector4& v) const {
    return Event(time_ + v[0], location_ + Point3(v.tail<3>()));
  }

  /// Returns inverse retraction
  inline Vector4 localCoordinates(const Event& q) const {
    return Vector4::Zero();  // TODO(frank) implement!
  }
};

// Define GTSAM traits
template <>
struct traits<Event> : internal::Manifold<Event> {};

/// Time of arrival to given sensor
class TimeOfArrival {
  const double speed_;  ///< signal speed

 public:
  typedef double result_type;

  /// Constructor with optional speed of signal, in m/sec
  explicit TimeOfArrival(double speed = 330) : speed_(speed) {}

  /// Calculate time of arrival
  double measure(const Event& event, const Point3& sensor) const {
    double distance = gtsam::distance3(event.location(), sensor);
    return event.time() + distance / speed_;
  }

  /// Calculate time of arrival, with derivatives
  double operator()(const Event& event, const Point3& sensor,  //
                    OptionalJacobian<1, 4> H1 = {},   //
                    OptionalJacobian<1, 3> H2 = {}) const {
    Matrix13 D1, D2;
    double distance = gtsam::distance3(event.location(), sensor, D1, D2);
    if (H1)
      // derivative of toa with respect to event
      *H1 << 1.0, D1 / speed_;
    if (H2)
      // derivative of toa with respect to sensor location
      *H2 << D2 / speed_;
    return event.time() + distance / speed_;
  }
};

}  // namespace gtsam
