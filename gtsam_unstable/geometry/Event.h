/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  Event
 *  @brief Space-time event
 *  @author Frank Dellaert
 *  @author Jay Chakravarty
 *  @date December 2014
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <cmath>

namespace gtsam {

/// A space-time event
class Event {

  double time_; ///< Time event was generated
  Point3 location_; ///< Location at time event was generated

public:
  enum { dimension = 4 };

  /// Default Constructor
  Event() :
      time_(0), location_(0,0,0) {
  }

  /// Constructor from time and location
  Event(double t, const Point3& p) :
      time_(t), location_(p) {
  }

  /// Constructor with doubles
  Event(double t, double x, double y, double z) :
      time_(t), location_(x, y, z) {
  }

  double time() const { return time_;}
  Point3 location() const { return location_;}

  // TODO we really have to think of a better way to do linear arguments
  double height(OptionalJacobian<1,4> H = boost::none) const {
    static const Matrix14 JacobianZ = (Matrix14() << 0,0,0,1).finished();
    if (H) *H = JacobianZ;
    return location_.z();
  }

  /** print with optional string */
  void print(const std::string& s = "") const {
    std::cout << s << "time = " << time_ << "location = " << location_.transpose();
  }

  /** equals with an tolerance */
  bool equals(const Event& other, double tol = 1e-9) const {
    return std::abs(time_ - other.time_) < tol
        && traits<Point3>::Equals(location_, other.location_, tol);
  }

  /// Updates a with tangent space delta
  inline Event retract(const Vector4& v) const {
    return Event(time_ + v[0], location_ + Point3(v.tail<3>()));
  }

  /// Returns inverse retraction
  inline Vector4 localCoordinates(const Event& q) const {
    return Vector4::Zero(); // TODO
  }

  /// Time of arrival to given microphone
  double toa(const Point3& microphone, //
      OptionalJacobian<1, 4> H1 = boost::none, //
      OptionalJacobian<1, 3> H2 = boost::none) const {
    static const double Speed = 330;
    Matrix13 D1, D2;
    double distance = gtsam::distance3(location_, microphone, D1, D2);
    if (H1)
      // derivative of toa with respect to event
      *H1 << 1.0, D1 / Speed;
    if (H2)
      // derivative of toa with respect to microphone location
      *H2 << D2 / Speed;
    return time_ + distance / Speed;
  }
};

// Define GTSAM traits
template<>
struct GTSAM_EXPORT traits<Event> : internal::Manifold<Event> {};

} //\ namespace gtsam
