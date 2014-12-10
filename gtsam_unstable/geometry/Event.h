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

  /// Speed of sound
  static const double Speed;

  /// Default Constructor
  Event() :
      time_(0) {
  }

  /// Constructor from time and location
  Event(double t, const Point3& p) :
      time_(t), location_(p) {
  }

  /// Constructor with doubles
  Event(double t, double x, double y, double z) :
      time_(t), location_(x, y, z) {
  }

  /** print with optional string */
  void print(const std::string& s = "") const {
    std::cout << s << "time = " << time_;
    location_.print("; location");
  }

  /** equals with an tolerance */
  bool equals(const Event& other, double tol = 1e-9) const {
    return std::abs(time_ - other.time_) < tol
        && location_.equals(other.location_, tol);
  }

  /// Manifold stuff:

  size_t dim() const {
    return 4;
  }
  static size_t Dim() {
    return 4;
  }

  /// Updates a with tangent space delta
  inline Event retract(const Vector4& v) const {
    return Event(time_ + v[0], location_.retract(v.tail(3)));
  }

  /// Returns inverse retraction
  inline Vector4 localCoordinates(const Event& q) const {
    return Vector4::Zero(); // TODO
  }

  /// Time of arrival to given microphone
  double toa(const Point3& microphone, //
      OptionalJacobian<1, 4> H1 = boost::none, //
      OptionalJacobian<1, 3> H2 = boost::none) const {
    Matrix13 D1, D2;
    double distance = location_.distance(microphone, D1, D2);
    if (H1)
      // derivative of toa with respect to event
      *H1 << 1.0, D1 / Speed;
    if (H2)
      // derivative of toa with respect to microphone location
      *H2 << D2 / Speed;
    return time_ + distance / Speed;
  }
};

const double Event::Speed = 330;

// Define GTSAM traits
namespace traits {

template<>
struct GTSAM_EXPORT dimension<Event> : public boost::integral_constant<int, 4> {
};

template<>
struct GTSAM_EXPORT is_manifold<Event> : public boost::true_type {
};

}

} //\ namespace gtsam
