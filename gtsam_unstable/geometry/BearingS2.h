/**
 * @file BearingS2.h
 *
 * @brief Manifold measurement between two points on a unit sphere
 *
 * @date Jan 26, 2012
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam_unstable/dllexport.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

class GTSAM_UNSTABLE_EXPORT BearingS2 {
protected:
  Rot2 azimuth_, elevation_;

public:
  static const size_t dimension = 2;

  /// @name Constructors
  /// @{

  /** Default constructor - straight ahead */
  BearingS2() {}

  /** Build from components */
  BearingS2(double azimuth, double elevation)
  : azimuth_(Rot2::fromAngle(azimuth)), elevation_(Rot2::fromAngle(elevation)) {}

  BearingS2(const Rot2& azimuth, const Rot2& elevation)
  : azimuth_(azimuth), elevation_(elevation) {}

  // access
  const Rot2& azimuth() const { return azimuth_; }
  const Rot2& elevation() const { return elevation_; }

  /// @}
  /// @name Measurements
  /// @{

  /**
   * Observation function for downwards-facing camera
   */
  // FIXME: will not work for TARGET = Point3
  template<class POSE, class TARGET>
  static BearingS2 fromDownwardsObservation(const POSE& A, const TARGET& B) {
    return fromDownwardsObservation(A.pose(), B.translation());
  }

  static BearingS2 fromDownwardsObservation(const Pose3& A, const Point3& B);

  /** Observation function with standard, forwards-facing camera */
  static BearingS2 fromForwardObservation(const Pose3& A, const Point3& B);

  /// @}
  /// @name Testable
  /// @{

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const BearingS2& x, double tol = 1e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// Dimensionality of tangent space = 2 DOF - used to autodetect sizes
  inline static size_t Dim() { return dimension; }

  /// Dimensionality of tangent space = 2 DOF
  inline size_t dim() const { return dimension; }

  /// Retraction from R^2 to BearingS2 manifold neighborhood around current pose
  /// Tangent space parameterization is [azimuth elevation]
  BearingS2 retract(const Vector& v) const;

  /// Local coordinates of BearingS2 manifold neighborhood around current pose
  Vector localCoordinates(const BearingS2& p2) const;

private:

  /// @}
  /// @name Advanced Interface
  /// @{

  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(azimuth_);
    ar & BOOST_SERIALIZATION_NVP(elevation_);
  }

};

/// traits
template<> struct traits<BearingS2> : public Testable<BearingS2> {};

} // \namespace gtsam
