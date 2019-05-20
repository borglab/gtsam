/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StereoCamera.h
 * @brief   A Rectified Stereo Camera
 * @author  Chris Beall
 */

#pragma once

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>

namespace gtsam {

class GTSAM_EXPORT StereoCheiralityException: public std::runtime_error {
public:
  StereoCheiralityException()
    : StereoCheiralityException(std::numeric_limits<Key>::max()) {}

  StereoCheiralityException(Key j)
    : std::runtime_error("Stereo Cheirality Exception"),
      j_(j) {}

  Key nearbyVariable() const {
    return j_;
  }

private:
  Key j_;
};

/**
 * A stereo camera class, parameterize by left camera pose and stereo calibration
 * @addtogroup geometry
 */
class GTSAM_EXPORT StereoCamera {

public:

  /**
   *  Some classes template on either PinholeCamera or StereoCamera,
   *  and this typedef informs those classes what "project" returns.
   */
  typedef StereoPoint2 Measurement;
  typedef StereoPoint2Vector MeasurementVector;

private:
  Pose3 leftCamPose_;
  Cal3_S2Stereo::shared_ptr K_;

public:

  enum {
    dimension = 6
  };

  /// @name Standard Constructors
  /// @{

  /// Default constructor allocates a calibration!
  StereoCamera() :
      K_(new Cal3_S2Stereo()) {
  }

  /// Construct from pose and shared calibration
  StereoCamera(const Pose3& leftCamPose, const Cal3_S2Stereo::shared_ptr K);

  /// Return shared pointer to calibration
  const Cal3_S2Stereo& calibration() const {
    return *K_;
  }

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const {
    leftCamPose_.print(s + ".camera.");
    K_->print(s + ".calibration.");
  }

  /// equals
  bool equals(const StereoCamera &camera, double tol = 1e-9) const {
    return leftCamPose_.equals(camera.leftCamPose_, tol)
        && K_->equals(*camera.K_, tol);
  }

  /// @}
  /// @name Manifold
  /// @{

  /// Dimensionality of the tangent space
  inline size_t dim() const {
    return 6;
  }

  /// Dimensionality of the tangent space
  static inline size_t Dim() {
    return 6;
  }

  /// Updates a with tangent space delta
  inline StereoCamera retract(const Vector& v) const {
    return StereoCamera(pose().retract(v), K_);
  }

  /// Local coordinates of manifold neighborhood around current value
  inline Vector6 localCoordinates(const StereoCamera& t2) const {
    return leftCamPose_.localCoordinates(t2.leftCamPose_);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// pose
  const Pose3& pose() const {
    return leftCamPose_;
  }

  /// baseline
  double baseline() const {
    return K_->baseline();
  }

  /// Project 3D point to StereoPoint2 (uL,uR,v)
  StereoPoint2 project(const Point3& point) const;

  /** Project 3D point and compute optional derivatives
   * @param H1 derivative with respect to pose
   * @param H2 derivative with respect to point
   */
  StereoPoint2 project2(const Point3& point, OptionalJacobian<3, 6> H1 =
      boost::none, OptionalJacobian<3, 3> H2 = boost::none) const;

  /// back-project a measurement
  Point3 backproject(const StereoPoint2& z) const;

  /** Back-project the 2D point and compute optional derivatives
   * @param H1 derivative with respect to pose
   * @param H2 derivative with respect to point
   */
  Point3 backproject2(const StereoPoint2& z,
                      OptionalJacobian<3, 6> H1 = boost::none,
                      OptionalJacobian<3, 3> H2 = boost::none) const;

  /// @}
  /// @name Deprecated
  /// @{

  /** Project 3D point and compute optional derivatives
   * @deprecated, use project2 - this class has fixed calibration
   * @param H1 derivative with respect to pose
   * @param H2 derivative with respect to point
   * @param H3 IGNORED (for calibration)
   */
  StereoPoint2 project(const Point3& point, OptionalJacobian<3, 6> H1,
      OptionalJacobian<3, 3> H2 = boost::none, OptionalJacobian<3, 0> H3 =
          boost::none) const;

  /// @}

private:

  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(leftCamPose_);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }

};

template<>
struct traits<StereoCamera> : public internal::Manifold<StereoCamera> {
};

template<>
struct traits<const StereoCamera> : public internal::Manifold<StereoCamera> {
};
}
