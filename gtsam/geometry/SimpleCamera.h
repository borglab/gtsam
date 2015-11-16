/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SimpleCamera.h
 * @brief A simple camera class with a Cal3_S2 calibration
 * @date Aug 16, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {

  /// A simple camera class with a Cal3_S2 calibration
typedef gtsam::PinholeCamera<gtsam::Cal3_S2> PinholeCameraCal3_S2;

/**
 * @deprecated: SimpleCamera for backwards compatability with GTSAM 3.x
 * Use PinholeCameraCal3_S2 instead
 */
class SimpleCamera : public PinholeCameraCal3_S2 {

  typedef PinholeCamera<Cal3_S2> Base;
  typedef boost::shared_ptr<SimpleCamera> shared_ptr;

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  SimpleCamera() :
      Base() {
  }

  /** constructor with pose */
  explicit SimpleCamera(const Pose3& pose) :
      Base(pose) {
  }

  /** constructor with pose and calibration */
  SimpleCamera(const Pose3& pose, const Cal3_S2& K) :
      Base(pose, K) {
  }

  /// @}
   /// @name Named Constructors
   /// @{

   /**
    * Create a level camera at the given 2D pose and height
    * @param K the calibration
    * @param pose2 specifies the location and viewing direction
    * (theta 0 = looking in direction of positive X axis)
    * @param height camera height
    */
   static SimpleCamera Level(const Cal3_S2 &K, const Pose2& pose2,
       double height) {
     return SimpleCamera(Base::LevelPose(pose2, height), K);
   }

   /// PinholeCamera::level with default calibration
   static SimpleCamera Level(const Pose2& pose2, double height) {
    return SimpleCamera::Level(Cal3_S2(), pose2, height);
   }

   /**
    * Create a camera at the given eye position looking at a target point in the scene
    * with the specified up direction vector.
    * @param eye specifies the camera position
    * @param target the point to look at
    * @param upVector specifies the camera up direction vector,
    *        doesn't need to be on the image plane nor orthogonal to the viewing axis
    * @param K optional calibration parameter
    */
   static SimpleCamera Lookat(const Point3& eye, const Point3& target,
       const Point3& upVector, const Cal3_S2& K = Cal3_S2()) {
    return SimpleCamera(Base::LookatPose(eye, target, upVector), K);
   }

   /// @}
   /// @name Advanced Constructors
   /// @{

   /// Init from vector, can be 6D (default calibration) or dim
   explicit SimpleCamera(const Vector &v) :
       Base(v) {
   }

   /// Init from Vector and calibration
   SimpleCamera(const Vector &v, const Vector &K) :
       Base(v, K) {
   }

   /// Copy this object as its actual derived type.
   SimpleCamera::shared_ptr clone() const { return boost::make_shared<SimpleCamera>(*this); }


   /// @}
   /// @name Manifold
   /// @{

   /// move a cameras according to d
   SimpleCamera retract(const Vector& d) const {
     if ((size_t) d.size() == 6)
       return SimpleCamera(this->pose().retract(d), calibration());
     else
       return SimpleCamera(this->pose().retract(d.head(6)),
           calibration().retract(d.tail(calibration().dim())));
   }

   /// @}

};

/// Recover camera from 3*4 camera matrix
GTSAM_EXPORT SimpleCamera simpleCamera(const Matrix34& P);

// manifold traits
template <>
struct traits<SimpleCamera> : public internal::Manifold<SimpleCamera> {};

template <>
struct traits<const SimpleCamera> : public internal::Manifold<SimpleCamera> {};

// range traits, used in RangeFactor
template <typename T>
struct Range<SimpleCamera, T> : HasRange<SimpleCamera, T, double> {};

}  // namespace gtsam
