
/**
 * @file DepthCamera3.h
 * @brief  Depth Camera.
 * @author junlinp
 * @date Nov 9, 2025
 */

#pragma once

#include <iostream>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a Calibration.
 * @ingroup geometry
 * \nosubgrouping
 */
template <class CALIBRATION>
class DepthCamera3 {
private:
  Point3 measurement_;               ///< The depth measurement  (u, v, depth)
  std::shared_ptr<CALIBRATION> k_;  ///< The fixed camera calibration

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  DepthCamera3() {}

  /** constructor with pose and calibration */
  DepthCamera3(const Point3& measurement, const std::shared_ptr<CALIBRATION>& k) :
    measurement_{measurement}, k_(k) {}

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~DepthCamera3() {}


  /// return calibration
  inline const std::shared_ptr<CALIBRATION>& calibration() const {  return k_; }

  /// print
  void print(const std::string& s = "") const {
    std::cout << s << "DepthCamera3:" << std::endl;
    std::cout << "  measurement: " << measurement_.transpose() << std::endl;
    if (k_) k_->print("  calibration: ");
  }


  /** project a point from world InvDepth parameterization to the image
   *  @param pw is a point in the world coordinate
   *  @param H1 is the jacobian w.r.t. [pose3 calibration]
   *  @param H2 is the jacobian w.r.t. inv_depth_landmark
   */
  inline gtsam::Point3 project(const Pose3& pose, const Point3& landmark,
      OptionalJacobian<3,6> H1 = {},
      OptionalJacobian<3,3> H2 = {}
      ) const {
    double fx = k_->fx(); 
    double fy = k_->fy();
    double cx = k_->px();
    double cy = k_->py();
    Matrix33 inv_intrinsics = (Matrix33() << 1.0 / fx, 0.0, -cx / fx,
                                              0.0, 1.0 / fy, -cy / fy,
                                              0.0, 0.0, 1.0).finished();

    Point3 observation_raw(measurement_.x() * measurement_.z(),
                        measurement_.y() * measurement_.z(),
                        measurement_.z());

    Point3 observation = inv_intrinsics * observation_raw;
    
    // Compute predicted in camera frame
    Point3 predicted_cam = pose.transformTo(landmark, H1, H2);
    
    // Negate Jacobians because we're subtracting predicted_cam
    if (H1) *H1 = (*H1);
    if (H2) *H2 = (*H2);
    
    return predicted_cam - observation;
  }

private:

  /// @}
  /// @name Advanced Interface
  /// @{

#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(measurement_);
    ar & BOOST_SERIALIZATION_NVP(k_);
  }
#endif
  /// @}
}; // \class InvDepthCamera
} // \namespace gtsam





