
/**
 * @file DepthCamera3.h
 * @brief  Depth Camera.
 * @author junlinp
 * @date Nov 9, 2025
 */

#pragma once

#include <iostream>
#include <optional>
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
  std::optional<Pose3> body_P_sensor_;  ///< The pose of the sensor in the body frame

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  DepthCamera3() {}

  /** constructor with pose and calibration */
  DepthCamera3(const Point3& measurement, const std::shared_ptr<CALIBRATION>& k) :
    measurement_{measurement}, k_(k) {}

  /** constructor with pose, calibration, and body_P_sensor */
  DepthCamera3(const Point3& measurement, const std::shared_ptr<CALIBRATION>& k,
               const std::optional<Pose3>& body_P_sensor) :
    measurement_{measurement}, k_(k), body_P_sensor_(body_P_sensor) {}

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
    if (body_P_sensor_) {
      body_P_sensor_->print("  body_P_sensor: ");
    }
  }


  /** project a point from world InvDepth parameterization to the image
   *  @param pose is the body pose in world frame
   *  @param landmark is a point in the world coordinate
   *  @param H1 is the jacobian w.r.t. body pose
   *  @param H2 is the jacobian w.r.t. landmark
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
    
    // Convert body pose to camera pose using body_P_sensor_
    Pose3 camera_pose_world;
    if (body_P_sensor_) {
      if (H1) {
        Matrix HbodySensor;
        camera_pose_world = pose.compose(*body_P_sensor_, HbodySensor);
        // Transform landmark from world to camera frame
        Matrix Hcam;
        Point3 predicted_cam = camera_pose_world.transformTo(landmark, Hcam, H2);
        *H1 = Hcam * HbodySensor;
        return predicted_cam - observation;
      } else {
        camera_pose_world = pose.compose(*body_P_sensor_);
      }
    } else {
      camera_pose_world = pose;
    }
    
    // Transform landmark from world to camera frame
    Point3 predicted_cam;
    if (H1 || H2) {
      Matrix Hcam;
      predicted_cam = camera_pose_world.transformTo(landmark, Hcam, H2);
      if (H1) {
        *H1 = Hcam;
      }
    } else {
      predicted_cam = camera_pose_world.transformTo(landmark);
    }
    
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
    ar & BOOST_SERIALIZATION_NVP(body_P_sensor_);
  }
#endif
  /// @}
}; // \class DepthCamera3
} // \namespace gtsam

