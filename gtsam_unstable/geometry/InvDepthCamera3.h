
/**
 * @file InvDepthCamera3.h
 * @brief Inverse Depth Camera based on Civera09tro, Montiel06rss.
 * Landmarks are initialized from the first camera observation with
 * (x,y,z,theta,phi,inv_depth), where x,y,z are the coordinates of
 * the camera
 * @author Chris Beall
 * @date Apr 14, 2012
 */

#pragma once

#include <cmath>
#include <boost/optional.hpp>
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/PinholeCamera.h>

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a Calibration.
 * @ingroup geometry
 * \nosubgrouping
 */
template <class CALIBRATION>
class InvDepthCamera3 {
private:
  Pose3 pose_;                        ///< The camera pose
  boost::shared_ptr<CALIBRATION> k_;  ///< The fixed camera calibration

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  InvDepthCamera3() {}

  /** constructor with pose and calibration */
  InvDepthCamera3(const Pose3& pose, const boost::shared_ptr<CALIBRATION>& k) :
    pose_(pose),k_(k) {}

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~InvDepthCamera3() {}

  /// return pose
  inline Pose3& pose() {  return pose_; }

  /// return calibration
  inline const boost::shared_ptr<CALIBRATION>& calibration() const {  return k_; }

  /// print
  void print(const std::string& s = "") const {
    pose_.print("pose3");
    k_.print("calibration");
  }

  /**
   * Convert an inverse depth landmark to cartesian Point3
   * @param pw first five parameters (x,y,z,theta,phi) of inv depth landmark
   * @param inv inverse depth
   * @return Point3
   */
  static gtsam::Point3 invDepthTo3D(const Vector5& pw, double rho) {
    gtsam::Point3 ray_base(pw.segment(0,3));
    double theta = pw(3), phi = pw(4);
    gtsam::Point3 m(cos(theta)*cos(phi),sin(theta)*cos(phi),sin(phi));
    return ray_base + m/rho;
  }

  /** project a point from world InvDepth parameterization to the image
   *  @param pw is a point in the world coordinate
   *  @param H1 is the jacobian w.r.t. [pose3 calibration]
   *  @param H2 is the jacobian w.r.t. inv_depth_landmark
   */
  inline gtsam::Point2 project(const Vector5& pw,
      double rho,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none,
      boost::optional<gtsam::Matrix&> H3 = boost::none) const {

    gtsam::Point3 ray_base(pw.segment(0,3));
    double theta = pw(3), phi = pw(4);
    gtsam::Point3 m(cos(theta)*cos(phi),sin(theta)*cos(phi),sin(phi));
    const gtsam::Point3 landmark = ray_base + m/rho;

    gtsam::PinholeCamera<CALIBRATION> camera(pose_, *k_);

    if (!H1 && !H2 && !H3) {
      gtsam::Point2 uv= camera.project(landmark);
      return uv;
    }
    else {
      gtsam::Matrix J2;
      gtsam::Point2 uv= camera.project(landmark,H1, J2, boost::none);
      if (H1) {
        *H1 = (*H1) * I_6x6;
      }

      double cos_theta = cos(theta);
      double sin_theta = sin(theta);
      double cos_phi = cos(phi);
      double sin_phi = sin(phi);
      double rho2 = rho * rho;

      if (H2) {
        double H11 = 1;
        double H12 = 0;
        double H13 = 0;
        double H14 = -cos_phi*sin_theta/rho;
        double H15 = -cos_theta*sin_phi/rho;

        double H21 = 0;
        double H22 = 1;
        double H23 = 0;
        double H24 = cos_phi*cos_theta/rho;
        double H25 = -sin_phi*sin_theta/rho;

        double H31 = 0;
        double H32 = 0;
        double H33 = 1;
        double H34 = 0;
        double H35 = cos_phi/rho;

        *H2 = J2 * (Matrix(3, 5) <<
            H11, H12, H13, H14, H15,
            H21, H22, H23, H24, H25,
            H31, H32, H33, H34, H35).finished();
      }
      if(H3) {
        double H16 = -cos_phi*cos_theta/rho2;
        double H26 = -cos_phi*sin_theta/rho2;
        double H36 = -sin_phi/rho2;
        *H3 = J2 * (Matrix(3, 1) <<
            H16,
            H26,
            H36).finished();
      }
      return uv;
    }
  }

  /**
   * backproject a 2-dimensional point to an Inverse Depth landmark
   * useful for point initialization
   */

  inline std::pair<Vector5, double> backproject(const gtsam::Point2& pi, const double depth) const {
    const gtsam::Point2 pn = k_->calibrate(pi);
    gtsam::Point3 pc(pn.x(), pn.y(), 1.0);
    pc = pc/pc.norm();
    gtsam::Point3 pw = pose_.transformFrom(pc);
    const gtsam::Point3& pt = pose_.translation();
    gtsam::Point3 ray = pw - pt;
    double theta = atan2(ray.y(), ray.x()); // longitude
    double phi = atan2(ray.z(), sqrt(ray.x()*ray.x()+ray.y()*ray.y()));
    return std::make_pair((Vector5() << pt.x(),pt.y(),pt.z(), theta, phi).finished(),
        double(1./depth));
  }

private:

  /// @}
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(pose_);
    ar & BOOST_SERIALIZATION_NVP(k_);
  }
  /// @}
}; // \class InvDepthCamera
} // \namesapce gtsam
