/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   PinholePose.h
 * @brief  Pinhole camera with known calibration
 * @author Yong-Dian Jian
 * @author Frank Dellaert
 * @date   Feb 20, 2015
 */

#pragma once

#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <boost/make_shared.hpp>
#include <cmath>

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a *fixed* Calibration.
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class PinholePose {

private:
  Pose3 pose_;
  boost::shared_ptr<Calibration> K_;

public:

  enum { dimension = 6 };

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  PinholePose() {
  }

  /** constructor with pose */
  explicit PinholePose(const Pose3& pose) :
      pose_(pose), K_(new Calibration()) {
  }

  /** constructor with pose and calibration */
  PinholePose(const Pose3& pose, const boost::shared_ptr<Calibration>& K) :
      pose_(pose), K_(K) {
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
  static PinholePose Level(const boost::shared_ptr<Calibration>& K, const Pose2& pose2,
      double height) {
    const double st = sin(pose2.theta()), ct = cos(pose2.theta());
    const Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
    const Rot3 wRc(x, y, z);
    const Point3 t(pose2.x(), pose2.y(), height);
    const Pose3 pose3(wRc, t);
    return PinholePose(pose3, K);
  }

  /// PinholePose::level with default calibration
  static PinholePose Level(const Pose2& pose2, double height) {
    return PinholePose::Level(boost::make_shared<Calibration>(), pose2, height);
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
  static PinholePose Lookat(const Point3& eye, const Point3& target,
      const Point3& upVector, const boost::shared_ptr<Calibration>& K =
          boost::make_shared<Calibration>()) {
    Point3 zc = target - eye;
    zc = zc / zc.norm();
    Point3 xc = (-upVector).cross(zc); // minus upVector since yc is pointing down
    xc = xc / xc.norm();
    Point3 yc = zc.cross(xc);
    Pose3 pose3(Rot3(xc, yc, zc), eye);
    return PinholePose(pose3, K);
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit PinholePose(const Vector &v) {
    pose_ = Pose3::Expmap(v);
  }

  PinholePose(const Vector &v, const Vector &K) :
      pose_(Pose3::Expmap(v)), K_(K) {
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const PinholePose &camera, double tol = 1e-9) const {
    return pose_.equals(camera.pose(), tol);
  }

  /// print
  void print(const std::string& s = "PinholePose") const {
    pose_.print(s + ".pose");
    K_->print(s + ".calibration");
  }

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~PinholePose() {
  }

  /// return pose
  inline Pose3& pose() {
    return pose_;
  }

  /// return pose, constant version
  inline const Pose3& pose() const {
    return pose_;
  }

  /// return pose, with derivative
  inline const Pose3& getPose(gtsam::OptionalJacobian<6, 6> H) const {
    if (H) {
      H->setZero();
      H->block(0, 0, 6, 6) = I_6x6;
    }
    return pose_;
  }

  /// return calibration
  inline boost::shared_ptr<Calibration> calibration() {
    return K_;
  }

  /// return calibration
  inline const boost::shared_ptr<Calibration> calibration() const {
    return K_;
  }

  /// @}
  /// @name Manifold
  /// @{

  /// Manifold 6
  inline size_t dim() const {
    return 6;
  }

  /// Manifold 6
  inline static size_t Dim() {
    return 6;
  }

  typedef Eigen::Matrix<double, 6, 1> VectorK6;

  /// move a cameras according to d
  PinholePose retract(const Vector6& d) const {
    return PinholePose(pose().retract(d), calibration());
  }

  /// return canonical coordinate
  VectorK6 localCoordinates(const PinholePose& p) const {
    return pose().localCoordinates(p.pose());
  }

  /// for Canonical
  static PinholePose identity() {
    return PinholePose(); // assumes that the default constructor is valid
  }

  /// @}
  /// @name Transformations and measurement functions
  /// @{

  /**
   * projects a 3-dimensional point in camera coordinates into the
   * camera and returns a 2-dimensional point, no calibration applied
   * @param P A point in camera coordinates
   * @param Dpoint is the 2*3 Jacobian w.r.t. P
   */
  static Point2 project_to_camera(const Point3& P, //
      OptionalJacobian<2, 3> Dpoint = boost::none) {
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
    if (P.z() <= 0)
      throw CheiralityException();
#endif
    double d = 1.0 / P.z();
    const double u = P.x() * d, v = P.y() * d;
    if (Dpoint)
      *Dpoint << d, 0.0, -u * d, 0.0, d, -v * d;
    return Point2(u, v);
  }

  /// Project a point into the image and check depth
  inline std::pair<Point2, bool> projectSafe(const Point3& pw) const {
    const Point3 pc = pose_.transform_to(pw);
    const Point2 pn = project_to_camera(pc);
    return std::make_pair(K_->uncalibrate(pn), pc.z() > 0);
  }

  /** project a point from world coordinate to the image, fixed Jacobians
   *  @param pw is a point in the world coordinate
   *  @param Dcamera is the Jacobian w.r.t. [pose3 calibration]
   *  @param Dpoint is the Jacobian w.r.t. point3
   */
  Point2 project2(
      const Point3& pw,
      OptionalJacobian<2, 6> Dcamera = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none) const {

    const Point3 pc = pose_.transform_to(pw);
    const Point2 pn = project_to_camera(pc);

    if (!Dcamera && !Dpoint) {
      return K_->uncalibrate(pn);
    } else {
      const double z = pc.z(), d = 1.0 / z;

      // uncalibration
      Matrix2 Dpi_pn;
      const Point2 pi = K_->uncalibrate(pn, boost::none, Dpi_pn);

      if (Dcamera)
        calculateDpose(pn, d, Dpi_pn, *Dcamera);
      if (Dpoint)
        calculateDpoint(pn, d, pose_.rotation().matrix(), Dpi_pn, *Dpoint);

      return pi;
    }
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  inline Point3 backproject(const Point2& p, double depth) const {
    const Point2 pn = K_->calibrate(p);
    const Point3 pc(pn.x() * depth, pn.y() * depth, depth);
    return pose_.transform_from(pc);
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at infinity
  inline Point3 backprojectPointAtInfinity(const Point2& p) const {
    const Point2 pn = K_->calibrate(p);
    const Point3 pc(pn.x(), pn.y(), 1.0); //by convention the last element is 1
    return pose_.rotation().rotate(pc);
  }

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @param Dcamera the optionally computed Jacobian with respect to pose
   * @param Dpoint the optionally computed Jacobian with respect to the landmark
   * @return range (double)
   */
  double range(
      const Point3& point, //
      OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 3> Dpoint = boost::none) const {
    return pose_.range(point, Dcamera, Dpoint);
  }

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @param Dcamera the optionally computed Jacobian with respect to pose
   * @param Dpose2 the optionally computed Jacobian with respect to the other pose
   * @return range (double)
   */
  double range(
      const Pose3& pose, //
      OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 6> Dpose = boost::none) const {
    return pose_.range(pose, Dcamera, Dpose);
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @param Dcamera the optionally computed Jacobian with respect to pose
   * @param Dother the optionally computed Jacobian with respect to the other camera
   * @return range (double)
   */
  template<class CalibrationB>
  double range(
      const PinholePose<CalibrationB>& camera, //
      OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 6> Dother = boost::none) const {
    return pose_.range(camera.pose(), Dcamera, Dother);
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @param Dcamera the optionally computed Jacobian with respect to pose
   * @param Dother the optionally computed Jacobian with respect to the other camera
   * @return range (double)
   */
  double range(
      const CalibratedCamera& camera, //
      OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 6> Dother = boost::none) const {
    return range(camera.pose(), Dcamera, Dother);
  }

private:

  /**
   * Calculate Jacobian with respect to pose
   * @param pn projection in normalized coordinates
   * @param d disparity (inverse depth)
   * @param Dpi_pn derivative of uncalibrate with respect to pn
   * @param Dpose Output argument, can be matrix or block, assumed right size !
   * See http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
   */
  template<typename Derived>
  static void calculateDpose(const Point2& pn, double d, const Matrix2& Dpi_pn,
      Eigen::MatrixBase<Derived> const & Dpose) {
    // optimized version of derivatives, see CalibratedCamera.nb
    const double u = pn.x(), v = pn.y();
    double uv = u * v, uu = u * u, vv = v * v;
    Matrix26 Dpn_pose;
    Dpn_pose << uv, -1 - uu, v, -d, 0, d * u, 1 + vv, -uv, -u, 0, -d, d * v;
    assert(Dpose.rows()==2 && Dpose.cols()==6);
    const_cast<Eigen::MatrixBase<Derived>&>(Dpose) = //
        Dpi_pn * Dpn_pose;
  }

  /**
   * Calculate Jacobian with respect to point
   * @param pn projection in normalized coordinates
   * @param d disparity (inverse depth)
   * @param Dpi_pn derivative of uncalibrate with respect to pn
   * @param Dpoint Output argument, can be matrix or block, assumed right size !
   * See http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
   */
  template<typename Derived>
  static void calculateDpoint(const Point2& pn, double d, const Matrix3& R,
      const Matrix2& Dpi_pn, Eigen::MatrixBase<Derived> const & Dpoint) {
    // optimized version of derivatives, see CalibratedCamera.nb
    const double u = pn.x(), v = pn.y();
    Matrix23 Dpn_point;
    Dpn_point << //
        R(0, 0) - u * R(0, 2), R(1, 0) - u * R(1, 2), R(2, 0) - u * R(2, 2), //
    /**/R(0, 1) - v * R(0, 2), R(1, 1) - v * R(1, 2), R(2, 1) - v * R(2, 2);
    Dpn_point *= d;
    assert(Dpoint.rows()==2 && Dpoint.cols()==3);
    const_cast<Eigen::MatrixBase<Derived>&>(Dpoint) = //
        Dpi_pn * Dpn_point;
  }

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(pose_);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }

};


template<typename Calibration>
struct traits< PinholePose<Calibration> > : public internal::Manifold<PinholePose<Calibration> > {};

template<typename Calibration>
struct traits< const PinholePose<Calibration> > : public internal::Manifold<PinholePose<Calibration> > {};

} // \ gtsam
