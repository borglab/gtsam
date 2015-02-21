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
class PinholeBase {

private:

  Pose3 pose_; ///< 3D pose of camera

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  PinholeBase() {
  }

  /** constructor with pose */
  explicit PinholeBase(const Pose3& pose) :
      pose_(pose) {
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit PinholeBase(const Vector &v) :
      pose_(Pose3::Expmap(v)) {
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const PinholeBase &camera, double tol = 1e-9) const {
    return pose_.equals(camera.pose(), tol);
  }

  /// print
  void print(const std::string& s = "PinholeBase") const {
    pose_.print(s + ".pose");
  }

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~PinholeBase() {
  }

  /// return pose, constant version
  const Pose3& pose() const {
    return pose_;
  }

  /// return pose, with derivative
  const Pose3& pose(OptionalJacobian<6, 6> H) const {
    if (H) {
      H->setZero();
      H->block(0, 0, 6, 6) = I_6x6;
    }
    return pose_;
  }

  /// return calibration
  virtual const Calibration& calibration() const = 0;

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
  std::pair<Point2, bool> projectSafe(const Point3& pw) const {
    const Point3 pc = pose_.transform_to(pw);
    const Point2 pn = project_to_camera(pc);
    return std::make_pair(calibration().uncalibrate(pn), pc.z() > 0);
  }

  /** project a point from world coordinate to the image, fixed Jacobians
   *  @param pw is a point in the world coordinate
   *  @param Dcamera is the Jacobian w.r.t. [pose3 calibration]
   *  @param Dpoint is the Jacobian w.r.t. point3
   */
  Point2 project2(const Point3& pw,
      OptionalJacobian<2, 6> Dcamera = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none) const {

    const Point3 pc = pose_.transform_to(pw);
    const Point2 pn = project_to_camera(pc);

    if (!Dcamera && !Dpoint) {
      return calibration().uncalibrate(pn);
    } else {
      const double z = pc.z(), d = 1.0 / z;

      // uncalibration
      Matrix2 Dpi_pn;
      const Point2 pi = calibration().uncalibrate(pn, boost::none, Dpi_pn);

      if (Dcamera)
        calculateDpose(pn, d, Dpi_pn, *Dcamera);
      if (Dpoint)
        calculateDpoint(pn, d, pose_.rotation().matrix(), Dpi_pn, *Dpoint);

      return pi;
    }
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  Point3 backproject(const Point2& p, double depth) const {
    const Point2 pn = calibration().calibrate(p);
    const Point3 pc(pn.x() * depth, pn.y() * depth, depth);
    return pose_.transform_from(pc);
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at infinity
  Point3 backprojectPointAtInfinity(const Point2& p) const {
    const Point2 pn = calibration().calibrate(p);
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
      const PinholeBase<CalibrationB>& camera, //
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

protected:

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

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(pose_);
  }

};
// end of class PinholeBase

/**
 * A pinhole camera class that has a Pose3 and a *fixed* Calibration.
 * Instead of using this class, one might consider calibrating the measurements
 * and using CalibratedCamera, which would then be faster.
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class PinholePose: public PinholeBase<Calibration> {

private:

  typedef PinholeBase<Calibration> Base; ///< base class has 3D pose as private member
  boost::shared_ptr<Calibration> K_; ///< shared pointer to fixed calibration

public:

  enum {
    dimension = 6
  }; ///< There are 6 DOF to optimize for

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  PinholePose() {
  }

  /** constructor with pose, uses default calibration */
  explicit PinholePose(const Pose3& pose) :
      Base(pose), K_(new Calibration()) {
  }

  /** constructor with pose and calibration */
  PinholePose(const Pose3& pose, const boost::shared_ptr<Calibration>& K) :
      Base(pose), K_(K) {
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
  static PinholePose Level(const boost::shared_ptr<Calibration>& K,
      const Pose2& pose2, double height) {
    return PinholePose(CalibratedCamera::LevelPose(pose2, height), K);
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
    return PinholePose(CalibratedCamera::LookatPose(eye, target, upVector), K);
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit PinholePose(const Vector &v) :
      Base(v), K_(new Calibration()) {
  }

  PinholePose(const Vector &v, const Vector &K) :
      Base(v), K_(new Calibration(K)) {
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const Base &camera, double tol = 1e-9) const {
    const PinholePose* e = dynamic_cast<const PinholePose*>(&camera);
    return Base::equals(camera, tol) && K_->equals(e->calibration(), tol);
  }

  /// print
  void print(const std::string& s = "PinholePose") const {
    Base::print(s);
    K_->print(s + ".calibration");
  }

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~PinholePose() {
  }

  /// return calibration
  virtual const Calibration& calibration() const {
    return *K_;
  }

  /// @}
  /// @name Manifold
  /// @{

  /// Manifold 6
  size_t dim() const {
    return 6;
  }

  /// Manifold 6
  static size_t Dim() {
    return 6;
  }

  typedef Eigen::Matrix<double, 6, 1> VectorK6;

  /// move a cameras according to d
  PinholePose retract(const Vector6& d) const {
    return PinholePose(Base::pose().retract(d), K_);
  }

  /// return canonical coordinate
  VectorK6 localCoordinates(const PinholePose& p) const {
    return Base::pose().localCoordinates(p.Base::pose());
  }

  /// for Canonical
  static PinholePose identity() {
    return PinholePose(); // assumes that the default constructor is valid
  }

  /// @}

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("PinholeBase",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(K_);
  }

};
// end of class PinholePose

template<typename Calibration>
struct traits<PinholePose<Calibration> > : public internal::Manifold<
    PinholePose<Calibration> > {
};

template<typename Calibration>
struct traits<const PinholePose<Calibration> > : public internal::Manifold<
    PinholePose<Calibration> > {
};

} // \ gtsam
