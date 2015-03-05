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
#include <gtsam/geometry/Point2.h>
#include <boost/make_shared.hpp>

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a *fixed* Calibration.
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class GTSAM_EXPORT PinholeBaseK: public PinholeBase {

private:

  GTSAM_CONCEPT_MANIFOLD_TYPE(Calibration);

  // Get dimensions of calibration type at compile time
  static const int DimK = FixedDimension<Calibration>::value;

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  PinholeBaseK() {
  }

  /** constructor with pose */
  explicit PinholeBaseK(const Pose3& pose) :
  PinholeBase(pose) {
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit PinholeBaseK(const Vector &v) :
  PinholeBase(v) {
  }

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~PinholeBaseK() {
  }

  /// return calibration
  virtual const Calibration& calibration() const = 0;

  /// @}
  /// @name Transformations and measurement functions
  /// @{

  /// Project a point into the image and check depth
  std::pair<Point2, bool> projectSafe(const Point3& pw) const {
    std::pair<Point2, bool> pn = PinholeBase::projectSafe(pw);
    pn.first = calibration().uncalibrate(pn.first);
    return pn;
  }

  /** project a point from world coordinate to the image
   *  @param pw is a point in the world coordinates
   */
  Point2 project(const Point3& pw) const {

    // project to normalized coordinates
    const Point2 pn = PinholeBase::project2(pw);

    // uncalibrate to pixel coordinates
    return calibration().uncalibrate(pn);
  }

  /** project a point from world coordinate to the image, w 2 derivatives
   *  @param pw is a point in the world coordinates
   */
  Point2 project2(const Point3& pw, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none) const {

    // project to normalized coordinates
    const Point2 pn = PinholeBase::project2(pw, Dpose, Dpoint);

    // uncalibrate to pixel coordinates
    Matrix2 Dpi_pn;
    const Point2 pi = calibration().uncalibrate(pn, boost::none,
        Dpose || Dpoint ? &Dpi_pn : 0);

    // If needed, apply chain rule
    if (Dpose) *Dpose = Dpi_pn * (*Dpose);
    if (Dpoint) *Dpoint = Dpi_pn * (*Dpoint);

    return pi;
  }

  /** project a point at infinity from world coordinate to the image
   *  @param pw is a point in the world coordinate (it is pw = lambda*[pw_x  pw_y  pw_z] with lambda->inf)
   *  @param Dpose is the Jacobian w.r.t. pose3
   *  @param Dpoint is the Jacobian w.r.t. point3
   *  @param Dcal is the Jacobian w.r.t. calibration
   */
  Point2 projectPointAtInfinity(const Point3& pw, OptionalJacobian<2, 6> Dpose =
      boost::none, OptionalJacobian<2, 2> Dpoint = boost::none,
      OptionalJacobian<2, DimK> Dcal = boost::none) const {

    if (!Dpose && !Dpoint && !Dcal) {
      const Point3 pc = this->pose().rotation().unrotate(pw); // get direction in camera frame (translation does not matter)
      const Point2 pn = PinholeBase::project_to_camera(pc);// project the point to the camera
      return calibration().uncalibrate(pn);
    }

    // world to camera coordinate
    Matrix3 Dpc_rot, Dpc_point;
    const Point3 pc = this->pose().rotation().unrotate(pw, Dpc_rot, Dpc_point);

    // only rotation is important
    Matrix36 Dpc_pose;
    Dpc_pose.setZero();
    Dpc_pose.leftCols<3>() = Dpc_rot;

    // camera to normalized image coordinate
    Matrix23 Dpn_pc;// 2*3
    const Point2 pn = PinholeBase::project_to_camera(pc, Dpn_pc);

    // uncalibration
    Matrix2 Dpi_pn;// 2*2
    const Point2 pi = calibration().uncalibrate(pn, Dcal, Dpi_pn);

    // chain the Jacobian matrices
    const Matrix23 Dpi_pc = Dpi_pn * Dpn_pc;
    if (Dpose)
    *Dpose = Dpi_pc * Dpc_pose;
    if (Dpoint)
    *Dpoint = (Dpi_pc * Dpc_point).leftCols<2>();// only 2dof are important for the point (direction-only)
    return pi;
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  Point3 backproject(const Point2& p, double depth) const {
    const Point2 pn = calibration().calibrate(p);
    return pose().transform_from(backproject_from_camera(pn, depth));
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at infinity
  Point3 backprojectPointAtInfinity(const Point2& p) const {
    const Point2 pn = calibration().calibrate(p);
    const Point3 pc(pn.x(), pn.y(), 1.0); //by convention the last element is 1
    return pose().rotation().rotate(pc);
  }

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point,
      OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 3> Dpoint = boost::none) const {
    return pose().range(point, Dcamera, Dpoint);
  }

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @return range (double)
   */
  double range(const Pose3& pose, OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 6> Dpose = boost::none) const {
    return this->pose().range(pose, Dcamera, Dpose);
  }

  /**
   * Calculate range to a CalibratedCamera
   * @param camera Other camera
   * @return range (double)
   */
  double range(const CalibratedCamera& camera, OptionalJacobian<1, 6> Dcamera =
      boost::none, OptionalJacobian<1, 6> Dother = boost::none) const {
    return pose().range(camera.pose(), Dcamera, Dother);
  }

  /**
   * Calculate range to a PinholePoseK derived class
   * @param camera Other camera
   * @return range (double)
   */
  template<class CalibrationB>
  double range(const PinholeBaseK<CalibrationB>& camera,
      OptionalJacobian<1, 6> Dcamera = boost::none,
      OptionalJacobian<1, 6> Dother = boost::none) const {
    return pose().range(camera.pose(), Dcamera, Dother);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar
    & boost::serialization::make_nvp("PinholeBase",
        boost::serialization::base_object<PinholeBase>(*this));
  }

};
// end of class PinholeBaseK

/**
 * A pinhole camera class that has a Pose3 and a *fixed* Calibration.
 * Instead of using this class, one might consider calibrating the measurements
 * and using CalibratedCamera, which would then be faster.
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class GTSAM_EXPORT PinholePose: public PinholeBaseK<Calibration> {

private:

  typedef PinholeBaseK<Calibration> Base; ///< base class has 3D pose as private member
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
    return PinholePose(Base::LevelPose(pose2, height), K);
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
    return PinholePose(Base::LookatPose(eye, target, upVector), K);
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// Init from 6D vector
  explicit PinholePose(const Vector &v) :
      Base(v), K_(new Calibration()) {
  }

  /// Init from Vector and calibration
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

  /// @deprecated
  size_t dim() const {
    return 6;
  }

  /// @deprecated
  static size_t Dim() {
    return 6;
  }

  /// move a cameras according to d
  PinholePose retract(const Vector6& d) const {
    return PinholePose(Base::pose().retract(d), K_);
  }

  /// return canonical coordinate
  Vector6 localCoordinates(const PinholePose& p) const {
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
        & boost::serialization::make_nvp("PinholeBaseK",
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
