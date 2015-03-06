/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file PinholeCamera.h
 * @brief Base class for all pinhole cameras
 * @author Yong-Dian Jian
 * @date Jan 27, 2012
 */

#pragma once

#include <gtsam/geometry/PinholePose.h>

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a Calibration.
 * Use PinholePose if you will not be optimizing for Calibration
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class GTSAM_EXPORT PinholeCamera: public PinholeBaseK<Calibration> {

public:

  /**
   *  Some classes template on either PinholeCamera or StereoCamera,
   *  and this typedef informs those classes what "project" returns.
   */
  typedef Point2 Measurement;

private:

  typedef PinholeBaseK<Calibration> Base; ///< base class has 3D pose as private member
  Calibration K_; ///< Calibration, part of class now

  // Get dimensions of calibration type at compile time
  static const int DimK = FixedDimension<Calibration>::value;

public:

  enum {
    dimension = 6 + DimK
  }; ///< Dimension depends on calibration

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  PinholeCamera() {
  }

  /** constructor with pose */
  explicit PinholeCamera(const Pose3& pose) :
      Base(pose) {
  }

  /** constructor with pose and calibration */
  PinholeCamera(const Pose3& pose, const Calibration& K) :
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
  static PinholeCamera Level(const Calibration &K, const Pose2& pose2,
      double height) {
    return PinholeCamera(Base::LevelPose(pose2, height), K);
  }

  /// PinholeCamera::level with default calibration
  static PinholeCamera Level(const Pose2& pose2, double height) {
    return PinholeCamera::Level(Calibration(), pose2, height);
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
  static PinholeCamera Lookat(const Point3& eye, const Point3& target,
      const Point3& upVector, const Calibration& K = Calibration()) {
    return PinholeCamera(Base::LookatPose(eye, target, upVector), K);
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// Init from vector, can be 6D (default calibration) or dim
  explicit PinholeCamera(const Vector &v) :
      Base(v.head<6>()) {
    if (v.size() > 6)
      K_ = Calibration(v.tail<DimK>());
  }

  /// Init from Vector and calibration
  PinholeCamera(const Vector &v, const Vector &K) :
      Base(v), K_(K) {
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const Base &camera, double tol = 1e-9) const {
    const PinholeCamera* e = dynamic_cast<const PinholeCamera*>(&camera);
    return Base::equals(camera, tol) && K_.equals(e->calibration(), tol);
  }

  /// print
  void print(const std::string& s = "PinholeCamera") const {
    Base::print(s);
    K_.print(s + ".calibration");
  }

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~PinholeCamera() {
  }

  /// return pose
  const Pose3& pose() const {
    return Base::pose();
  }

  /// return pose, with derivative
  const Pose3& getPose(OptionalJacobian<6, dimension> H) const {
    if (H) {
      H->setZero();
      H->block(0, 0, 6, 6) = I_6x6;
    }
    return Base::pose();
  }

  /// return calibration
  const Calibration& calibration() const {
    return K_;
  }

  /// @}
  /// @name Manifold
  /// @{

  /// @deprecated
  size_t dim() const {
    return dimension;
  }

  /// @deprecated
  static size_t Dim() {
    return dimension;
  }

  typedef Eigen::Matrix<double, dimension, 1> VectorK6;

  /// move a cameras according to d
  PinholeCamera retract(const Vector& d) const {
    if ((size_t) d.size() == 6)
      return PinholeCamera(this->pose().retract(d), calibration());
    else
      return PinholeCamera(this->pose().retract(d.head(6)),
          calibration().retract(d.tail(calibration().dim())));
  }

  /// return canonical coordinate
  VectorK6 localCoordinates(const PinholeCamera& T2) const {
    VectorK6 d;
    // TODO: why does d.head<6>() not compile??
    d.head(6) = this->pose().localCoordinates(T2.pose());
    d.tail(DimK) = calibration().localCoordinates(T2.calibration());
    return d;
  }

  /// for Canonical
  static PinholeCamera identity() {
    return PinholeCamera(); // assumes that the default constructor is valid
  }

  /// @}
  /// @name Transformations and measurement functions
  /// @{

  typedef Eigen::Matrix<double, 2, DimK> Matrix2K;

  /** project a point from world coordinate to the image
   *  @param pw is a point in world coordinates
   *  @param Dpose is the Jacobian w.r.t. pose3
   *  @param Dpoint is the Jacobian w.r.t. point3
   *  @param Dcal is the Jacobian w.r.t. calibration
   */
  Point2 project(const Point3& pw, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none,
      OptionalJacobian<2, DimK> Dcal = boost::none) const {

    // project to normalized coordinates
    const Point2 pn = PinholeBase::project2(pw, Dpose, Dpoint);

    // uncalibrate to pixel coordinates
    Matrix2 Dpi_pn;
    const Point2 pi = calibration().uncalibrate(pn, Dcal,
        Dpose || Dpoint ? &Dpi_pn : 0);

    // If needed, apply chain rule
    if (Dpose)
      *Dpose = Dpi_pn * *Dpose;
    if (Dpoint)
      *Dpoint = Dpi_pn * *Dpoint;

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
      const Point2 pn = Base::project_to_camera(pc); // project the point to the camera
      return K_.uncalibrate(pn);
    }

    // world to camera coordinate
    Matrix3 Dpc_rot, Dpc_point;
    const Point3 pc = this->pose().rotation().unrotate(pw, Dpc_rot, Dpc_point);

    Matrix36 Dpc_pose;
    Dpc_pose.setZero();
    Dpc_pose.leftCols<3>() = Dpc_rot;

    // camera to normalized image coordinate
    Matrix23 Dpn_pc; // 2*3
    const Point2 pn = Base::project_to_camera(pc, Dpn_pc);

    // uncalibration
    Matrix2 Dpi_pn; // 2*2
    const Point2 pi = K_.uncalibrate(pn, Dcal, Dpi_pn);

    // chain the Jacobian matrices
    const Matrix23 Dpi_pc = Dpi_pn * Dpn_pc;
    if (Dpose)
      *Dpose = Dpi_pc * Dpc_pose;
    if (Dpoint)
      *Dpoint = (Dpi_pc * Dpc_point).leftCols<2>(); // only 2dof are important for the point (direction-only)
    return pi;
  }

  /** project a point from world coordinate to the image, fixed Jacobians
   *  @param pw is a point in the world coordinate
   */
  Point2 project2(
      const Point3& pw, //
      OptionalJacobian<2, dimension> Dcamera = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none) const {

    // project to normalized coordinates
    Matrix26 Dpose;
    const Point2 pn = PinholeBase::project2(pw, Dpose, Dpoint);

    // uncalibrate to pixel coordinates
    Matrix2K Dcal;
    Matrix2 Dpi_pn;
    const Point2 pi = calibration().uncalibrate(pn, Dcamera ? &Dcal : 0,
        Dcamera || Dpoint ? &Dpi_pn : 0);

    // If needed, calculate derivatives
    if (Dcamera)
      *Dcamera << Dpi_pn * Dpose, Dcal;
    if (Dpoint)
      *Dpoint = Dpi_pn * (*Dpoint);

    return pi;
  }

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point, OptionalJacobian<1, dimension> Dcamera =
      boost::none, OptionalJacobian<1, 3> Dpoint = boost::none) const {
    Matrix16 Dpose_;
    double result = this->pose().range(point, Dcamera ? &Dpose_ : 0, Dpoint);
    if (Dcamera)
      *Dcamera << Dpose_, Eigen::Matrix<double, 1, DimK>::Zero();
    return result;
  }

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @return range (double)
   */
  double range(const Pose3& pose, OptionalJacobian<1, dimension> Dcamera =
      boost::none, OptionalJacobian<1, 6> Dpose = boost::none) const {
    Matrix16 Dpose_;
    double result = this->pose().range(pose, Dcamera ? &Dpose_ : 0, Dpose);
    if (Dcamera)
      *Dcamera << Dpose_, Eigen::Matrix<double, 1, DimK>::Zero();
    return result;
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @return range (double)
   */
  template<class CalibrationB>
  double range(const PinholeCamera<CalibrationB>& camera,
      OptionalJacobian<1, dimension> Dcamera = boost::none,
      boost::optional<Matrix&> Dother = boost::none) const {
    Matrix16 Dcamera_, Dother_;
    double result = this->pose().range(camera.pose(), Dcamera ? &Dcamera_ : 0,
        Dother ? &Dother_ : 0);
    if (Dcamera) {
      Dcamera->resize(1, 6 + DimK);
      *Dcamera << Dcamera_, Eigen::Matrix<double, 1, DimK>::Zero();
    }
    if (Dother) {
      Dother->resize(1, 6 + CalibrationB::dimension);
      Dother->setZero();
      Dother->block(0, 0, 1, 6) = Dother_;
    }
    return result;
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @return range (double)
   */
  double range(const CalibratedCamera& camera,
      OptionalJacobian<1, dimension> Dcamera = boost::none,
      OptionalJacobian<1, 6> Dother = boost::none) const {
    return range(camera.pose(), Dcamera, Dother);
  }

private:

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("PinholeBaseK",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(K_);
  }

};

template<typename Calibration>
struct traits<PinholeCamera<Calibration> > : public internal::Manifold<
    PinholeCamera<Calibration> > {
};

template<typename Calibration>
struct traits<const PinholeCamera<Calibration> > : public internal::Manifold<
    PinholeCamera<Calibration> > {
};

} // \ gtsam
