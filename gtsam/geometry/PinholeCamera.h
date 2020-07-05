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
#include <gtsam/geometry/BearingRange.h>

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a Calibration.
 * Use PinholePose if you will not be optimizing for Calibration
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class PinholeCamera: public PinholeBaseK<Calibration> {

public:

  /**
   *  Some classes template on either PinholeCamera or StereoCamera,
   *  and this typedef informs those classes what "project" returns.
   */
  typedef Point2 Measurement;
  typedef Point2Vector MeasurementVector;

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

  // Create PinholeCamera, with derivatives
  static PinholeCamera Create(const Pose3& pose, const Calibration &K,
      OptionalJacobian<dimension, 6> H1 = boost::none, //
      OptionalJacobian<dimension, DimK> H2 = boost::none) {
    typedef Eigen::Matrix<double, DimK, 6> MatrixK6;
    if (H1)
      *H1 << I_6x6, MatrixK6::Zero();
    typedef Eigen::Matrix<double, 6, DimK> Matrix6K;
    typedef Eigen::Matrix<double, DimK, DimK> MatrixK;
    if (H2)
      *H2 << Matrix6K::Zero(), MatrixK::Identity();
    return PinholeCamera(pose,K);
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
      H->template block<6, 6>(0, 0) = I_6x6;
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
      return PinholeCamera(this->pose().retract(d.head<6>()),
          calibration().retract(d.tail(calibration().dim())));
  }

  /// return canonical coordinate
  VectorK6 localCoordinates(const PinholeCamera& T2) const {
    VectorK6 d;
    d.template head<6>() = this->pose().localCoordinates(T2.pose());
    d.template tail<DimK>() = calibration().localCoordinates(T2.calibration());
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

  /** Templated projection of a 3D point or a point at infinity into the image
   *  @param pw either a Point3 or a Unit3, in world coordinates
   */
  template<class POINT>
  Point2 _project2(const POINT& pw, OptionalJacobian<2, dimension> Dcamera,
      OptionalJacobian<2, FixedDimension<POINT>::value> Dpoint) const {
    // We just call 3-derivative version in Base
    Matrix26 Dpose;
    Eigen::Matrix<double, 2, DimK> Dcal;
    Point2 pi = Base::project(pw, Dcamera ? &Dpose : 0, Dpoint,
        Dcamera ? &Dcal : 0);
    if (Dcamera)
      *Dcamera << Dpose, Dcal;
    return pi;
  }

  /// project a 3D point from world coordinates into the image
  Point2 project2(const Point3& pw, OptionalJacobian<2, dimension> Dcamera =
      boost::none, OptionalJacobian<2, 3> Dpoint = boost::none) const {
    return _project2(pw, Dcamera, Dpoint);
  }

  /// project a point at infinity from world coordinates into the image
  Point2 project2(const Unit3& pw, OptionalJacobian<2, dimension> Dcamera =
      boost::none, OptionalJacobian<2, 2> Dpoint = boost::none) const {
    return _project2(pw, Dcamera, Dpoint);
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
      OptionalJacobian<1, 6 + CalibrationB::dimension> Dother = boost::none) const {
    Matrix16 Dcamera_, Dother_;
    double result = this->pose().range(camera.pose(), Dcamera ? &Dcamera_ : 0,
        Dother ? &Dother_ : 0);
    if (Dcamera) {
      *Dcamera << Dcamera_, Eigen::Matrix<double, 1, DimK>::Zero();
    }
    if (Dother) {
      Dother->setZero();
      Dother->template block<1, 6>(0, 0) = Dother_;
    }
    return result;
  }

  /**
   * Calculate range to a calibrated camera
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

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

// manifold traits

template <typename Calibration>
struct traits<PinholeCamera<Calibration> >
    : public internal::Manifold<PinholeCamera<Calibration> > {};

template <typename Calibration>
struct traits<const PinholeCamera<Calibration> >
    : public internal::Manifold<PinholeCamera<Calibration> > {};

// range traits, used in RangeFactor
template <typename Calibration, typename T>
struct Range<PinholeCamera<Calibration>, T> : HasRange<PinholeCamera<Calibration>, T, double> {};

}  // \ gtsam
