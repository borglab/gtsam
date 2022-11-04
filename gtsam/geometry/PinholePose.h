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
 * @ingroup geometry
 * \nosubgrouping
 */
template<typename CALIBRATION>
class PinholeBaseK: public PinholeBase {

private:

  GTSAM_CONCEPT_MANIFOLD_TYPE(CALIBRATION)

  // Get dimensions of calibration type at compile time
  static const int DimK = FixedDimension<CALIBRATION>::value;

public:

  typedef CALIBRATION CalibrationType;

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
  virtual const CALIBRATION& calibration() const = 0;

  /// @}
  /// @name Transformations and measurement functions
  /// @{

  /// Project a point into the image and check depth
  std::pair<Point2, bool> projectSafe(const Point3& pw) const {
    std::pair<Point2, bool> pn = PinholeBase::projectSafe(pw);
    pn.first = calibration().uncalibrate(pn.first);
    return pn;
  }


  /** Templated projection of a point (possibly at infinity) from world coordinate to the image
   *  @param pw is a 3D point or a Unit3 (point at infinity) in world coordinates
   *  @param Dpose is the Jacobian w.r.t. pose3
   *  @param Dpoint is the Jacobian w.r.t. point3
   *  @param Dcal is the Jacobian w.r.t. calibration
   */
  template <class POINT>
  Point2 _project(const POINT& pw, OptionalJacobian<2, 6> Dpose,
      OptionalJacobian<2, FixedDimension<POINT>::value> Dpoint,
      OptionalJacobian<2, DimK> Dcal) const {

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

  /// project a 3D point from world coordinates into the image
  Point2 project(const Point3& pw, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none,
      OptionalJacobian<2, DimK> Dcal = boost::none) const {
    return _project(pw, Dpose, Dpoint, Dcal);
  }

  /// project a 3D point from world coordinates into the image
  Point2 reprojectionError(const Point3& pw, const Point2& measured, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none,
      OptionalJacobian<2, DimK> Dcal = boost::none) const {
    return Point2(_project(pw, Dpose, Dpoint, Dcal) - measured);
  }

  /// project a point at infinity from world coordinates into the image
  Point2 project(const Unit3& pw, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 2> Dpoint = boost::none,
      OptionalJacobian<2, DimK> Dcal = boost::none) const {
    return _project(pw, Dpose, Dpoint, Dcal);
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  Point3 backproject(const Point2& p, double depth,
                     OptionalJacobian<3, 6> Dresult_dpose = boost::none,
                     OptionalJacobian<3, 2> Dresult_dp = boost::none,
                     OptionalJacobian<3, 1> Dresult_ddepth = boost::none,
                     OptionalJacobian<3, DimK> Dresult_dcal = boost::none) const {
    typedef Eigen::Matrix<double, 2, DimK> Matrix2K;
    Matrix2K Dpn_dcal;
    Matrix22 Dpn_dp;
    const Point2 pn = calibration().calibrate(p, Dresult_dcal ? &Dpn_dcal : 0,
                                                 Dresult_dp ? &Dpn_dp : 0);
    Matrix32 Dpoint_dpn;
    Matrix31 Dpoint_ddepth;
    const Point3 point = BackprojectFromCamera(pn, depth,
                                               (Dresult_dp || Dresult_dcal) ? &Dpoint_dpn : 0,
                                               Dresult_ddepth ? &Dpoint_ddepth : 0);
    Matrix33 Dresult_dpoint;
    const Point3 result = pose().transformFrom(point, Dresult_dpose,
                                                    (Dresult_ddepth ||
                                                     Dresult_dp     ||
                                                     Dresult_dcal) ? &Dresult_dpoint : 0);
    if (Dresult_dcal)
       *Dresult_dcal = Dresult_dpoint * Dpoint_dpn * Dpn_dcal;  // (3x3)*(3x2)*(2xDimK)
    if (Dresult_dp)
       *Dresult_dp =   Dresult_dpoint * Dpoint_dpn * Dpn_dp;    // (3x3)*(3x2)*(2x2)
    if (Dresult_ddepth)
       *Dresult_ddepth = Dresult_dpoint * Dpoint_ddepth;        // (3x3)*(3x1)

    return result;
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at infinity
  Unit3 backprojectPointAtInfinity(const Point2& p) const {
    const Point2 pn = calibration().calibrate(p);
    const Unit3 pc(pn.x(), pn.y(), 1.0); //by convention the last element is 1
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
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar
    & boost::serialization::make_nvp("PinholeBase",
        boost::serialization::base_object<PinholeBase>(*this));
  }

public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};
// end of class PinholeBaseK

/**
 * A pinhole camera class that has a Pose3 and a *fixed* Calibration.
 * Instead of using this class, one might consider calibrating the measurements
 * and using CalibratedCamera, which would then be faster.
 * @ingroup geometry
 * \nosubgrouping
 */
template<typename CALIBRATION>
class PinholePose: public PinholeBaseK<CALIBRATION> {

private:

  typedef PinholeBaseK<CALIBRATION> Base; ///< base class has 3D pose as private member
  boost::shared_ptr<CALIBRATION> K_; ///< shared pointer to fixed calibration

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
      Base(pose), K_(new CALIBRATION()) {
  }

  /** constructor with pose and calibration */
  PinholePose(const Pose3& pose, const boost::shared_ptr<CALIBRATION>& K) :
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
  static PinholePose Level(const boost::shared_ptr<CALIBRATION>& K,
      const Pose2& pose2, double height) {
    return PinholePose(Base::LevelPose(pose2, height), K);
  }

  /// PinholePose::level with default calibration
  static PinholePose Level(const Pose2& pose2, double height) {
    return PinholePose::Level(boost::make_shared<CALIBRATION>(), pose2, height);
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
      const Point3& upVector, const boost::shared_ptr<CALIBRATION>& K =
          boost::make_shared<CALIBRATION>()) {
    return PinholePose(Base::LookatPose(eye, target, upVector), K);
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// Init from 6D vector
  explicit PinholePose(const Vector &v) :
      Base(v), K_(new CALIBRATION()) {
  }

  /// Init from Vector and calibration
  PinholePose(const Vector &v, const Vector &K) :
      Base(v), K_(new CALIBRATION(K)) {
  }

  // Init from Pose3 and calibration
  PinholePose(const Pose3 &pose, const Vector &K) :
      Base(pose), K_(new CALIBRATION(K)) {
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const Base &camera, double tol = 1e-9) const {
    const PinholePose* e = dynamic_cast<const PinholePose*>(&camera);
    return Base::equals(camera, tol) && K_->equals(e->calibration(), tol);
  }

  /// stream operator
  friend std::ostream& operator<<(std::ostream &os, const PinholePose& camera) {
    os << "{R: " << camera.pose().rotation().rpy().transpose();
    os << ", t: " << camera.pose().translation().transpose();
    if (!camera.K_) os << ", K: none";
    else     os << ", K: " << *camera.K_;
    os << "}";
    return os;
  }

  /// print
  void print(const std::string& s = "PinholePose") const override {
    Base::print(s);
    if (!K_)
      std::cout << "s No calibration given" << std::endl;
    else
      K_->print(s + ".calibration");
  }

  /// @}
  /// @name Standard Interface
  /// @{

  ~PinholePose() override {
  }

  /// return shared pointer to calibration
  const boost::shared_ptr<CALIBRATION>& sharedCalibration() const {
    return K_;
  }

  /// return calibration
  const CALIBRATION& calibration() const override {
    return *K_;
  }

  /** project a point from world coordinate to the image, 2 derivatives only
   *  @param pw is a point in world coordinates
   *  @param Dpose is the Jacobian w.r.t. the whole camera (really only the pose)
   *  @param Dpoint is the Jacobian w.r.t. point3
   */
  Point2 project2(const Point3& pw, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 3> Dpoint = boost::none) const {
    return Base::project(pw, Dpose, Dpoint);
  }

  /// project2 version for point at infinity
  Point2 project2(const Unit3& pw, OptionalJacobian<2, 6> Dpose = boost::none,
      OptionalJacobian<2, 2> Dpoint = boost::none) const {
    return Base::project(pw, Dpose, Dpoint);
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
  static PinholePose Identity() {
    return PinholePose(); // assumes that the default constructor is valid
  }

  /// for Linear Triangulation
  Matrix34 cameraProjectionMatrix() const {
    Matrix34 P = Matrix34(PinholeBase::pose().inverse().matrix().block(0, 0, 3, 4));
    return K_->K() * P;
  }

  /// for Nonlinear Triangulation
  Vector defaultErrorWhenTriangulatingBehindCamera() const {
    return Eigen::Matrix<double,traits<Point2>::dimension,1>::Constant(2.0 * K_->fx());;
  }
  /// @}

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
// end of class PinholePose

template<typename CALIBRATION>
struct traits<PinholePose<CALIBRATION> > : public internal::Manifold<
    PinholePose<CALIBRATION> > {
};

template<typename CALIBRATION>
struct traits<const PinholePose<CALIBRATION> > : public internal::Manifold<
    PinholePose<CALIBRATION> > {
};

} // \ gtsam
