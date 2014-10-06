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

#include <cmath>
#include <boost/optional.hpp>
#include <boost/serialization/nvp.hpp>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>

namespace gtsam {

/**
 * A pinhole camera class that has a Pose3 and a Calibration.
 * @addtogroup geometry
 * \nosubgrouping
 */
template<typename Calibration>
class PinholeCamera: public DerivedValue<PinholeCamera<Calibration> > {
private:
  Pose3 pose_;
  Calibration K_;

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor */
  PinholeCamera() {
  }

  /** constructor with pose */
  explicit PinholeCamera(const Pose3& pose) :
      pose_(pose) {
  }

  /** constructor with pose and calibration */
  PinholeCamera(const Pose3& pose, const Calibration& K) :
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
  static PinholeCamera Level(const Calibration &K, const Pose2& pose2,
      double height) {
    const double st = sin(pose2.theta()), ct = cos(pose2.theta());
    const Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
    const Rot3 wRc(x, y, z);
    const Point3 t(pose2.x(), pose2.y(), height);
    const Pose3 pose3(wRc, t);
    return PinholeCamera(pose3, K);
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
    Point3 zc = target - eye;
    zc = zc / zc.norm();
    Point3 xc = (-upVector).cross(zc); // minus upVector since yc is pointing down
    xc = xc / xc.norm();
    Point3 yc = zc.cross(xc);
    Pose3 pose3(Rot3(xc, yc, zc), eye);
    return PinholeCamera(pose3, K);
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit PinholeCamera(const Vector &v) {
    pose_ = Pose3::Expmap(v.head(Pose3::Dim()));
    if (v.size() > Pose3::Dim()) {
      K_ = Calibration(v.tail(Calibration::Dim()));
    }
  }

  PinholeCamera(const Vector &v, const Vector &K) :
      pose_(Pose3::Expmap(v)), K_(K) {
  }

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const PinholeCamera &camera, double tol = 1e-9) const {
    return pose_.equals(camera.pose(), tol)
        && K_.equals(camera.calibration(), tol);
  }

  /// print
  void print(const std::string& s = "PinholeCamera") const {
    pose_.print(s + ".pose");
    K_.print(s + ".calibration");
  }

  /// @}
  /// @name Standard Interface
  /// @{

  virtual ~PinholeCamera() {
  }

  /// return pose
  inline Pose3& pose() {
    return pose_;
  }

  /// return pose
  inline const Pose3& pose() const {
    return pose_;
  }

  /// return calibration
  inline Calibration& calibration() {
    return K_;
  }

  /// return calibration
  inline const Calibration& calibration() const {
    return K_;
  }

  /// @}
  /// @name Group ?? Frank says this might not make sense
  /// @{

  /// compose two cameras: TODO Frank says this might not make sense
  inline const PinholeCamera compose(const PinholeCamera &c,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    PinholeCamera result(pose_.compose(c.pose(), H1, H2), K_);
    if (H1) {
      H1->conservativeResize(Dim(), Dim());
      H1->topRightCorner(Pose3::Dim(), Calibration::Dim()) = zeros(Pose3::Dim(),
          Calibration::Dim());
      H1->bottomRows(Calibration::Dim()) = zeros(Calibration::Dim(), Dim());
    }
    if (H2) {
      H2->conservativeResize(Dim(), Dim());
      H2->topRightCorner(Pose3::Dim(), Calibration::Dim()) = zeros(Pose3::Dim(),
          Calibration::Dim());
      H2->bottomRows(Calibration::Dim()) = zeros(Calibration::Dim(), Dim());
    }
    return result;
  }

  /// between two cameras: TODO Frank says this might not make sense
  inline const PinholeCamera between(const PinholeCamera& c,
      boost::optional<Matrix&> H1 = boost::none, boost::optional<Matrix&> H2 =
          boost::none) const {
    PinholeCamera result(pose_.between(c.pose(), H1, H2), K_);
    if (H1) {
      H1->conservativeResize(Dim(), Dim());
      H1->topRightCorner(Pose3::Dim(), Calibration::Dim()) = zeros(Pose3::Dim(),
          Calibration::Dim());
      H1->bottomRows(Calibration::Dim()) = zeros(Calibration::Dim(), Dim());
    }
    if (H2) {
      H2->conservativeResize(Dim(), Dim());
      H2->topRightCorner(Pose3::Dim(), Calibration::Dim()) = zeros(Pose3::Dim(),
          Calibration::Dim());
      H2->bottomRows(Calibration::Dim()) = zeros(Calibration::Dim(), Dim());
    }
    return result;
  }

  /// inverse camera: TODO Frank says this might not make sense
  inline const PinholeCamera inverse(
      boost::optional<Matrix&> H1 = boost::none) const {
    PinholeCamera result(pose_.inverse(H1), K_);
    if (H1) {
      H1->conservativeResize(Dim(), Dim());
      H1->topRightCorner(Pose3::Dim(), Calibration::Dim()) = zeros(Pose3::Dim(),
          Calibration::Dim());
      H1->bottomRows(Calibration::Dim()) = zeros(Calibration::Dim(), Dim());
    }
    return result;
  }

  /// compose two cameras: TODO Frank says this might not make sense
  inline const PinholeCamera compose(const Pose3 &c) const {
    return PinholeCamera(pose_.compose(c), K_);
  }

  /// @}
  /// @name Manifold
  /// @{

  /// move a cameras according to d
  PinholeCamera retract(const Vector& d) const {
    if ((size_t) d.size() == pose_.dim())
      return PinholeCamera(pose().retract(d), calibration());
    else
      return PinholeCamera(pose().retract(d.head(pose().dim())),
          calibration().retract(d.tail(calibration().dim())));
  }

  /// return canonical coordinate
  Vector localCoordinates(const PinholeCamera& T2) const {
    Vector d(dim());
    d.head(pose().dim()) = pose().localCoordinates(T2.pose());
    d.tail(calibration().dim()) = calibration().localCoordinates(
        T2.calibration());
    return d;
  }

  /// Manifold dimension
  inline size_t dim() const {
    return pose_.dim() + K_.dim();
  }

  /// Manifold dimension
  inline static size_t Dim() {
    return Pose3::Dim() + Calibration::Dim();
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
  inline static Point2 project_to_camera(const Point3& P,
      boost::optional<Matrix&> Dpoint = boost::none) {
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
    if (P.z() <= 0)
      throw CheiralityException();
#endif
    double d = 1.0 / P.z();
    const double u = P.x() * d, v = P.y() * d;
    if (Dpoint) {
      *Dpoint = (Matrix(2, 3) << d, 0.0, -u * d, 0.0, d, -v * d);
    }
    return Point2(u, v);
  }

  /// Project a point into the image and check depth
  inline std::pair<Point2, bool> projectSafe(const Point3& pw) const {
    const Point3 pc = pose_.transform_to(pw);
    const Point2 pn = project_to_camera(pc);
    return std::make_pair(K_.uncalibrate(pn), pc.z() > 0);
  }

  /** project a point from world coordinate to the image
   *  @param pw is a point in world coordinates
   *  @param Dpose is the Jacobian w.r.t. pose3
   *  @param Dpoint is the Jacobian w.r.t. point3
   *  @param Dcal is the Jacobian w.r.t. calibration
   */
  inline Point2 project(
      const Point3& pw, //
      boost::optional<Matrix&> Dpose = boost::none,
      boost::optional<Matrix&> Dpoint = boost::none,
      boost::optional<Matrix&> Dcal = boost::none) const {

    // Transform to camera coordinates and check cheirality
    const Point3 pc = pose_.transform_to(pw);

    // Project to normalized image coordinates
    const Point2 pn = project_to_camera(pc);

    if (Dpose || Dpoint) {
      const double z = pc.z(), d = 1.0 / z;

      // uncalibration
      Matrix Dpi_pn(2, 2);
      const Point2 pi = K_.uncalibrate(pn, Dcal, Dpi_pn);

      // chain the Jacobian matrices
      if (Dpose) {
        Dpose->resize(2, 6);
        calculateDpose(pn, d, Dpi_pn, *Dpose);
      }
      if (Dpoint) {
        Dpoint->resize(2, 3);
        calculateDpoint(pn, d, pose_.rotation().matrix(), Dpi_pn, *Dpoint);
      }
      return pi;
    } else
      return K_.uncalibrate(pn, Dcal);
  }

  /** project a point at infinity from world coordinate to the image
   *  @param pw is a point in the world coordinate (it is pw = lambda*[pw_x  pw_y  pw_z] with lambda->inf)
   *  @param Dpose is the Jacobian w.r.t. pose3
   *  @param Dpoint is the Jacobian w.r.t. point3
   *  @param Dcal is the Jacobian w.r.t. calibration
   */
  inline Point2 projectPointAtInfinity(
      const Point3& pw, //
      boost::optional<Matrix&> Dpose = boost::none,
      boost::optional<Matrix&> Dpoint = boost::none,
      boost::optional<Matrix&> Dcal = boost::none) const {

    if (!Dpose && !Dpoint && !Dcal) {
      const Point3 pc = pose_.rotation().unrotate(pw); // get direction in camera frame (translation does not matter)
      const Point2 pn = project_to_camera(pc); // project the point to the camera
      return K_.uncalibrate(pn);
    }

    // world to camera coordinate
    Matrix Dpc_rot /* 3*3 */, Dpc_point /* 3*3 */;
    const Point3 pc = pose_.rotation().unrotate(pw, Dpc_rot, Dpc_point);

    Matrix Dpc_pose = Matrix::Zero(3, 6);
    Dpc_pose.block(0, 0, 3, 3) = Dpc_rot;

    // camera to normalized image coordinate
    Matrix Dpn_pc; // 2*3
    const Point2 pn = project_to_camera(pc, Dpn_pc);

    // uncalibration
    Matrix Dpi_pn; // 2*2
    const Point2 pi = K_.uncalibrate(pn, Dcal, Dpi_pn);

    // chain the Jacobian matrices
    const Matrix Dpi_pc = Dpi_pn * Dpn_pc;
    if (Dpose)
      *Dpose = Dpi_pc * Dpc_pose;
    if (Dpoint)
      *Dpoint = (Dpi_pc * Dpc_point).block(0, 0, 2, 2); // only 2dof are important for the point (direction-only)
    return pi;
  }

  /** project a point from world coordinate to the image
   *  @param pw is a point in the world coordinate
   *  @param Dcamera is the Jacobian w.r.t. [pose3 calibration]
   *  @param Dpoint is the Jacobian w.r.t. point3
   */
  inline Point2 project2(
      const Point3& pw, //
      boost::optional<Matrix&> Dcamera = boost::none,
      boost::optional<Matrix&> Dpoint = boost::none) const {

    const Point3 pc = pose_.transform_to(pw);
    const Point2 pn = project_to_camera(pc);

    if (!Dcamera && !Dpoint) {
      return K_.uncalibrate(pn);
    } else {
      const double z = pc.z(), d = 1.0 / z;

      // uncalibration
      Matrix Dcal, Dpi_pn(2, 2);
      const Point2 pi = K_.uncalibrate(pn, Dcal, Dpi_pn);

      if (Dcamera) {
        Dcamera->resize(2, this->dim());
        calculateDpose(pn, d, Dpi_pn, Dcamera->leftCols<6>());
        Dcamera->rightCols(K_.dim()) = Dcal; // Jacobian wrt calib
      }
      if (Dpoint) {
        Dpoint->resize(2, 3);
        calculateDpoint(pn, d, pose_.rotation().matrix(), Dpi_pn, *Dpoint);
      }
      return pi;
    }
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  inline Point3 backproject(const Point2& p, double depth) const {
    const Point2 pn = K_.calibrate(p);
    const Point3 pc(pn.x() * depth, pn.y() * depth, depth);
    return pose_.transform_from(pc);
  }

  /// backproject a 2-dimensional point to a 3-dimensional point at infinity
  inline Point3 backprojectPointAtInfinity(const Point2& p) const {
    const Point2 pn = K_.calibrate(p);
    const Point3 pc(pn.x(), pn.y(), 1.0); //by convention the last element is 1
    return pose_.rotation().rotate(pc);
  }

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @param Dpose the optionally computed Jacobian with respect to pose
   * @param Dpoint the optionally computed Jacobian with respect to the landmark
   * @return range (double)
   */
  double range(
      const Point3& point, //
      boost::optional<Matrix&> Dpose = boost::none,
      boost::optional<Matrix&> Dpoint = boost::none) const {
    double result = pose_.range(point, Dpose, Dpoint);
    if (Dpose) {
      // Add columns of zeros to Jacobian for calibration
      Matrix& H1r(*Dpose);
      H1r.conservativeResize(Eigen::NoChange, pose_.dim() + K_.dim());
      H1r.block(0, pose_.dim(), 1, K_.dim()) = Matrix::Zero(1, K_.dim());
    }
    return result;
  }

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @param Dpose the optionally computed Jacobian with respect to pose
   * @param Dpose2 the optionally computed Jacobian with respect to the other pose
   * @return range (double)
   */
  double range(
      const Pose3& pose, //
      boost::optional<Matrix&> Dpose = boost::none,
      boost::optional<Matrix&> Dpose2 = boost::none) const {
    double result = pose_.range(pose, Dpose, Dpose2);
    if (Dpose) {
      // Add columns of zeros to Jacobian for calibration
      Matrix& H1r(*Dpose);
      H1r.conservativeResize(Eigen::NoChange, pose_.dim() + K_.dim());
      H1r.block(0, pose_.dim(), 1, K_.dim()) = Matrix::Zero(1, K_.dim());
    }
    return result;
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @param Dpose the optionally computed Jacobian with respect to pose
   * @param Dother the optionally computed Jacobian with respect to the other camera
   * @return range (double)
   */
  template<class CalibrationB>
  double range(
      const PinholeCamera<CalibrationB>& camera, //
      boost::optional<Matrix&> Dpose = boost::none,
      boost::optional<Matrix&> Dother = boost::none) const {
    double result = pose_.range(camera.pose_, Dpose, Dother);
    if (Dpose) {
      // Add columns of zeros to Jacobian for calibration
      Matrix& H1r(*Dpose);
      H1r.conservativeResize(Eigen::NoChange, pose_.dim() + K_.dim());
      H1r.block(0, pose_.dim(), 1, K_.dim()) = Matrix::Zero(1, K_.dim());
    }
    if (Dother) {
      // Add columns of zeros to Jacobian for calibration
      Matrix& H2r(*Dother);
      H2r.conservativeResize(Eigen::NoChange,
          camera.pose().dim() + camera.calibration().dim());
      H2r.block(0, camera.pose().dim(), 1, camera.calibration().dim()) =
          Matrix::Zero(1, camera.calibration().dim());
    }
    return result;
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @param Dpose the optionally computed Jacobian with respect to pose
   * @param Dother the optionally computed Jacobian with respect to the other camera
   * @return range (double)
   */
  double range(
      const CalibratedCamera& camera, //
      boost::optional<Matrix&> Dpose = boost::none,
      boost::optional<Matrix&> Dother = boost::none) const {
    return pose_.range(camera.pose_, Dpose, Dother);
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
  static void calculateDpose(const Point2& pn, double d, const Matrix& Dpi_pn,
      Eigen::MatrixBase<Derived> const & Dpose) {
    // optimized version of derivatives, see CalibratedCamera.nb
    const double u = pn.x(), v = pn.y();
    double uv = u * v, uu = u * u, vv = v * v;
    Eigen::Matrix<double, 2, 6> Dpn_pose;
    Dpn_pose << uv, -1 - uu, v, -d, 0, d * u, 1 + vv, -uv, -u, 0, -d, d * v;
    assert(Dpose.rows()==2 && Dpose.cols()==6);
    const_cast<Eigen::MatrixBase<Derived>&>(Dpose) = //
        Dpi_pn.block<2, 2>(0, 0) * Dpn_pose;
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
  static void calculateDpoint(const Point2& pn, double d, const Matrix& R,
      const Matrix& Dpi_pn, Eigen::MatrixBase<Derived> const & Dpoint) {
    // optimized version of derivatives, see CalibratedCamera.nb
    const double u = pn.x(), v = pn.y();
    Eigen::Matrix<double, 2, 3> Dpn_point;
    Dpn_point << //
        R(0, 0) - u * R(0, 2), R(1, 0) - u * R(1, 2), R(2, 0) - u * R(2, 2), //
    /**/R(0, 1) - v * R(0, 2), R(1, 1) - v * R(1, 2), R(2, 1) - v * R(2, 2);
    Dpn_point *= d;
    assert(Dpoint.rows()==2 && Dpoint.cols()==3);
    const_cast<Eigen::MatrixBase<Derived>&>(Dpoint) = //
        Dpi_pn.block<2, 2>(0, 0) * Dpn_point;
  }

  /// @}
  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Value);
    ar & BOOST_SERIALIZATION_NVP(pose_);
    ar & BOOST_SERIALIZATION_NVP(K_);
  }
  /// @}
      }
      ;}
