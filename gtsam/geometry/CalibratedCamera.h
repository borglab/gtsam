/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file CalibratedCamera.h
 * @brief Calibrated camera for which only pose is unknown
 * @date Aug 17, 2009
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#define PINHOLEBASE_LINKING_FIX
#ifdef PINHOLEBASE_LINKING_FIX
#include <gtsam/geometry/Pose2.h>
#endif
namespace gtsam {

class Point2;

class GTSAM_EXPORT CheiralityException: public ThreadsafeException<
    CheiralityException> {
public:
  CheiralityException() :
      ThreadsafeException<CheiralityException>("Cheirality Exception") {
  }
};

/**
 * A pinhole camera class that has a Pose3, functions as base class for all pinhole cameras
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT PinholeBase {

private:

  Pose3 pose_; ///< 3D pose of camera

#ifndef PINHOLEBASE_LINKING_FIX

protected:

  /// @name Derivatives
  /// @{

  /**
   * Calculate Jacobian with respect to pose
   * @param pn projection in normalized coordinates
   * @param d disparity (inverse depth)
   */
  static Matrix26 Dpose(const Point2& pn, double d);

  /**
   * Calculate Jacobian with respect to point
   * @param pn projection in normalized coordinates
   * @param d disparity (inverse depth)
   * @param Rt transposed rotation matrix
   */
  static Matrix23 Dpoint(const Point2& pn, double d, const Matrix3& Rt);

  /// @}

public:

  /// @name Static functions
  /// @{

  /**
   * Create a level pose at the given 2D pose and height
   * @param K the calibration
   * @param pose2 specifies the location and viewing direction
   * (theta 0 = looking in direction of positive X axis)
   * @param height camera height
   */
  static Pose3 LevelPose(const Pose2& pose2, double height);

  /**
   * Create a camera pose at the given eye position looking at a target point in the scene
   * with the specified up direction vector.
   * @param eye specifies the camera position
   * @param target the point to look at
   * @param upVector specifies the camera up direction vector,
   *        doesn't need to be on the image plane nor orthogonal to the viewing axis
   */
  static Pose3 LookatPose(const Point3& eye, const Point3& target,
      const Point3& upVector);

  /// @}
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
  bool equals(const PinholeBase &camera, double tol = 1e-9) const;

  /// print
  void print(const std::string& s = "PinholeBase") const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// return pose, constant version
  const Pose3& pose() const {
    return pose_;
  }

  /// return pose, with derivative
  const Pose3& getPose(OptionalJacobian<6, 6> H) const;

  /// @}
  /// @name Transformations and measurement functions
  /// @{

  /**
   * Project from 3D point in camera coordinates into image
   * Does *not* throw a CheiralityException, even if pc behind image plane
   * @param pc point in camera coordinates
   */
  static Point2 project_to_camera(const Point3& pc, //
      OptionalJacobian<2, 3> Dpoint = boost::none);

  /// Project a point into the image and check depth
  std::pair<Point2, bool> projectSafe(const Point3& pw) const;

  /**
   * Project point into the image
   * Throws a CheiralityException if point behind image plane iff GTSAM_THROW_CHEIRALITY_EXCEPTION
   * @param point 3D point in world coordinates
   * @return the intrinsic coordinates of the projected point
   */
  Point2 project2(const Point3& point, OptionalJacobian<2, 6> Dpose =
      boost::none, OptionalJacobian<2, 3> Dpoint = boost::none) const;

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  static Point3 backproject_from_camera(const Point2& p, const double depth);

#else

public:

  PinholeBase() {
  }

  explicit PinholeBase(const Pose3& pose) :
      pose_(pose) {
  }

  explicit PinholeBase(const Vector &v) :
      pose_(Pose3::Expmap(v)) {
  }

  const Pose3& pose() const {
    return pose_;
  }

  /* ************************************************************************* */
  static Matrix26 Dpose(const Point2& pn, double d) {
    // optimized version of derivatives, see CalibratedCamera.nb
    const double u = pn.x(), v = pn.y();
    double uv = u * v, uu = u * u, vv = v * v;
    Matrix26 Dpn_pose;
    Dpn_pose << uv, -1 - uu, v, -d, 0, d * u, 1 + vv, -uv, -u, 0, -d, d * v;
    return Dpn_pose;
  }

  /* ************************************************************************* */
  static Matrix23 Dpoint(const Point2& pn, double d, const Matrix3& Rt) {
    // optimized version of derivatives, see CalibratedCamera.nb
    const double u = pn.x(), v = pn.y();
    Matrix23 Dpn_point;
    Dpn_point << //
        Rt(0, 0) - u * Rt(2, 0), Rt(0, 1) - u * Rt(2, 1), Rt(0, 2) - u * Rt(2, 2), //
    /**/Rt(1, 0) - v * Rt(2, 0), Rt(1, 1) - v * Rt(2, 1), Rt(1, 2) - v * Rt(2, 2);
    Dpn_point *= d;
    return Dpn_point;
  }

  /* ************************************************************************* */
  static Pose3 LevelPose(const Pose2& pose2, double height) {
    const double st = sin(pose2.theta()), ct = cos(pose2.theta());
    const Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
    const Rot3 wRc(x, y, z);
    const Point3 t(pose2.x(), pose2.y(), height);
    return Pose3(wRc, t);
  }

  /* ************************************************************************* */
  static Pose3 LookatPose(const Point3& eye, const Point3& target,
      const Point3& upVector) {
    Point3 zc = target - eye;
    zc = zc / zc.norm();
    Point3 xc = (-upVector).cross(zc); // minus upVector since yc is pointing down
    xc = xc / xc.norm();
    Point3 yc = zc.cross(xc);
    return Pose3(Rot3(xc, yc, zc), eye);
  }

  /* ************************************************************************* */
  bool equals(const PinholeBase &camera, double tol=1e-9) const {
    return pose_.equals(camera.pose(), tol);
  }

  /* ************************************************************************* */
  void print(const std::string& s) const {
    pose_.print(s + ".pose");
  }

  /* ************************************************************************* */
  const Pose3& getPose(OptionalJacobian<6, 6> H) const {
    if (H) {
      H->setZero();
      H->block(0, 0, 6, 6) = I_6x6;
    }
    return pose_;
  }

  /* ************************************************************************* */
  static Point2 project_to_camera(const Point3& pc,
      OptionalJacobian<2, 3> Dpoint = boost::none) {
    double d = 1.0 / pc.z();
    const double u = pc.x() * d, v = pc.y() * d;
    if (Dpoint)
      *Dpoint << d, 0.0, -u * d, 0.0, d, -v * d;
    return Point2(u, v);
  }

  /* ************************************************************************* */
  std::pair<Point2, bool> projectSafe(const Point3& pw) const {
    const Point3 pc = pose().transform_to(pw);
    const Point2 pn = project_to_camera(pc);
    return std::make_pair(pn, pc.z() > 0);
  }

  /* ************************************************************************* */
  Point2 project2(const Point3& point, OptionalJacobian<2, 6> Dpose,
      OptionalJacobian<2, 3> Dpoint) const {

    Matrix3 Rt; // calculated by transform_to if needed
    const Point3 q = pose().transform_to(point, boost::none, Dpoint ? &Rt : 0);
  #ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
    if (q.z() <= 0)
    throw CheiralityException();
  #endif
    const Point2 pn = project_to_camera(q);

    if (Dpose || Dpoint) {
      const double d = 1.0 / q.z();
      if (Dpose)
        *Dpose = PinholeBase::Dpose(pn, d);
      if (Dpoint)
        *Dpoint = PinholeBase::Dpoint(pn, d, Rt);
    }
    return pn;
  }

  /* ************************************************************************* */
  static Point3 backproject_from_camera(const Point2& p,
      const double depth) {
    return Point3(p.x() * depth, p.y() * depth, depth);
  }

#endif

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
 * A Calibrated camera class [R|-R't], calibration K=I.
 * If calibration is known, it is more computationally efficient
 * to calibrate the measurements rather than try to predict in pixels.
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT CalibratedCamera: public PinholeBase {

public:

  enum {
    dimension = 6
  };

  /// @name Standard Constructors
  /// @{

  /// default constructor
  CalibratedCamera() {
  }

  /// construct with pose
  explicit CalibratedCamera(const Pose3& pose) :
      PinholeBase(pose) {
  }

  /// @}
  /// @name Named Constructors
  /// @{

  /**
   * Create a level camera at the given 2D pose and height
   * @param pose2 specifies the location and viewing direction
   * @param height specifies the height of the camera (along the positive Z-axis)
   * (theta 0 = looking in direction of positive X axis)
   */
  static CalibratedCamera Level(const Pose2& pose2, double height);

  /**
   * Create a camera at the given eye position looking at a target point in the scene
   * with the specified up direction vector.
   * @param eye specifies the camera position
   * @param target the point to look at
   * @param upVector specifies the camera up direction vector,
   *        doesn't need to be on the image plane nor orthogonal to the viewing axis
   */
  static CalibratedCamera Lookat(const Point3& eye, const Point3& target,
      const Point3& upVector);

  /// @}
  /// @name Advanced Constructors
  /// @{

  /// construct from vector
  explicit CalibratedCamera(const Vector &v) :
      PinholeBase(v) {
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// destructor
  virtual ~CalibratedCamera() {
  }

  /// @}
  /// @name Manifold
  /// @{

  /// move a cameras pose according to d
  CalibratedCamera retract(const Vector& d) const;

  /// Return canonical coordinate
  Vector localCoordinates(const CalibratedCamera& T2) const;

  /// @deprecated
  inline size_t dim() const {
    return 6;
  }

  /// @deprecated
  inline static size_t Dim() {
    return 6;
  }

  /// @}
  /// @name Transformations and mesaurement functions
  /// @{

  /**
   * @deprecated
   * Use project2, which is more consistently named across Pinhole cameras
   */
  Point2 project(const Point3& point, OptionalJacobian<2, 6> Dcamera =
      boost::none, OptionalJacobian<2, 3> Dpoint = boost::none) const;

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  Point3 backproject(const Point2& pn, double depth) const {
    return pose().transform_from(backproject_from_camera(pn, depth));
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
   * Calculate range to another camera
   * @param camera Other camera
   * @return range (double)
   */
  double range(const CalibratedCamera& camera, //
      OptionalJacobian<1, 6> H1 = boost::none, //
      OptionalJacobian<1, 6> H2 = boost::none) const {
    return pose().range(camera.pose(), H1, H2);
  }

  /// @}

private:

  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar
        & boost::serialization::make_nvp("PinholeBase",
            boost::serialization::base_object<PinholeBase>(*this));
  }

  /// @}
};

template<>
struct traits<CalibratedCamera> : public internal::Manifold<CalibratedCamera> {
};

template<>
struct traits<const CalibratedCamera> : public internal::Manifold<
    CalibratedCamera> {
};

}

