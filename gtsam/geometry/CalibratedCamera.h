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

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/concepts.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/dllexport.h>
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/nvp.hpp>
#endif

namespace gtsam {

class GTSAM_EXPORT CheiralityException: public ThreadsafeException<CheiralityException> {
public:
  CheiralityException()
    : CheiralityException(std::numeric_limits<Key>::max()) {}

  CheiralityException(Key j)
    : ThreadsafeException<CheiralityException>("CheiralityException"),
      j_(j) {}

  Key nearbyVariable() const {return j_;}

private:
  Key j_;
};

/**
 * A pinhole camera class that has a Pose3, functions as base class for all pinhole cameras
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT PinholeBase {

public:

  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;

  /**
   *  Some classes template on either PinholeCamera or StereoCamera,
   *  and this typedef informs those classes what "project" returns.
   */
  typedef Point2 Measurement;
  typedef Point2Vector MeasurementVector;

private:

  Pose3 pose_; ///< 3D pose of camera

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

  /// Default constructor
  PinholeBase() {}

  /// Constructor with pose
  explicit PinholeBase(const Pose3& pose) : pose_(pose) {}

  /// @}
  /// @name Advanced Constructors
  /// @{

  explicit PinholeBase(const Vector& v) : pose_(Pose3::Expmap(v)) {}

  /// Default destructor
  virtual ~PinholeBase() = default;

  /// @}
  /// @name Testable
  /// @{

  /// assert equality up to a tolerance
  bool equals(const PinholeBase &camera, double tol = 1e-9) const;

  /// print
  virtual void print(const std::string& s = "PinholeBase") const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// return pose, constant version
  const Pose3& pose() const {
    return pose_;
  }

  /// get rotation
  const Rot3& rotation() const {
    return pose_.rotation();
  }

  /// get translation
  const Point3& translation() const {
    return pose_.translation();
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
  static Point2 Project(const Point3& pc, //
      OptionalJacobian<2, 3> Dpoint = {});

  /**
   * Project from 3D point at infinity in camera coordinates into image
   * Does *not* throw a CheiralityException, even if pc behind image plane
   * @param pc point in camera coordinates
   */
  static Point2 Project(const Unit3& pc, //
      OptionalJacobian<2, 2> Dpoint = {});

  /// Project a point into the image and check depth
  std::pair<Point2, bool> projectSafe(const Point3& pw) const;

  /** Project point into the image
   * Throws a CheiralityException if point behind image plane iff GTSAM_THROW_CHEIRALITY_EXCEPTION
   * @param point 3D point in world coordinates
   * @return the intrinsic coordinates of the projected point
   */
  Point2 project2(const Point3& point, OptionalJacobian<2, 6> Dpose =
      {}, OptionalJacobian<2, 3> Dpoint = {}) const;

  /** Project point at infinity into the image
   * Throws a CheiralityException if point behind image plane iff GTSAM_THROW_CHEIRALITY_EXCEPTION
   * @param point 3D point in world coordinates
   * @return the intrinsic coordinates of the projected point
   */
  Point2 project2(const Unit3& point,
      OptionalJacobian<2, 6> Dpose = {},
      OptionalJacobian<2, 2> Dpoint = {}) const;

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  static Point3 BackprojectFromCamera(const Point2& p, const double depth,
                                      OptionalJacobian<3, 2> Dpoint = {},
                                      OptionalJacobian<3, 1> Ddepth = {});

  /// @}
  /// @name Advanced interface
  /// @{

  /**
   * Return the start and end indices (inclusive) of the translation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  inline static std::pair<size_t, size_t> translationInterval() {
    return std::make_pair(3, 5);
  }

  /// @}

private:

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(pose_);
  }
#endif
};
// end of class PinholeBase

/**
 * A Calibrated camera class [R|-R't], calibration K=I.
 * If calibration is known, it is more computationally efficient
 * to calibrate the measurements rather than try to predict in pixels.
 * @ingroup geometry
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

  // Create CalibratedCamera, with derivatives
  static CalibratedCamera Create(const Pose3& pose,
                                 OptionalJacobian<dimension, 6> H1 = {}) {
    if (H1)
      *H1 << I_6x6;
    return CalibratedCamera(pose);
  }

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

  /// print
  void print(const std::string& s = "CalibratedCamera") const override {
    PinholeBase::print(s);
  }

  /// @deprecated
  inline size_t dim() const {
    return dimension;
  }

  /// @deprecated
  inline static size_t Dim() {
    return dimension;
  }

  /// @}
  /// @name Transformations and measurement functions
  /// @{

  /**
   * @deprecated
   * Use project2, which is more consistently named across Pinhole cameras
   */
  Point2 project(const Point3& point, OptionalJacobian<2, 6> Dcamera =
      {}, OptionalJacobian<2, 3> Dpoint = {}) const;

  /// backproject a 2-dimensional point to a 3-dimensional point at given depth
  Point3 backproject(const Point2& pn, double depth,
                     OptionalJacobian<3, 6> Dresult_dpose = {},
                     OptionalJacobian<3, 2> Dresult_dp = {},
                     OptionalJacobian<3, 1> Dresult_ddepth = {}) const {

    Matrix32 Dpoint_dpn;
    Matrix31 Dpoint_ddepth;
    const Point3 point = BackprojectFromCamera(pn, depth,
                                                 Dresult_dp ? &Dpoint_dpn : 0,
                                                 Dresult_ddepth ? &Dpoint_ddepth : 0);

    Matrix33 Dresult_dpoint;
    const Point3 result = pose().transformFrom(point, Dresult_dpose,
                                                       (Dresult_ddepth ||
                                                        Dresult_dp) ? &Dresult_dpoint : 0);

    if (Dresult_dp)
      *Dresult_dp = Dresult_dpoint * Dpoint_dpn;
    if (Dresult_ddepth)
      *Dresult_ddepth = Dresult_dpoint * Dpoint_ddepth;

    return result;
  }

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point,
      OptionalJacobian<1, 6> Dcamera = {},
      OptionalJacobian<1, 3> Dpoint = {}) const {
    return pose().range(point, Dcamera, Dpoint);
  }

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @return range (double)
   */
  double range(const Pose3& pose, OptionalJacobian<1, 6> Dcamera = {},
      OptionalJacobian<1, 6> Dpose = {}) const {
    return this->pose().range(pose, Dcamera, Dpose);
  }

  /**
   * Calculate range to another camera
   * @param camera Other camera
   * @return range (double)
   */
  double range(const CalibratedCamera& camera, //
      OptionalJacobian<1, 6> H1 = {}, //
      OptionalJacobian<1, 6> H2 = {}) const {
    return pose().range(camera.pose(), H1, H2);
  }

  /// @}

private:

  /// @name Advanced Interface
  /// @{

  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("PinholeBase",
            boost::serialization::base_object<PinholeBase>(*this));
  }

  /// @}
};

// manifold traits
template <>
struct traits<CalibratedCamera> : public internal::Manifold<CalibratedCamera> {};

template <>
struct traits<const CalibratedCamera> : public internal::Manifold<CalibratedCamera> {};

// range traits, used in RangeFactor
template <typename T>
struct Range<CalibratedCamera, T> : HasRange<CalibratedCamera, T, double> {};

}  // namespace gtsam
