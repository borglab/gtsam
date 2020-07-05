/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file triangulation.h
 * @brief Functions for triangulation
 * @date July 31, 2013
 * @author Chris Beall
 */

#pragma once

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/TriangulationFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {

/// Exception thrown by triangulateDLT when SVD returns rank < 3
class GTSAM_EXPORT TriangulationUnderconstrainedException: public std::runtime_error {
public:
  TriangulationUnderconstrainedException() :
      std::runtime_error("Triangulation Underconstrained Exception.") {
  }
};

/// Exception thrown by triangulateDLT when landmark is behind one or more of the cameras
class GTSAM_EXPORT TriangulationCheiralityException: public std::runtime_error {
public:
  TriangulationCheiralityException() :
      std::runtime_error(
          "Triangulation Cheirality Exception: The resulting landmark is behind one or more cameras.") {
  }
};

/**
 * DLT triangulation: See Hartley and Zisserman, 2nd Ed., page 312
 * @param projection_matrices Projection matrices (K*P^-1)
 * @param measurements 2D measurements
 * @param rank_tol SVD rank tolerance
 * @return Triangulated point, in homogeneous coordinates
 */
GTSAM_EXPORT Vector4 triangulateHomogeneousDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements, double rank_tol = 1e-9);

/**
 * DLT triangulation: See Hartley and Zisserman, 2nd Ed., page 312
 * @param projection_matrices Projection matrices (K*P^-1)
 * @param measurements 2D measurements
 * @param rank_tol SVD rank tolerance
 * @return Triangulated Point3
 */
GTSAM_EXPORT Point3 triangulateDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements,
    double rank_tol = 1e-9);

/**
 * Create a factor graph with projection factors from poses and one calibration
 * @param poses Camera poses
 * @param sharedCal shared pointer to single calibration object (monocular only!)
 * @param measurements 2D measurements
 * @param landmarkKey to refer to landmark
 * @param initialEstimate
 * @return graph and initial values
 */
template<class CALIBRATION>
std::pair<NonlinearFactorGraph, Values> triangulationGraph(
    const std::vector<Pose3>& poses, boost::shared_ptr<CALIBRATION> sharedCal,
    const Point2Vector& measurements, Key landmarkKey,
    const Point3& initialEstimate) {
  Values values;
  values.insert(landmarkKey, initialEstimate); // Initial landmark value
  NonlinearFactorGraph graph;
  static SharedNoiseModel unit2(noiseModel::Unit::Create(2));
  static SharedNoiseModel prior_model(noiseModel::Isotropic::Sigma(6, 1e-6));
  for (size_t i = 0; i < measurements.size(); i++) {
    const Pose3& pose_i = poses[i];
    typedef PinholePose<CALIBRATION> Camera;
    Camera camera_i(pose_i, sharedCal);
    graph.emplace_shared<TriangulationFactor<Camera> > //
        (camera_i, measurements[i], unit2, landmarkKey);
  }
  return std::make_pair(graph, values);
}

/**
 * Create a factor graph with projection factors from pinhole cameras
 * (each camera has a pose and calibration)
 * @param cameras pinhole cameras (monocular or stereo)
 * @param measurements 2D measurements
 * @param landmarkKey to refer to landmark
 * @param initialEstimate
 * @return graph and initial values
 */
template<class CAMERA>
std::pair<NonlinearFactorGraph, Values> triangulationGraph(
    const CameraSet<CAMERA>& cameras,
    const typename CAMERA::MeasurementVector& measurements, Key landmarkKey,
    const Point3& initialEstimate) {
  Values values;
  values.insert(landmarkKey, initialEstimate); // Initial landmark value
  NonlinearFactorGraph graph;
  static SharedNoiseModel unit(noiseModel::Unit::Create(
      traits<typename CAMERA::Measurement>::dimension));
  for (size_t i = 0; i < measurements.size(); i++) {
    const CAMERA& camera_i = cameras[i];
    graph.emplace_shared<TriangulationFactor<CAMERA> > //
        (camera_i, measurements[i], unit, landmarkKey);
  }
  return std::make_pair(graph, values);
}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
/// DEPRECATED: PinholeCamera specific version
template<class CALIBRATION>
Point3 triangulateNonlinear(
    const CameraSet<PinholeCamera<CALIBRATION> >& cameras,
    const Point2Vector& measurements, const Point3& initialEstimate) {
  return triangulateNonlinear<PinholeCamera<CALIBRATION> > //
  (cameras, measurements, initialEstimate);
}

/// DEPRECATED: PinholeCamera specific version
template<class CALIBRATION>
std::pair<NonlinearFactorGraph, Values> triangulationGraph(
    const CameraSet<PinholeCamera<CALIBRATION> >& cameras,
    const Point2Vector& measurements, Key landmarkKey,
    const Point3& initialEstimate) {
  return triangulationGraph<PinholeCamera<CALIBRATION> > //
  (cameras, measurements, landmarkKey, initialEstimate);
}
#endif

/**
 * Optimize for triangulation
 * @param graph nonlinear factors for projection
 * @param values initial values
 * @param landmarkKey to refer to landmark
 * @return refined Point3
 */
GTSAM_EXPORT Point3 optimize(const NonlinearFactorGraph& graph,
    const Values& values, Key landmarkKey);

/**
 * Given an initial estimate , refine a point using measurements in several cameras
 * @param poses Camera poses
 * @param sharedCal shared pointer to single calibration object
 * @param measurements 2D measurements
 * @param initialEstimate
 * @return refined Point3
 */
template<class CALIBRATION>
Point3 triangulateNonlinear(const std::vector<Pose3>& poses,
    boost::shared_ptr<CALIBRATION> sharedCal,
    const Point2Vector& measurements, const Point3& initialEstimate) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraph<CALIBRATION> //
      (poses, sharedCal, measurements, Symbol('p', 0), initialEstimate);

  return optimize(graph, values, Symbol('p', 0));
}

/**
 * Given an initial estimate , refine a point using measurements in several cameras
 * @param cameras pinhole cameras (monocular or stereo)
 * @param measurements 2D measurements
 * @param initialEstimate
 * @return refined Point3
 */
template<class CAMERA>
Point3 triangulateNonlinear(
    const CameraSet<CAMERA>& cameras,
    const typename CAMERA::MeasurementVector& measurements, const Point3& initialEstimate) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraph<CAMERA> //
      (cameras, measurements, Symbol('p', 0), initialEstimate);

  return optimize(graph, values, Symbol('p', 0));
}

/**
 * Create a 3*4 camera projection matrix from calibration and pose.
 * Functor for partial application on calibration
 * @param pose The camera pose
 * @param cal  The calibration
 * @return Returns a Matrix34
 */
template<class CALIBRATION>
struct CameraProjectionMatrix {
  CameraProjectionMatrix(const CALIBRATION& calibration) :
      K_(calibration.K()) {
  }
  Matrix34 operator()(const Pose3& pose) const {
    return K_ * (pose.inverse().matrix()).block<3, 4>(0, 0);
  }
private:
  const Matrix3 K_;
public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. The function checks that the
 * resulting point lies in front of all cameras, but has no other checks
 * to verify the quality of the triangulation.
 * @param poses A vector of camera poses
 * @param sharedCal shared pointer to single calibration object
 * @param measurements A vector of camera measurements
 * @param rank_tol rank tolerance, default 1e-9
 * @param optimize Flag to turn on nonlinear refinement of triangulation
 * @return Returns a Point3
 */
template<class CALIBRATION>
Point3 triangulatePoint3(const std::vector<Pose3>& poses,
    boost::shared_ptr<CALIBRATION> sharedCal,
    const Point2Vector& measurements, double rank_tol = 1e-9,
    bool optimize = false) {

  assert(poses.size() == measurements.size());
  if (poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  // construct projection matrices from poses & calibration
  std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>> projection_matrices;
  CameraProjectionMatrix<CALIBRATION> createP(*sharedCal); // partially apply
  for(const Pose3& pose: poses)
    projection_matrices.push_back(createP(pose));

  // Triangulate linearly
  Point3 point = triangulateDLT(projection_matrices, measurements, rank_tol);

  // Then refine using non-linear optimization
  if (optimize)
    point = triangulateNonlinear<CALIBRATION> //
        (poses, sharedCal, measurements, point);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies in front of all cameras
  for(const Pose3& pose: poses) {
    const Point3& p_local = pose.transformTo(point);
    if (p_local.z() <= 0)
      throw(TriangulationCheiralityException());
  }
#endif

  return point;
}

/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. This function is similar to the one
 * above, except that each camera has its own calibration. The function
 * checks that the resulting point lies in front of all cameras, but has
 * no other checks to verify the quality of the triangulation.
 * @param cameras pinhole cameras
 * @param measurements A vector of camera measurements
 * @param rank_tol rank tolerance, default 1e-9
 * @param optimize Flag to turn on nonlinear refinement of triangulation
 * @return Returns a Point3
 */
template<class CAMERA>
Point3 triangulatePoint3(
    const CameraSet<CAMERA>& cameras,
    const typename CAMERA::MeasurementVector& measurements, double rank_tol = 1e-9,
    bool optimize = false) {

  size_t m = cameras.size();
  assert(measurements.size() == m);

  if (m < 2)
    throw(TriangulationUnderconstrainedException());

  // construct projection matrices from poses & calibration
  std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>> projection_matrices;
  for(const CAMERA& camera: cameras)
    projection_matrices.push_back(
        CameraProjectionMatrix<typename CAMERA::CalibrationType>(camera.calibration())(
            camera.pose()));
  Point3 point = triangulateDLT(projection_matrices, measurements, rank_tol);

  // The n refine using non-linear optimization
  if (optimize)
    point = triangulateNonlinear<CAMERA>(cameras, measurements, point);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies in front of all cameras
  for(const CAMERA& camera: cameras) {
    const Point3& p_local = camera.pose().transformTo(point);
    if (p_local.z() <= 0)
      throw(TriangulationCheiralityException());
  }
#endif

  return point;
}

/// Pinhole-specific version
template<class CALIBRATION>
Point3 triangulatePoint3(
    const CameraSet<PinholeCamera<CALIBRATION> >& cameras,
    const Point2Vector& measurements, double rank_tol = 1e-9,
    bool optimize = false) {
  return triangulatePoint3<PinholeCamera<CALIBRATION> > //
  (cameras, measurements, rank_tol, optimize);
}

struct GTSAM_EXPORT TriangulationParameters {

  double rankTolerance; ///< threshold to decide whether triangulation is result.degenerate
  ///< (the rank is the number of singular values of the triangulation matrix which are larger than rankTolerance)
  bool enableEPI; ///< if set to true, will refine triangulation using LM

  /**
   * if the landmark is triangulated at distance larger than this,
   * result is flagged as degenerate.
   */
  double landmarkDistanceThreshold; //

  /**
   * If this is nonnegative the we will check if the average reprojection error
   * is smaller than this threshold after triangulation, otherwise result is
   * flagged as degenerate.
   */
  double dynamicOutlierRejectionThreshold;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param enableEPI if true refine triangulation with embedded LM iterations
   * @param landmarkDistanceThreshold flag as degenerate if point further than this
   * @param dynamicOutlierRejectionThreshold or if average error larger than this
   *
   */
  TriangulationParameters(const double _rankTolerance = 1.0,
      const bool _enableEPI = false, double _landmarkDistanceThreshold = -1,
      double _dynamicOutlierRejectionThreshold = -1) :
      rankTolerance(_rankTolerance), enableEPI(_enableEPI), //
      landmarkDistanceThreshold(_landmarkDistanceThreshold), //
      dynamicOutlierRejectionThreshold(_dynamicOutlierRejectionThreshold) {
  }

  // stream to output
  friend std::ostream &operator<<(std::ostream &os,
      const TriangulationParameters& p) {
    os << "rankTolerance = " << p.rankTolerance << std::endl;
    os << "enableEPI = " << p.enableEPI << std::endl;
    os << "landmarkDistanceThreshold = " << p.landmarkDistanceThreshold
        << std::endl;
    os << "dynamicOutlierRejectionThreshold = "
        << p.dynamicOutlierRejectionThreshold << std::endl;
    return os;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(rankTolerance);
    ar & BOOST_SERIALIZATION_NVP(enableEPI);
    ar & BOOST_SERIALIZATION_NVP(landmarkDistanceThreshold);
    ar & BOOST_SERIALIZATION_NVP(dynamicOutlierRejectionThreshold);
  }
};

/**
 * TriangulationResult is an optional point, along with the reasons why it is invalid.
 */
class TriangulationResult: public boost::optional<Point3> {
  enum Status {
    VALID, DEGENERATE, BEHIND_CAMERA, OUTLIER, FAR_POINT
  };
  Status status_;
  TriangulationResult(Status s) :
      status_(s) {
  }
public:

  /**
   * Default constructor, only for serialization
   */
  TriangulationResult() {}

  /**
   * Constructor
   */
  TriangulationResult(const Point3& p) :
      status_(VALID) {
    reset(p);
  }
  static TriangulationResult Degenerate() {
    return TriangulationResult(DEGENERATE);
  }
  static TriangulationResult Outlier() {
    return TriangulationResult(OUTLIER);
  }
  static TriangulationResult FarPoint() {
    return TriangulationResult(FAR_POINT);
  }
  static TriangulationResult BehindCamera() {
    return TriangulationResult(BEHIND_CAMERA);
  }
  bool valid() const {
    return status_ == VALID;
  }
  bool degenerate() const {
    return status_ == DEGENERATE;
  }
  bool outlier() const {
    return status_ == OUTLIER;
  }
  bool farPoint() const {
    return status_ == FAR_POINT;
  }
  bool behindCamera() const {
    return status_ == BEHIND_CAMERA;
  }
  // stream to output
  friend std::ostream &operator<<(std::ostream &os,
      const TriangulationResult& result) {
    if (result)
      os << "point = " << *result << std::endl;
    else
      os << "no point, status = " << result.status_ << std::endl;
    return os;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int version) {
    ar & BOOST_SERIALIZATION_NVP(status_);
  }
};

/// triangulateSafe: extensive checking of the outcome
template<class CAMERA>
TriangulationResult triangulateSafe(const CameraSet<CAMERA>& cameras,
    const typename CAMERA::MeasurementVector& measured,
    const TriangulationParameters& params) {

  size_t m = cameras.size();

  // if we have a single pose the corresponding factor is uninformative
  if (m < 2)
    return TriangulationResult::Degenerate();
  else
    // We triangulate the 3D position of the landmark
    try {
      Point3 point = triangulatePoint3<CAMERA>(cameras, measured,
          params.rankTolerance, params.enableEPI);

      // Check landmark distance and re-projection errors to avoid outliers
      size_t i = 0;
      double maxReprojError = 0.0;
      for(const CAMERA& camera: cameras) {
        const Pose3& pose = camera.pose();
        if (params.landmarkDistanceThreshold > 0
            && distance3(pose.translation(), point)
                > params.landmarkDistanceThreshold)
          return TriangulationResult::FarPoint();
#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
        // verify that the triangulated point lies in front of all cameras
        // Only needed if this was not yet handled by exception
        const Point3& p_local = pose.transformTo(point);
        if (p_local.z() <= 0)
          return TriangulationResult::BehindCamera();
#endif
        // Check reprojection error
        if (params.dynamicOutlierRejectionThreshold > 0) {
          const Point2& zi = measured.at(i);
          Point2 reprojectionError(camera.project(point) - zi);
          maxReprojError = std::max(maxReprojError, reprojectionError.norm());
        }
        i += 1;
      }
      // Flag as degenerate if average reprojection error is too large
      if (params.dynamicOutlierRejectionThreshold > 0
          && maxReprojError > params.dynamicOutlierRejectionThreshold)
        return TriangulationResult::Outlier();

      // all good!
      return TriangulationResult(point);
    } catch (TriangulationUnderconstrainedException&) {
      // This exception is thrown if
      // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
      // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel cameras (or motion towards the landmark)
      return TriangulationResult::Degenerate();
    } catch (TriangulationCheiralityException&) {
      // point is behind one of the cameras: can be the case of close-to-parallel cameras or may depend on outliers
      return TriangulationResult::BehindCamera();
    }
}

} // \namespace gtsam

