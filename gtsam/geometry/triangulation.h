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
#include <gtsam/slam/TriangulationFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

namespace gtsam {

/// Exception thrown by triangulateDLT when SVD returns rank < 3
class TriangulationUnderconstrainedException: public std::runtime_error {
public:
  TriangulationUnderconstrainedException() :
      std::runtime_error("Triangulation Underconstrained Exception.") {
  }
};

/// Exception thrown by triangulateDLT when landmark is behind one or more of the cameras
class TriangulationCheiralityException: public std::runtime_error {
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
 * @return Triangulated Point3
 */
GTSAM_EXPORT Point3 triangulateDLT(
    const std::vector<Matrix34>& projection_matrices,
    const std::vector<Point2>& measurements, double rank_tol);

/**
 * Create a factor graph with projection factors from poses and one calibration
 * @param poses Camera poses
 * @param sharedCal shared pointer to single calibration object
 * @param measurements 2D measurements
 * @param landmarkKey to refer to landmark
 * @param initialEstimate
 * @return graph and initial values
 */
template<class CALIBRATION>
std::pair<NonlinearFactorGraph, Values> triangulationGraph(
    const std::vector<Pose3>& poses, boost::shared_ptr<CALIBRATION> sharedCal,
    const std::vector<Point2>& measurements, Key landmarkKey,
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
    graph.push_back(TriangulationFactor<Camera> //
        (camera_i, measurements[i], unit2, landmarkKey));
  }
  return std::make_pair(graph, values);
}

/**
 * Create a factor graph with projection factors from pinhole cameras
 * (each camera has a pose and calibration)
 * @param cameras pinhole cameras
 * @param measurements 2D measurements
 * @param landmarkKey to refer to landmark
 * @param initialEstimate
 * @return graph and initial values
 */
template<class CAMERA>
std::pair<NonlinearFactorGraph, Values> triangulationGraph(
    const std::vector<CAMERA>& cameras, const std::vector<Point2>& measurements,
    Key landmarkKey, const Point3& initialEstimate) {
  Values values;
  values.insert(landmarkKey, initialEstimate); // Initial landmark value
  NonlinearFactorGraph graph;
  static SharedNoiseModel unit2(noiseModel::Unit::Create(2));
  static SharedNoiseModel prior_model(noiseModel::Isotropic::Sigma(6, 1e-6));
  for (size_t i = 0; i < measurements.size(); i++) {
    const CAMERA& camera_i = cameras[i];
    graph.push_back(TriangulationFactor<CAMERA> //
        (camera_i, measurements[i], unit2, landmarkKey));
  }
  return std::make_pair(graph, values);
}

/// PinholeCamera specific version
template<class CALIBRATION>
std::pair<NonlinearFactorGraph, Values> triangulationGraph(
    const std::vector<PinholeCamera<CALIBRATION> >& cameras,
    const std::vector<Point2>& measurements, Key landmarkKey,
    const Point3& initialEstimate) {
  return triangulationGraph<PinholeCamera<CALIBRATION> > //
  (cameras, measurements, landmarkKey, initialEstimate);
}

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
    const std::vector<Point2>& measurements, const Point3& initialEstimate) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraph<CALIBRATION> //
      (poses, sharedCal, measurements, Symbol('p', 0), initialEstimate);

  return optimize(graph, values, Symbol('p', 0));
}

/**
 * Given an initial estimate , refine a point using measurements in several cameras
 * @param cameras pinhole cameras
 * @param measurements 2D measurements
 * @param initialEstimate
 * @return refined Point3
 */
template<class CAMERA>
Point3 triangulateNonlinear(const std::vector<CAMERA>& cameras,
    const std::vector<Point2>& measurements, const Point3& initialEstimate) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraph<CAMERA> //
      (cameras, measurements, Symbol('p', 0), initialEstimate);

  return optimize(graph, values, Symbol('p', 0));
}

/// PinholeCamera specific version
template<class CALIBRATION>
Point3 triangulateNonlinear(
    const std::vector<PinholeCamera<CALIBRATION> >& cameras,
    const std::vector<Point2>& measurements, const Point3& initialEstimate) {
  return triangulateNonlinear<PinholeCamera<CALIBRATION> > //
  (cameras, measurements, initialEstimate);
}

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
    const std::vector<Point2>& measurements, double rank_tol = 1e-9,
    bool optimize = false) {

  assert(poses.size() == measurements.size());
  if (poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  // construct projection matrices from poses & calibration
  std::vector<Matrix34> projection_matrices;
  BOOST_FOREACH(const Pose3& pose, poses) {
    projection_matrices.push_back(
        sharedCal->K() * (pose.inverse().matrix()).block<3, 4>(0, 0));
  }
  // Triangulate linearly
  Point3 point = triangulateDLT(projection_matrices, measurements, rank_tol);

  // The n refine using non-linear optimization
  if (optimize)
    point = triangulateNonlinear<CALIBRATION> //
        (poses, sharedCal, measurements, point);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const Pose3& pose, poses) {
    const Point3& p_local = pose.transform_to(point);
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
Point3 triangulatePoint3(const std::vector<CAMERA>& cameras,
    const std::vector<Point2>& measurements, double rank_tol = 1e-9,
    bool optimize = false) {

  size_t m = cameras.size();
  assert(measurements.size()==m);

  if (m < 2)
    throw(TriangulationUnderconstrainedException());

  // construct projection matrices from poses & calibration
  std::vector<Matrix34> projection_matrices;
  BOOST_FOREACH(const CAMERA& camera, cameras) {
    Matrix P_ = (camera.pose().inverse().matrix());
    projection_matrices.push_back(
        camera.calibration().K() * (P_.block<3, 4>(0, 0)));
  }
  Point3 point = triangulateDLT(projection_matrices, measurements, rank_tol);

  // The n refine using non-linear optimization
  if (optimize)
    point = triangulateNonlinear<CAMERA>(cameras, measurements, point);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const CAMERA& camera, cameras) {
    const Point3& p_local = camera.pose().transform_to(point);
    if (p_local.z() <= 0)
    throw(TriangulationCheiralityException());
  }
#endif

  return point;
}

/// Pinhole-specific version
template<class CALIBRATION>
Point3 triangulatePoint3(
    const std::vector<PinholeCamera<CALIBRATION> >& cameras,
    const std::vector<Point2>& measurements, double rank_tol = 1e-9,
    bool optimize = false) {
  return triangulatePoint3<PinholeCamera<CALIBRATION> > //
  (cameras, measurements, rank_tol, optimize);
}

struct TriangulationParameters {

  const double rankTolerance; ///< threshold to decide whether triangulation is result.degenerate
  const bool enableEPI; ///< if set to true, will refine triangulation using LM

  /**
   * if the landmark is triangulated at distance larger than this,
   * result is flagged as degenerate.
   */
  const double landmarkDistanceThreshold; //

  /**
   * If this is nonnegative the we will check if the average reprojection error
   * is smaller than this threshold after triangulation, otherwise result is
   * flagged as degenerate.
   */
  const double dynamicOutlierRejectionThreshold;

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param enableEPI if set to true linear triangulation is refined with embedded LM iterations
   */
  TriangulationParameters(const double _rankTolerance = 1.0,
      const bool _enableEPI = false, double _landmarkDistanceThreshold = 1e10,
      double _dynamicOutlierRejectionThreshold = -1) :
      rankTolerance(_rankTolerance), enableEPI(_enableEPI), landmarkDistanceThreshold(
          _landmarkDistanceThreshold), dynamicOutlierRejectionThreshold(
          _dynamicOutlierRejectionThreshold) {
  }
};

struct TriangulationResult {
  Point3 point;
  bool degenerate;
  bool cheiralityException;
};

/// triangulateSafe: extensive checking of the outcome
template<class CAMERA>
TriangulationResult triangulateSafe(const std::vector<CAMERA>& cameras,
    const std::vector<Point2>& measured,
    const TriangulationParameters& params) {

  TriangulationResult result;

  size_t m = cameras.size();

  // if we have a single pose the corresponding factor is uninformative
  if (m < 2) {
    result.degenerate = true;
    return result;
  }

  // We triangulate the 3D position of the landmark
  try {
    // std::cout << "triangulatePoint3 i \n" << rankTolerance << std::endl;
    result.point = triangulatePoint3<CAMERA>(cameras, measured,
        params.rankTolerance, params.enableEPI);
    result.degenerate = false;
    result.cheiralityException = false;

    // Check landmark distance and reprojection errors to avoid outliers
    double totalReprojError = 0.0;
    size_t i = 0;
    BOOST_FOREACH(const CAMERA& camera, cameras) {
      Point3 cameraTranslation = camera.pose().translation();
      // we discard smart factors corresponding to points that are far away
      if (cameraTranslation.distance(result.point)
          > params.landmarkDistanceThreshold) {
        result.degenerate = true;
        break;
      }
      const Point2& zi = measured.at(i);
      try {
        Point2 reprojectionError(camera.project(result.point) - zi);
        totalReprojError += reprojectionError.vector().norm();
      } catch (CheiralityException) {
        result.cheiralityException = true;
      }
      i += 1;
    }
    // we discard smart factors that have large reprojection error
    if (params.dynamicOutlierRejectionThreshold > 0
        && totalReprojError / m > params.dynamicOutlierRejectionThreshold)
      result.degenerate = true;

  } catch (TriangulationUnderconstrainedException&) {
    // if TriangulationUnderconstrainedException can be
    // 1) There is a single pose for triangulation - this should not happen because we checked the number of poses before
    // 2) The rank of the matrix used for triangulation is < 3: rotation-only, parallel cameras (or motion towards the landmark)
    // in the second case we want to use a rotation-only smart factor
    result.degenerate = true;
    result.cheiralityException = false;
  } catch (TriangulationCheiralityException&) {
    // point is behind one of the cameras: can be the case of close-to-parallel cameras or may depend on outliers
    // we manage this case by either discarding the smart factor, or imposing a rotation-only constraint
    result.cheiralityException = true;
  }
  return result;
}

} // \namespace gtsam

