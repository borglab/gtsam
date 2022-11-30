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
 * @author Luca Carlone
 * @author Akshay Krishnan
 */

#pragma once

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/SphericalCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/TriangulationFactor.h>

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
 * Same math as Hartley and Zisserman, 2nd Ed., page 312, but with unit-norm bearing vectors
 * (contrarily to pinhole projection, the z entry is not assumed to be 1 as in Hartley and Zisserman)
 * @param projection_matrices Projection matrices (K*P^-1)
 * @param measurements Unit3 bearing measurements
 * @param rank_tol SVD rank tolerance
 * @return Triangulated point, in homogeneous coordinates
 */
GTSAM_EXPORT Vector4 triangulateHomogeneousDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const std::vector<Unit3>& measurements, double rank_tol = 1e-9);

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
 * overload of previous function to work with Unit3 (projected to canonical camera)
 */
GTSAM_EXPORT Point3 triangulateDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const std::vector<Unit3>& measurements,
    double rank_tol = 1e-9);

/**
 * @brief Triangulation using the LOST (Linear Optimal Sine Triangulation)
 * algorithm proposed in https://arxiv.org/pdf/2205.12197.pdf by Sebastien Henry
 * and John Christian.
 * @param poses camera poses in world frame
 * @param calibratedMeasurements measurements in homogeneous coordinates in each
 * camera pose
 * @param measurementNoise isotropic noise model for the measurements
 * @return triangulated point in world coordinates
 */
GTSAM_EXPORT Point3 triangulateLOST(const std::vector<Pose3>& poses,
                                    const Point3Vector& calibratedMeasurements,
                                    const SharedIsotropic& measurementNoise);

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
    const Point3& initialEstimate,
    const SharedNoiseModel& model = noiseModel::Unit::Create(2)) {
  Values values;
  values.insert(landmarkKey, initialEstimate); // Initial landmark value
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < measurements.size(); i++) {
    const Pose3& pose_i = poses[i];
    typedef PinholePose<CALIBRATION> Camera;
    Camera camera_i(pose_i, sharedCal);
    graph.emplace_shared<TriangulationFactor<Camera> > //
        (camera_i, measurements[i], model, landmarkKey);
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
    const Point3& initialEstimate,
    const SharedNoiseModel& model = nullptr) {
  Values values;
  values.insert(landmarkKey, initialEstimate); // Initial landmark value
  NonlinearFactorGraph graph;
  static SharedNoiseModel unit(noiseModel::Unit::Create(
      traits<typename CAMERA::Measurement>::dimension));
  for (size_t i = 0; i < measurements.size(); i++) {
    const CAMERA& camera_i = cameras[i];
    graph.emplace_shared<TriangulationFactor<CAMERA> > //
        (camera_i, measurements[i], model? model : unit, landmarkKey);
  }
  return std::make_pair(graph, values);
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
    const Point2Vector& measurements, const Point3& initialEstimate,
    const SharedNoiseModel& model = nullptr) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraph<CALIBRATION> //
      (poses, sharedCal, measurements, Symbol('p', 0), initialEstimate, model);

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
    const typename CAMERA::MeasurementVector& measurements, const Point3& initialEstimate,
    const SharedNoiseModel& model = nullptr) {

  // Create a factor graph and initial values
  Values values;
  NonlinearFactorGraph graph;
  boost::tie(graph, values) = triangulationGraph<CAMERA> //
      (cameras, measurements, Symbol('p', 0), initialEstimate, model);

  return optimize(graph, values, Symbol('p', 0));
}

template<class CAMERA>
std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>
projectionMatricesFromCameras(const CameraSet<CAMERA> &cameras) {
  std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>> projection_matrices;
  for (const CAMERA &camera: cameras) {
    projection_matrices.push_back(camera.cameraProjectionMatrix());
  }
  return projection_matrices;
}

// overload, assuming pinholePose
template<class CALIBRATION>
std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>> projectionMatricesFromPoses(
        const std::vector<Pose3> &poses, boost::shared_ptr<CALIBRATION> sharedCal) {
  std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>> projection_matrices;
  for (size_t i = 0; i < poses.size(); i++) {
    PinholePose<CALIBRATION> camera(poses.at(i), sharedCal);
    projection_matrices.push_back(camera.cameraProjectionMatrix());
  }
  return projection_matrices;
}

/** Create a pinhole calibration from a different Cal3 object, removing
 * distortion.
 *
 * @tparam CALIBRATION Original calibration object.
 * @param cal Input calibration object.
 * @return Cal3_S2 with only the pinhole elements of cal.
 */
template <class CALIBRATION>
Cal3_S2 createPinholeCalibration(const CALIBRATION& cal) {
  const auto& K = cal.K();
  return Cal3_S2(K(0, 0), K(1, 1), K(0, 1), K(0, 2), K(1, 2));
}

/** Internal undistortMeasurement to be used by undistortMeasurement and
 * undistortMeasurements */
template <class CALIBRATION, class MEASUREMENT>
MEASUREMENT undistortMeasurementInternal(
    const CALIBRATION& cal, const MEASUREMENT& measurement,
    boost::optional<Cal3_S2> pinholeCal = boost::none) {
  if (!pinholeCal) {
    pinholeCal = createPinholeCalibration(cal);
  }
  return pinholeCal->uncalibrate(cal.calibrate(measurement));
}

/** Remove distortion for measurements so as if the measurements came from a
 * pinhole camera.
 *
 * Removes distortion but maintains the K matrix of the initial cal. Operates by
 * calibrating using full calibration and uncalibrating with only the pinhole
 * component of the calibration.
 * @tparam CALIBRATION Calibration type to use.
 * @param cal Calibration with which measurements were taken.
 * @param measurements Vector of measurements to undistort.
 * @return measurements with the effect of the distortion of sharedCal removed.
 */
template <class CALIBRATION>
Point2Vector undistortMeasurements(const CALIBRATION& cal,
                                   const Point2Vector& measurements) {
  Cal3_S2 pinholeCalibration = createPinholeCalibration(cal);
  Point2Vector undistortedMeasurements;
  // Calibrate with cal and uncalibrate with pinhole version of cal so that
  // measurements are undistorted.
  std::transform(measurements.begin(), measurements.end(),
                 std::back_inserter(undistortedMeasurements),
                 [&cal, &pinholeCalibration](const Point2& measurement) {
                   return undistortMeasurementInternal<CALIBRATION>(
                       cal, measurement, pinholeCalibration);
                 });
  return undistortedMeasurements;
}

/** Specialization for Cal3_S2 as it doesn't need to be undistorted. */
template <>
inline Point2Vector undistortMeasurements(const Cal3_S2& cal,
                                          const Point2Vector& measurements) {
  return measurements;
}

/** Remove distortion for measurements so as if the measurements came from a
 * pinhole camera.
 *
 * Removes distortion but maintains the K matrix of the initial calibrations.
 * Operates by calibrating using full calibration and uncalibrating with only
 * the pinhole component of the calibration.
 * @tparam CAMERA Camera type to use.
 * @param cameras Cameras corresponding to each measurement.
 * @param measurements Vector of measurements to undistort.
 * @return measurements with the effect of the distortion of the camera removed.
 */
template <class CAMERA>
typename CAMERA::MeasurementVector undistortMeasurements(
    const CameraSet<CAMERA>& cameras,
    const typename CAMERA::MeasurementVector& measurements) {
  const size_t nrMeasurements = measurements.size();
  assert(nrMeasurements == cameras.size());
  typename CAMERA::MeasurementVector undistortedMeasurements(nrMeasurements);
  for (size_t ii = 0; ii < nrMeasurements; ++ii) {
    // Calibrate with cal and uncalibrate with pinhole version of cal so that
    // measurements are undistorted.
    undistortedMeasurements[ii] =
        undistortMeasurementInternal<typename CAMERA::CalibrationType>(
            cameras[ii].calibration(), measurements[ii]);
  }
  return undistortedMeasurements;
}

/** Specialize for Cal3_S2 to do nothing. */
template <class CAMERA = PinholeCamera<Cal3_S2>>
inline PinholeCamera<Cal3_S2>::MeasurementVector undistortMeasurements(
    const CameraSet<PinholeCamera<Cal3_S2>>& cameras,
    const PinholeCamera<Cal3_S2>::MeasurementVector& measurements) {
  return measurements;
}

/** Specialize for SphericalCamera to do nothing. */
template <class CAMERA = SphericalCamera>
inline SphericalCamera::MeasurementVector undistortMeasurements(
    const CameraSet<SphericalCamera>& cameras,
    const SphericalCamera::MeasurementVector& measurements) {
  return measurements;
}

/** Convert pixel measurements in image to homogeneous measurements in the image
 * plane using shared camera intrinsics.
 *
 * @tparam CALIBRATION Calibration type to use.
 * @param cal Calibration with which measurements were taken.
 * @param measurements Vector of measurements to undistort.
 * @return homogeneous measurements in image plane
 */
template <class CALIBRATION>
inline Point3Vector calibrateMeasurementsShared(
    const CALIBRATION& cal, const Point2Vector& measurements) {
  Point3Vector calibratedMeasurements;
  // Calibrate with cal and uncalibrate with pinhole version of cal so that
  // measurements are undistorted.
  std::transform(measurements.begin(), measurements.end(),
                 std::back_inserter(calibratedMeasurements),
                 [&cal](const Point2& measurement) {
                   Point3 p;
                   p << cal.calibrate(measurement), 1.0;
                   return p;
                 });
  return calibratedMeasurements;
}

/** Convert pixel measurements in image to homogeneous measurements in the image
 * plane using camera intrinsics of each measurement.
 *
 * @tparam CAMERA Camera type to use.
 * @param cameras Cameras corresponding to each measurement.
 * @param measurements Vector of measurements to undistort.
 * @return homogeneous measurements in image plane
 */
template <class CAMERA>
inline Point3Vector calibrateMeasurements(
    const CameraSet<CAMERA>& cameras,
    const typename CAMERA::MeasurementVector& measurements) {
  const size_t nrMeasurements = measurements.size();
  assert(nrMeasurements == cameras.size());
  Point3Vector calibratedMeasurements(nrMeasurements);
  for (size_t ii = 0; ii < nrMeasurements; ++ii) {
    calibratedMeasurements[ii]
        << cameras[ii].calibration().calibrate(measurements[ii]),
        1.0;
  }
  return calibratedMeasurements;
}

/** Specialize for SphericalCamera to do nothing. */
template <class CAMERA = SphericalCamera>
inline Point3Vector calibrateMeasurements(
    const CameraSet<SphericalCamera>& cameras,
    const SphericalCamera::MeasurementVector& measurements) {
  Point3Vector calibratedMeasurements(measurements.size());
  for (size_t ii = 0; ii < measurements.size(); ++ii) {
    calibratedMeasurements[ii] << measurements[ii].point3();
  }
  return calibratedMeasurements;
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
 * @param useLOST whether to use the LOST algorithm instead of DLT
 * @return Returns a Point3
 */
template <class CALIBRATION>
Point3 triangulatePoint3(const std::vector<Pose3>& poses,
                         boost::shared_ptr<CALIBRATION> sharedCal,
                         const Point2Vector& measurements,
                         double rank_tol = 1e-9, bool optimize = false,
                         const SharedNoiseModel& model = nullptr,
                         const bool useLOST = false) {
  assert(poses.size() == measurements.size());
  if (poses.size() < 2) throw(TriangulationUnderconstrainedException());

  // Triangulate linearly
  Point3 point;
  if (useLOST) {
    // Reduce input noise model to an isotropic noise model using the mean of
    // the diagonal.
    const double measurementSigma = model ? model->sigmas().mean() : 1e-4;
    SharedIsotropic measurementNoise =
        noiseModel::Isotropic::Sigma(2, measurementSigma);
    // calibrate the measurements to obtain homogenous coordinates in image
    // plane.
    auto calibratedMeasurements =
        calibrateMeasurementsShared<CALIBRATION>(*sharedCal, measurements);

    point = triangulateLOST(poses, calibratedMeasurements, measurementNoise);
  } else {
    // construct projection matrices from poses & calibration
    auto projection_matrices = projectionMatricesFromPoses(poses, sharedCal);

    // Undistort the measurements, leaving only the pinhole elements in effect.
    auto undistortedMeasurements =
        undistortMeasurements<CALIBRATION>(*sharedCal, measurements);

    point =
        triangulateDLT(projection_matrices, undistortedMeasurements, rank_tol);
  }

  // Then refine using non-linear optimization
  if (optimize)
    point = triangulateNonlinear<CALIBRATION>  //
        (poses, sharedCal, measurements, point, model);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies in front of all cameras
  for (const Pose3& pose : poses) {
    const Point3& p_local = pose.transformTo(point);
    if (p_local.z() <= 0) throw(TriangulationCheiralityException());
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
 * @param useLOST whether to use the LOST algorithm instead of
 * DLT
 * @return Returns a Point3
 */
template <class CAMERA>
Point3 triangulatePoint3(const CameraSet<CAMERA>& cameras,
                         const typename CAMERA::MeasurementVector& measurements,
                         double rank_tol = 1e-9, bool optimize = false,
                         const SharedNoiseModel& model = nullptr,
                         const bool useLOST = false) {
  size_t m = cameras.size();
  assert(measurements.size() == m);

  if (m < 2) throw(TriangulationUnderconstrainedException());

  // Triangulate linearly
  Point3 point;
  if (useLOST) {
    // Reduce input noise model to an isotropic noise model using the mean of
    // the diagonal.
    const double measurementSigma = model ? model->sigmas().mean() : 1e-4;
    SharedIsotropic measurementNoise =
        noiseModel::Isotropic::Sigma(2, measurementSigma);

    // construct poses from cameras.
    std::vector<Pose3> poses;
    poses.reserve(cameras.size());
    for (const auto& camera : cameras) poses.push_back(camera.pose());

    // calibrate the measurements to obtain homogenous coordinates in image
    // plane.
    auto calibratedMeasurements =
        calibrateMeasurements<CAMERA>(cameras, measurements);

    point = triangulateLOST(poses, calibratedMeasurements, measurementNoise);
  } else {
    // construct projection matrices from poses & calibration
    auto projection_matrices = projectionMatricesFromCameras(cameras);

    // Undistort the measurements, leaving only the pinhole elements in effect.
    auto undistortedMeasurements =
        undistortMeasurements<CAMERA>(cameras, measurements);

    point =
        triangulateDLT(projection_matrices, undistortedMeasurements, rank_tol);
  }

  // Then refine using non-linear optimization
  if (optimize) {
    point = triangulateNonlinear<CAMERA>(cameras, measurements, point, model);
  }

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies in front of all cameras
  for (const CAMERA& camera : cameras) {
    const Point3& p_local = camera.pose().transformTo(point);
    if (p_local.z() <= 0) throw(TriangulationCheiralityException());
  }
#endif

  return point;
}

/// Pinhole-specific version
template <class CALIBRATION>
Point3 triangulatePoint3(const CameraSet<PinholeCamera<CALIBRATION>>& cameras,
                         const Point2Vector& measurements,
                         double rank_tol = 1e-9, bool optimize = false,
                         const SharedNoiseModel& model = nullptr,
                         const bool useLOST = false) {
  return triangulatePoint3<PinholeCamera<CALIBRATION>>  //
      (cameras, measurements, rank_tol, optimize, model, useLOST);
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

  SharedNoiseModel noiseModel; ///< used in the nonlinear triangulation

  /**
   * Constructor
   * @param rankTol tolerance used to check if point triangulation is degenerate
   * @param enableEPI if true refine triangulation with embedded LM iterations
   * @param landmarkDistanceThreshold flag as degenerate if point further than this
   * @param dynamicOutlierRejectionThreshold or if average error larger than this
   * @param noiseModel noise model to use during nonlinear triangulation
   *
   */
  TriangulationParameters(const double _rankTolerance = 1.0,
      const bool _enableEPI = false, double _landmarkDistanceThreshold = -1,
      double _dynamicOutlierRejectionThreshold = -1,
      const SharedNoiseModel& _noiseModel = nullptr) :
      rankTolerance(_rankTolerance), enableEPI(_enableEPI), //
      landmarkDistanceThreshold(_landmarkDistanceThreshold), //
      dynamicOutlierRejectionThreshold(_dynamicOutlierRejectionThreshold),
      noiseModel(_noiseModel){
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
    os << "noise model" << std::endl;
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
 * TriangulationResult is an optional point, along with the reasons why it is
 * invalid.
 */
class TriangulationResult : public boost::optional<Point3> {
 public:
  enum Status { VALID, DEGENERATE, BEHIND_CAMERA, OUTLIER, FAR_POINT };
  Status status;

 private:
  TriangulationResult(Status s) : status(s) {}

 public:
  /**
   * Default constructor, only for serialization
   */
  TriangulationResult() {}

  /**
   * Constructor
   */
  TriangulationResult(const Point3& p) : status(VALID) { reset(p); }
  static TriangulationResult Degenerate() {
    return TriangulationResult(DEGENERATE);
  }
  static TriangulationResult Outlier() { return TriangulationResult(OUTLIER); }
  static TriangulationResult FarPoint() {
    return TriangulationResult(FAR_POINT);
  }
  static TriangulationResult BehindCamera() {
    return TriangulationResult(BEHIND_CAMERA);
  }
  bool valid() const { return status == VALID; }
  bool degenerate() const { return status == DEGENERATE; }
  bool outlier() const { return status == OUTLIER; }
  bool farPoint() const { return status == FAR_POINT; }
  bool behindCamera() const { return status == BEHIND_CAMERA; }
  // stream to output
  friend std::ostream& operator<<(std::ostream& os,
                                  const TriangulationResult& result) {
    if (result)
      os << "point = " << *result << std::endl;
    else
      os << "no point, status = " << result.status << std::endl;
    return os;
  }

 private:
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& BOOST_SERIALIZATION_NVP(status);
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
      Point3 point =
          triangulatePoint3<CAMERA>(cameras, measured, params.rankTolerance,
                                    params.enableEPI, params.noiseModel);

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
          const typename CAMERA::Measurement& zi = measured.at(i);
          Point2 reprojectionError = camera.reprojectionError(point, zi);
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

// Vector of Cameras - used by the Python/MATLAB wrapper
using CameraSetCal3Bundler = CameraSet<PinholeCamera<Cal3Bundler>>;
using CameraSetCal3_S2 = CameraSet<PinholeCamera<Cal3_S2>>;
using CameraSetCal3DS2 = CameraSet<PinholeCamera<Cal3DS2>>;
using CameraSetCal3Fisheye = CameraSet<PinholeCamera<Cal3Fisheye>>;
using CameraSetCal3Unified = CameraSet<PinholeCamera<Cal3Unified>>;
using CameraSetSpherical = CameraSet<SphericalCamera>;
} // \namespace gtsam

