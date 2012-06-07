/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file visualSLAM.h
 *  @brief Basic typedefs for general VisualSLAM problems. Useful for monocular and stereo systems.
 *  @date Jan 14, 2010
 *  @author Richard Roberts
 *  @author Chris Beall
 */

#pragma once

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace visualSLAM {

	using namespace gtsam;

  /**
   * Typedefs that make up the visualSLAM namespace.
   */
  typedef NonlinearEquality<Pose3> PoseConstraint;	 ///< put a hard constraint on a pose
  typedef NonlinearEquality<Point3> PointConstraint; ///< put a hard constraint on a point
  typedef PriorFactor<Pose3> PosePrior;							 ///< put a soft prior on a Pose
  typedef PriorFactor<Point3> PointPrior;						 ///< put a soft prior on a point
  typedef RangeFactor<Pose3, Point3> RangeFactor;  ///< put a factor on the range from a pose to a point
  
  /// monocular and stereo camera typedefs for general use
  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;
  typedef GenericStereoFactor<Pose3, Point3> StereoFactor;

  /// Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
  struct Values: public gtsam::Values {

    typedef boost::shared_ptr<Values> shared_ptr;

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
        gtsam::Values(values) {
    }

    /// insert a pose
    void insertPose(Key i, const Pose3& pose) { insert(i, pose); }

    /// insert a point
    void insertPoint(Key j, const Point3& point) { insert(j, point); }

    /// update a pose
    void updatePose(Key i, const Pose3& pose) { update(i, pose); }

    /// update a point
    void updatePoint(Key j, const Point3& point) { update(j, point); }

    /// get a pose
    Pose3 pose(Key i) const { return at<Pose3>(i); }

    /// get a point
    Point3 point(Key j) const { return at<Point3>(j); }

    /// check if value with specified key exists
    bool exists(Key i) const { return gtsam::Values::exists(i); }

    Vector xs() const; ///< get all pose x coordinates in a matrix
    Vector ys() const; ///< get all pose y coordinates in a matrix
    Vector zs() const; ///< get all pose z coordinates in a matrix

    Matrix points() const; ///< get all point coordinates in a matrix

  };

  /**
   * Non-linear factor graph for vanilla visual SLAM
   */
  class Graph: public NonlinearFactorGraph {

  public:
  	/// shared pointer to this type of graph
    typedef boost::shared_ptr<Graph> shared_graph;

    /// default constructor is empty graph
    Graph() {
    }

    /// print out graph
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      NonlinearFactorGraph::print(s, keyFormatter);
    }

    /// check if two graphs are equal
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph::equals(p, tol);
    }

    /**
     *  Add a projection factor measurement (monocular)
     *  @param measured the measurement
     *  @param model the noise model for the measurement
     *  @param poseKey variable key for the camera pose
     *  @param pointKey variable key for the landmark
     *  @param K shared pointer to calibration object
     */
    void addMeasurement(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const shared_ptrK K);

    /**
     *  Add a stereo factor measurement
     *  @param measured the measurement
     *  @param model the noise model for the measurement
     *  @param poseKey variable key for the camera pose
     *  @param pointKey variable key for the landmark
     *  @param K shared pointer to stereo calibration object
     */
    void addStereoMeasurement(const StereoPoint2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const shared_ptrKStereo K);

    /**
     *  Add a constraint on a pose (for now, *must* be satisfied in any Values)
     *  @param key variable key of the camera pose
     *  @param p to which pose to constrain it to
     */
    void addPoseConstraint(Key poseKey, const Pose3& p = Pose3());

    /**
     *  Add a constraint on a point (for now, *must* be satisfied in any Values)
     *  @param key variable key of the landmark
     *  @param p point around which soft prior is defined
     */
    void addPointConstraint(Key pointKey, const Point3& p = Point3());

    /**
     *  Add a prior on a pose
     *  @param key variable key of the camera pose
     *  @param p around which soft prior is defined
     *  @param model uncertainty model of this prior
     */
    void addPosePrior(Key poseKey, const Pose3& p = Pose3(), const SharedNoiseModel& model = noiseModel::Unit::Create(6));

    /**
     *  Add a prior on a landmark
     *  @param key variable key of the landmark
     *  @param p to which point to constrain it to
     *  @param model uncertainty model of this prior
     */
    void addPointPrior(Key pointKey, const Point3& p = Point3(), const SharedNoiseModel& model = noiseModel::Unit::Create(3));

    /**
     *  Add a range prior to a landmark
     *  @param poseKey variable key of the camera pose
     *  @param pointKey variable key of the landmark
     *  @param range approximate range to landmark
     *  @param model uncertainty model of this prior
     */
    void addRangeFactor(Key poseKey, Key pointKey, double range, const SharedNoiseModel& model = noiseModel::Unit::Create(1));

    /**
     *  Add an odometry between two poses
     *  @param x1 variable key of the first camera pose
     *  @param x2 variable key of the second camera pose
     *  @param odometry measurement from x1 to x2 (x1.between(x2))
     *  @param model uncertainty model of this measurement
     */
    void addOdometry(Key x1, Key x2, const Pose3& odometry, const SharedNoiseModel& model);

    /**
     *  Optimize the graph
     *  @param initialEstimate initial estimate of all variables in the graph
     *  @param pointKey variable key of the landmark
     *  @param range approximate range to landmark
     *  @param model uncertainty model of this prior
     */
    Values optimize(const Values& initialEstimate) {
      return LevenbergMarquardtOptimizer(*this, initialEstimate).optimize();
    }

    /// Return a Marginals object
    Marginals marginals(const Values& solution) const {
      return Marginals(*this,solution);
    }

  }; // Graph

  /**
   * Non-linear ISAM for vanilla incremental visual SLAM inference
   */
  typedef gtsam::NonlinearISAM ISAM;

} // namespaces
