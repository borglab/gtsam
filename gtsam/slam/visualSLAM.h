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

#include <gtsam/slam/pose3SLAM.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearISAM.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace visualSLAM {

	using namespace gtsam;

  /// Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
  struct Values: public pose3SLAM::Values {

    typedef boost::shared_ptr<Values> shared_ptr;
    typedef gtsam::Values::ConstFiltered<Pose3> PoseFiltered;
    typedef gtsam::Values::ConstFiltered<Point3> PointFiltered;

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) :
    	pose3SLAM::Values(values) {
    }

    /// Constructor from filtered values view of poses
    Values(const PoseFiltered& view) : pose3SLAM::Values(view) {}

    /// Constructor from filtered values view of points
    Values(const PointFiltered& view) : pose3SLAM::Values(view) {}

    PoseFiltered allPoses() const { return this->filter<Pose3>(); } ///< pose view
    size_t  nrPoses()  const { return allPoses().size(); } ///< get number of poses
    KeyList poseKeys() const { return allPoses().keys(); } ///< get keys to poses only

    PointFiltered allPoints() const { return this->filter<Point3>(); } ///< point view
    size_t  nrPoints()  const { return allPoints().size(); } ///< get number of points
    KeyList pointKeys() const { return allPoints().keys(); } ///< get keys to points only

    /// insert a point
    void insertPoint(Key j, const Point3& point) { insert(j, point); }

    /// update a point
    void updatePoint(Key j, const Point3& point) { update(j, point); }

    /// get a point
    Point3 point(Key j) const { return at<Point3>(j); }

    /// insert a number of initial point values by backprojecting
    void insertBackprojections(const SimpleCamera& c, const Vector& J,
        const Matrix& Z, double depth);

    /// perturb all points using normally distributed noise
    void perturbPoints(double sigma, int32_t seed = 42u);

    /// get all point coordinates in a matrix
    Matrix points() const;

  };

  /**
   * Non-linear factor graph for vanilla visual SLAM
   */
  struct Graph: public pose3SLAM::Graph {

  	/// shared pointer to this type of graph
    typedef boost::shared_ptr<Graph> shared_graph;

	  /// Default constructor
    Graph(){}

    /// Copy constructor given any other NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph):
    	pose3SLAM::Graph(graph) {}

    /// check if two graphs are equal
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph::equals(p, tol);
    }

    /**
     *  Add a constraint on a point (for now, *must* be satisfied in any Values)
     *  @param key variable key of the point
     *  @param p point around which soft prior is defined
     */
    void addPointConstraint(Key pointKey, const Point3& p = Point3());

    /**
     *  Add a prior on a point
     *  @param key variable key of the point
     *  @param p to which point to constrain it to
     *  @param model uncertainty model of this prior
     */
    void addPointPrior(Key pointKey, const Point3& p = Point3(),
				const SharedNoiseModel& model = noiseModel::Unit::Create(3));

    /**
     *  Add a range prior to a point
     *  @param poseKey variable key of the camera pose
     *  @param pointKey variable key of the point
     *  @param range approximate range to point
     *  @param model uncertainty model of this prior
     */
    void addRangeFactor(Key poseKey, Key pointKey, double range,
				const SharedNoiseModel& model = noiseModel::Unit::Create(1));

    /**
     *  Add a projection factor measurement (monocular)
     *  @param measured the measurement
     *  @param model the noise model for the measurement
     *  @param poseKey variable key for the camera pose
     *  @param pointKey variable key for the point
     *  @param K shared pointer to calibration object
     */
    void addMeasurement(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const shared_ptrK K);

    /**
     *  Add a number of measurements at the same time
     *  @param i variable key for the camera pose
     *  @param J variable keys for the point, KeyVector of size K
     *  @param Z the 2*K measurements as a matrix
     *  @param model the noise model for the measurement
     *  @param K shared pointer to calibration object
     */
    void addMeasurements(Key i, const Vector& J, const Matrix& Z,
        const SharedNoiseModel& model, const shared_ptrK K);

    /**
     *  Add a stereo factor measurement
     *  @param measured the measurement
     *  @param model the noise model for the measurement
     *  @param poseKey variable key for the camera pose
     *  @param pointKey variable key for the point
     *  @param K shared pointer to stereo calibration object
     */
    void addStereoMeasurement(const StereoPoint2& measured, const SharedNoiseModel& model,
        Key poseKey, Key pointKey, const shared_ptrKStereo K);

  }; // Graph

  /**
   * Non-linear ISAM for vanilla incremental visual SLAM inference
   */
  typedef gtsam::NonlinearISAM ISAM;

} // visualSLAM

/**
 * Backwards compatibility and wrap use only, avoid using
 */
namespace visualSLAM {
	typedef gtsam::NonlinearEquality<Pose3> PoseConstraint; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::NonlinearEquality<Point3> PointConstraint; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::PriorFactor<Pose3> PosePrior; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::PriorFactor<Point3> PointPrior; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::RangeFactor<Pose3, Point3> RangeFactor; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::GenericProjectionFactor<Pose3, Point3> ProjectionFactor; ///< \deprecated typedef for backwards compatibility
	typedef gtsam::GenericStereoFactor<Pose3, Point3> StereoFactor; ///< \deprecated typedef for backwards compatibility
}

