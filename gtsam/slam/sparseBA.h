/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   sba.h
 * @brief  a suite for sparse bundle adjustment
 * @date   Jul 5, 2012
 * @author ydjian
 */

#pragma once

#include <gtsam/slam/visualSLAM.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace sparseBA {

  using namespace gtsam;

  /// Values class, inherited from Values, mainly used as a convenience for MATLAB wrapper
  struct Values: public visualSLAM::Values {

    typedef boost::shared_ptr<Values> shared_ptr;
    typedef gtsam::Values::ConstFiltered<SimpleCamera> SimpleCameraFiltered;
    typedef gtsam::Values::ConstFiltered<Cal3_S2> Cal3_S2Filtered;

    /// Default constructor
    Values() {}

    /// Copy constructor
    Values(const gtsam::Values& values) : visualSLAM::Values(values) {}

    /// Constructor from filtered values view of poses
    Values(const SimpleCameraFiltered& view) : visualSLAM::Values(view) {}

    /// Constructor from filtered values view of points
    Values(const PointFiltered& view) : visualSLAM::Values(view) {}

    SimpleCameraFiltered allSimpleCameras() const { return this->filter<SimpleCamera>(); }  ///< camera view
    size_t  nrSimpleCameras()  const { return allSimpleCameras().size(); }                  ///< get number of poses
    KeyList simpleCameraKeys() const { return allSimpleCameras().keys(); }                  ///< get keys to poses only

    /// insert a camera
    void insertSimpleCamera(Key j, const SimpleCamera& camera) { insert(j, camera); }

    /// update a camera
    void updateSimpleCamera(Key j, const SimpleCamera& camera) { update(j, camera); }

    /// get a camera
    SimpleCamera simpleCamera(Key j) const { return at<SimpleCamera>(j); }
  };

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Graph: public visualSLAM::Graph {

    /// Default constructor
    Graph(){}

    /// Copy constructor given any other NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph): visualSLAM::Graph(graph) {}

    /// check if two graphs are equal
    bool equals(const Graph& p, double tol = 1e-9) const {
      return NonlinearFactorGraph::equals(p, tol);
    }

    /**
      *  Add a prior on a pose
      *  @param key variable key of the camera
      *  @param p around which soft prior is defined
      *  @param model uncertainty model of this prior
      */
    template <typename Camera>
    void addCameraPrior(Key cameraKey, const Camera &camera, SharedNoiseModel &model = noiseModel::Unit::Create(Camera::Dim())) {
      sharedFactor factor(new PriorFactor<Camera>(cameraKey, camera, model));
      push_back(factor);
    }

    /**
     *  Add a constraint on a camera
     *  @param key variable key of the camera
     *  @param p to which camera to constrain it to
     */
    template <typename Camera>
    void addCameraConstraint(Key cameraKey, const Camera &camera) {
      sharedFactor factor(new NonlinearEquality<Camera>(cameraKey, camera));
      push_back(factor);
    }

    /**
     *  Add a 2d projection measurement
     *  @param z the measurement
     *  @param model the noise model for the measurement
     *  @param cameraKey variable key for the pose+calibration
     *  @param pointKey variable key for the point
     */
    template <typename Camera>
    void addMeasurement(const Point2 &z, const SharedNoiseModel& model, const Index cameraKey, const Index pointKey) {
      typedef GeneralSFMFactor<Camera, Point3> SFMFactor;
      boost::shared_ptr<SFMFactor> factor(new SFMFactor(z, model, cameraKey, pointKey));
      push_back(factor);
    }

    /**
     *  Add a 2d projection measurement, but supports separated (or shared) pose and calibration object
     *  @param z the measurement
     *  @param model the noise model for the measurement
     *  @param poseKey variable key for the pose
     *  @param pointKey variable key for the point
     *  @param calibKey variable key for the calibration
     */
    template <typename Calib>
    void addMeasurement(const Point2 &z, const SharedNoiseModel& model, const Index posekey, const Index pointkey, const Index calibkey) {
      typedef GeneralSFMFactor2<Calib> SFMFactor;
      boost::shared_ptr<SFMFactor> factor(new SFMFactor(z, model, posekey, pointkey, calibkey));
      push_back(factor);
    }

    /// Return a 2*K Matrix of reprojection errors
    Matrix reprojectionErrors(const Values& values) const;

    /**
     * Matlab-specific wrappers
     */

    void addSimpleCameraPrior(Key cameraKey, const SimpleCamera &camera, SharedNoiseModel &model);
    void addSimpleCameraConstraint(Key cameraKey, const SimpleCamera &camera) ;
    void addSimpleCameraMeasurement(const Point2 &z, SharedNoiseModel& model, Index cameraKey, Index pointKey);
  };

}
