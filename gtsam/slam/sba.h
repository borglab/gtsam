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

namespace sba {

  using namespace gtsam;

  /**
   * Graph class, inherited from NonlinearFactorGraph, used as a convenience for MATLAB wrapper
   * @addtogroup SLAM
   */
  struct Graph: public visualSLAM::Graph {

    /// Default constructor
    Graph(){}

    /// Copy constructor given any other NonlinearFactorGraph
    Graph(const NonlinearFactorGraph& graph): visualSLAM::Graph(graph) {}

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
     *  Add a 2d projection measurement
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
  };

}
