/**
 * @file    easyExample.h
 * @brief   Create examples for unit testing
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/Pose3.h>

#include "EasySLAMGraph.h"

/**
 *  create a factor
 */
CameraMarkerFactor::shared_ptr createExampleFactor1();

/**
 *  create a factor
 */
CameraMarkerFactor::shared_ptr createExampleFactor2();

/**
 *  create a small graph with two measurements of one marker
 */
EasySLAMGraph createExampleGraph();

// marker at the origin
gtsam::Pose3 marker1();

// camera at .5 meters, looking down at it
gtsam::Pose3 camera1();

// camera at same height, rolled 90 degrees ccw
gtsam::Pose3 camera2();

/**
 * make config with ground truth as above
 */
gtsam::FGConfig createExampleConfig();

/**
 * create a pose prior
 */
boost::shared_ptr<gtsam::NonlinearFactor1> createPosePrior(
    const gtsam::Pose3& pose, double sigma, const std::string& key1);
