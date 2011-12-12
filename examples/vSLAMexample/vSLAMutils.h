/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Feature2D.cpp
 * @brief
 * @author Duy-Nguyen Ta
 */

#pragma once

#include <map>
#include <vector>
#include "Feature2D.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Cal3_S2.h"


std::map<int, gtsam::Point3> readLandMarks(const std::string& landmarkFile);

gtsam::Pose3 readPose(const char* poseFn);
std::map<int, gtsam::Pose3> readPoses(const std::string& baseFolder, const std::string& posesFN);

gtsam::shared_ptrK readCalibData(const std::string& calibFn);

std::vector<Feature2D> readFeatureFile(const char* filename);
std::vector<Feature2D> readAllMeasurements(const std::string& baseFolder, const std::string& measurementsFn);
std::map<int, std::vector<Feature2D> > readAllMeasurementsISAM(const std::string& baseFolder, const std::string& measurementsFn);
