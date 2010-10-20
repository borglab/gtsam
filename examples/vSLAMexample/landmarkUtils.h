#ifndef LANDMARKUTILS_H
#define LANDMARKUTILS_H

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


#endif // LANDMARKUTILS_H
