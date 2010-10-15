#ifndef LANDMARKUTILS_H
#define LANDMARKUTILS_H

#include <map>
#include <vector>
#include "Feature2D.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Cal3_S2.h"


bool readLandMarks(const char* landmarkFile, std::map<int, gtsam::Point3>& landmarks);

gtsam::Pose3 readPose(const char* poseFn);
gtsam::Pose3 readPose(const char* poseFn_pre, const char* poseFn_suf, int poseId);

gtsam::Cal3_S2 readCalibData(const char* calibFn);

std::vector<Feature2D> readFeatures(const char* filename);
std::vector<Feature2D> readFeatures(const char* featFn_pre, const char* featFn_suf, int imageId);


#endif // LANDMARKUTILS_H
