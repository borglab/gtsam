/**
 * @file   VSLAMConfig.cpp
 * @brief  The Config
 * @author Alireza Fathi
 * @author Carlos Nieto
 */

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "VSLAMConfig.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

#define SIGMA 1.0

namespace gtsam {

/* ************************************************************************* */
// Exponential map
VSLAMConfig VSLAMConfig::exmap(const VectorConfig & delta) const {

	VSLAMConfig newConfig;

	for (map<string, Vector>::const_iterator it = delta.begin(); it
			!= delta.end(); it++) {
		string key = it->first;
		if (key[0] == 'x') {
			int cameraNumber = atoi(key.substr(1, key.size() - 1).c_str());
			if (cameraPoseExists(cameraNumber)) {
				Pose3 basePose = cameraPose(cameraNumber);
				newConfig.addCameraPose(cameraNumber, basePose.exmap(it->second));
			}
		}
		if (key[0] == 'l') {
			int landmarkNumber = atoi(key.substr(1, key.size() - 1).c_str());
			if (landmarkPointExists(landmarkNumber)) {
				Point3 basePoint = landmarkPoint(landmarkNumber);
				newConfig.addLandmarkPoint(landmarkNumber, basePoint.exmap(it->second));
			}
		}
	}

	return newConfig;
}

/* ************************************************************************* */
void VSLAMConfig::print(const std::string& s) const
{
  printf("%s:\n", s.c_str());
  printf("Camera Poses:\n");
  for(PoseMap::const_iterator it = cameraIteratorBegin(); it != cameraIteratorEnd(); it++)
  {
    printf("x%d:\n", it->first);
    it->second.print();
  }
  printf("Landmark Points:\n");
  for(PointMap::const_iterator it = landmarkIteratorBegin(); it != landmarkIteratorEnd(); it++)
  {
    printf("l%d:\n", (*it).first);
    (*it).second.print();
  }
}

/* ************************************************************************* */
bool VSLAMConfig::equals(const VSLAMConfig& c, double tol) const {
	for (PoseMap::const_iterator it = cameraIteratorBegin(); it
			!= cameraIteratorEnd(); it++) {
		if (!c.cameraPoseExists(it->first)) return false;
		if (!it->second.equals(c.cameraPose(it->first), tol)) return false;
	}

	for (PointMap::const_iterator it = landmarkIteratorBegin(); it
			!= landmarkIteratorEnd(); it++) {
		if (!c.landmarkPointExists(it->first)) return false;
		if (!it->second.equals(c.landmarkPoint(it->first), tol)) return false;
	}
	return true;
}

/* ************************************************************************* */
void VSLAMConfig::addCameraPose(const int i, Pose3 cp)
{
  pair<int, Pose3> camera;
  camera.first = i;
  camera.second = cp;
  cameraPoses_.insert(camera);
}

/* ************************************************************************* */
void VSLAMConfig::addLandmarkPoint(const int i, Point3 lp)
{
  pair<int, Point3> landmark;
  landmark.first = i;
  landmark.second = lp;
  landmarkPoints_.insert(landmark);
}

/* ************************************************************************* */
void VSLAMConfig::removeCameraPose(const int i)
{
  if(cameraPoseExists(i))
    cameraPoses_.erase(i);
}

/* ************************************************************************* */
void VSLAMConfig::removeLandmarkPose(const int i)
{
  if(landmarkPointExists(i))
    landmarkPoints_.erase(i);
}

/* ************************************************************************* */

} // namespace gtsam

