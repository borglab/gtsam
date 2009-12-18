/**
 * @file   UrbanConfig.cpp
 * @brief  The Config for Urban example
 * @author Frank Dellaert
 * @author Viorela Ila
 */

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>

#include "UrbanConfig.h"

using namespace std;

// trick from some reading group
#define FOREACH_PAIR( KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY,VAL),COL)

#define SIGMA 1.0

namespace gtsam {

/* ************************************************************************* */
// Exponential map
UrbanConfig UrbanConfig::exmap(const VectorConfig & delta) const {

	UrbanConfig newConfig;

	for (map<string, Vector>::const_iterator it = delta.begin(); it
			!= delta.end(); it++) {
		string key = it->first;
		if (key[0] == 'x') {
			int robotNumber = atoi(key.substr(1, key.size() - 1).c_str());
			if (robotPoseExists(robotNumber)) {
				Pose3 basePose = robotPose(robotNumber);
				newConfig.addRobotPose(robotNumber, basePose.exmap(it->second));
			}
		}
		if (key[0] == 'l') {
			int landmarkNumber = atoi(key.substr(1, key.size() - 1).c_str());
			if (landmarkPointExists(landmarkNumber)) {
				Point2 basePoint = landmarkPoint(landmarkNumber);
				newConfig.addLandmark(landmarkNumber, basePoint.exmap(it->second));
			}
		}
	}

	return newConfig;
}

/* ************************************************************************* */
void UrbanConfig::print(const std::string& s) const
{
  printf("%s:\n", s.c_str());
  printf("robot Poses:\n");
  for(PoseMap::const_iterator it = robotIteratorBegin(); it != robotIteratorEnd(); it++)
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
bool UrbanConfig::equals(const UrbanConfig& c, double tol) const {
	for (PoseMap::const_iterator it = robotIteratorBegin(); it
			!= robotIteratorEnd(); it++) {
		if (!c.robotPoseExists(it->first)) return false;
		if (!it->second.equals(c.robotPose(it->first), tol)) return false;
	}

	for (PointMap::const_iterator it = landmarkIteratorBegin(); it
			!= landmarkIteratorEnd(); it++) {
		if (!c.landmarkPointExists(it->first)) return false;
		if (!it->second.equals(c.landmarkPoint(it->first), tol)) return false;
	}
	return true;
}

/* ************************************************************************* */
void UrbanConfig::addRobotPose(const int i, Pose3 cp)
{
  robotPoses_.insert(make_pair(i,cp));
}

/* ************************************************************************* */
void UrbanConfig::addLandmark(const int j, Point2 lp)
{
  landmarkPoints_.insert(make_pair(j,lp));
}

/* ************************************************************************* */
void UrbanConfig::removeRobotPose(const int i)
{
  if(robotPoseExists(i))
    robotPoses_.erase(i);
}

/* ************************************************************************* */
void UrbanConfig::removeLandmark(const int i)
{
  if(landmarkPointExists(i))
    landmarkPoints_.erase(i);
}

/* ************************************************************************* */

} // namespace gtsam

