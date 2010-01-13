/**
 * @file    VSLAMGraph.h
 * @brief   A factor graph for the VSLAM problem
 * @author  Alireza Fathi
 * @author  Carlos Nieto
 */

#include <set>
#include <fstream>
#include <boost/foreach.hpp>

#include "VSLAMGraph.h"
#include "NonlinearFactorGraph-inl.h"
#include "NonlinearOptimizer-inl.h"
#include "NonlinearEquality.h"

using namespace std;

namespace gtsam {

// explicit instantiation so all the code is there and we can link with it
template class FactorGraph<VSLAMFactor>;
template class NonlinearFactorGraph<VSLAMConfig>;
template class NonlinearOptimizer<VSLAMGraph,VSLAMConfig>;

/* ************************************************************************* */
bool compareLandmark(const std::string& key,
					const VSLAMConfig& feasible,
					const VSLAMConfig& input) {
	int j = atoi(key.substr(1, key.size() - 1).c_str());
	return feasible.landmarkPoint(j).equals(input.landmarkPoint(j));
}

/* ************************************************************************* */
void VSLAMGraph::addLandmarkConstraint(int j, const gtsam::Point3& p) {
  typedef NonlinearEquality<VSLAMConfig,VSLAMConfig::PointKey,Point3> NLE;
  boost::shared_ptr<NLE> factor(new NLE(j, p));
  push_back(factor);
}

/* ************************************************************************* */
bool compareCamera(const std::string& key,
					const VSLAMConfig& feasible,
					const VSLAMConfig& input) {
	int j = atoi(key.substr(1, key.size() - 1).c_str());
	return feasible.cameraPose(j).equals(input.cameraPose(j));
}

/* ************************************************************************* */
void VSLAMGraph::addCameraConstraint(int j, const gtsam::Pose3& p) {
  typedef NonlinearEquality<VSLAMConfig,VSLAMConfig::PoseKey,Pose3> NLE;
  boost::shared_ptr<NLE> factor(new NLE(j,p));
  push_back(factor);
}

/* ************************************************************************* */

} // namespace gtsam

