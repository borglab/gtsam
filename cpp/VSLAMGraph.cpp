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
//TODO: CB: This constructor is specific to loading VO data. Should probably
//      get rid of this.
VSLAMGraph::VSLAMGraph(const std::string& path)
{
  ifstream ifs(path.c_str(), ios::in);

  if(ifs) {
    // read calibration K
    double fx, fy, s, u0, v0;
    ifs >> fx >> fy >> s >> u0 >> v0;
    Cal3_S2 K(fx, fy, s, u0, v0);

    // read sigma
    double sigma;
    ifs >> sigma;

    // read number of frames
    int nrFrames;
    ifs >> nrFrames;
    nFrames = nrFrames;

    // read all frames
    for (int i=0;i<nrFrames;i++) {
      int nrMeasurements;
      ifs >> nrMeasurements;
      // read all measurements in ith frame
      for (int k=0;k<nrMeasurements;k++) {
        int j; // landmark number
        double u, v;
        ifs >> j >> u >> v;
        Point2 z(u,v);

        // this works
        //VSLAMFactor::shared_ptr testing(new VSLAMFactor());
        //factors_.push_back(testing);

        VSLAMFactor::shared_ptr f1(new VSLAMFactor::VSLAMFactor(z.vector(), sigma, i+1, j,
        		VSLAMFactor::shared_ptrK(new Cal3_S2(K))));
        factors_.push_back(f1);
        //cout << "Added factor " << i+1 << endl;
        // just to keep a record of all the feature id's that have been encountered
        // value is unused/useless right now, but could be used to keep count
        feat_ids.insert(pair<int, int>(j,0));
      }
    }
  }
  else {
    printf("Unable to load values in %s\n", path.c_str());
    exit(0);
  }

  ifs.close();
}

/* ************************************************************************* */
bool VSLAMGraph::equals(const VSLAMGraph& p, double tol) const {
  if (&p == NULL) return false;
  if (nFrames != p.nFrames || feat_ids != p.feat_ids ) return false;
  return true;
}

/* ************************************************************************* */
bool compareLandmark(const std::string& key,
					const VSLAMConfig& feasible,
					const VSLAMConfig& input) {
	int j = atoi(key.substr(1, key.size() - 1).c_str());
	return feasible.landmarkPoint(j).equals(input.landmarkPoint(j));
}

/* ************************************************************************* */
void VSLAMGraph::addLandmarkConstraint(int j, const gtsam::Point3& p) {
  typedef NonlinearEquality<VSLAMConfig> NLE;
  VSLAMConfig feasible;
  feasible.addLandmarkPoint(j,p);
  boost::shared_ptr<NLE> factor(new NLE(symbol('l',j), feasible, 3, compareLandmark));
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
  typedef NonlinearEquality<VSLAMConfig> NLE;
  VSLAMConfig feasible;
  feasible.addCameraPose(j,p);
  boost::shared_ptr<NLE> factor(new NLE(symbol('x',j), feasible, 6, compareCamera));
  push_back(factor);
}

/* ************************************************************************* */

} // namespace gtsam

