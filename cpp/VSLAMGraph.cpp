/**
 * @file    VSLAMGraph.h
 * @brief   A factor graph for the VSLAM problem
 * @author  Alireza Fathi
 * @author  Carlos Nieto
 */

#include <set>
#include <fstream>
#include <boost/foreach.hpp>

//#include "VSLAMFactor.h"
#include "VSLAMGraph.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
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

        VSLAMFactor::shared_ptr f1(new VSLAMFactor::VSLAMFactor(z.vector(), sigma, i+1, j, K));
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
VSLAMGraph::VSLAMGraph(const std::string& path,
			 int nrFrames, double sigma,
			 const gtsam::Cal3_S2 &K)
{
  ifstream ifs(path.c_str(), ios::in);

  if(ifs) {
    int cameraFrameNumber, landmarkNumber;
    double landmarkX, landmarkY, landmarkZ;
    double u, v;
    ifs >> cameraFrameNumber >> landmarkNumber >> u >> v >> landmarkX >> landmarkY >> landmarkZ;

    //Store the measurements
    Vector z(2);
    z(0)=u;
    z(1)=v;

    //VSLAMFactor::shared_ptr f1(new VSLAMFactor<VSLAMConfig>::VSLAMFactor(z, sigma, cameraFrameNumber, landmarkNumber, K));
    //factors_.push_back(f1);
  }
  else {
    printf("Unable to load values in %s\n", path.c_str());
    exit(0);
  }

  ifs.close();
}

/* ************************************************************************* */

