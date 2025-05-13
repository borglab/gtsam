/**
 * @file    easySLAMGraph.cpp
 * @brief   A factor graph for the EasySLAM problem
 * @author  Alireza Fathi
 */

#include "EasySLAMGraph.h"

#include <colamd/colamd.h>

#include <boost/foreach.hpp>
#include <fstream>
#include <set>

#include "ARRobotMarker.h"

using namespace std;
using namespace gtsam;

#define LOCAL_SIGMA 1.0

void EasySLAMGraph::load(int refMarker, std::vector<ARRobotMarker *> markers,
                         gtsam::Cal3_S2 K, double markerSize) {
  set<int> markRobotFrames;
  set<int> markMarkers;
  char buffer[200];

  for (vector<ARRobotMarker *>::iterator it = markers.begin();
       it != markers.end(); it++) {
    if (markRobotFrames.find((*it)->getRobotNumber()) ==
        markRobotFrames.end()) {
      markRobotFrames.insert((*it)->getRobotNumber());
    }
  }

  for (vector<ARRobotMarker *>::iterator it = markers.begin();
       it != markers.end(); it++) {
    if ((*it)->getMarkerNumber() == refMarker) continue;
    if (markMarkers.find((*it)->getMarkerNumber()) == markMarkers.end()) {
      markMarkers.insert((*it)->getMarkerNumber());
    }
  }

  double sigma = LOCAL_SIGMA;

  // bool firstMarkerBeingAdded = true;
  for (vector<ARRobotMarker *>::iterator it = markers.begin();
       it != markers.end(); it++) {
    const ARRobotMarker &marker = **it;
    if (refMarker != (*it)->getMarkerNumber()) {
      shared_ptr f1(new CameraMarkerFactor(marker, sigma));
      push_back(f1);
    } else {
      shared_ptr0 f2(new CameraMarkerFactor0(marker, sigma));
      push_back(f2);
    }
  }
}
/* ************************************************************************* */
// see doc in .h file
EasySLAMGraph::EasySLAMGraph(const string &path, int nrFrames) {
  // defined in Loader, it contains pairs of robot landmarks from images.
  vector<ARRobotMarker *> markers = loadARToolKit(path, nrFrames);
  int refMarker = markers[0]->getMarkerNumber();

  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "%s/markerSize.txt", path.c_str());
  ifstream infile(buffer, ios::in);
  double markerSize;
  if (infile)
    infile >> markerSize;
  else {
    printf("Unable to load the markerSize\n");
    exit(0);
  }
  infile.close();

  // loading other stuff such as calibration and markers locations
  Cal3_S2 K;
  K.load(path);
  load(refMarker, markers, K, markerSize);
}

/* ************************************************************************* */
void EasySLAMGraph::loadAFrame(const string &path, int frameNumber,
                               int refMarker) {
  // load a frame with the frame number
  vector<ARRobotMarker *> markers = loadARToolKit_oneFrame(path, frameNumber);

  // load the marker size
  // FIXME: Move this to a once-fired function
  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "%s/markerSize.txt", path.c_str());

  ifstream infile(buffer, ios::in);
  double markerSize;
  if (infile)
    infile >> markerSize;
  else {
    printf("Unable to load the markerSize\n");
    exit(0);
  }
  infile.close();

  // loading other stuff such as calibration and markers locations
  Cal3_S2 K;
  K.load(path);

  insertMarker(markers, frameNumber, refMarker);
}

/* ************************************************************************* */
void EasySLAMGraph::loadAFrame(int refMarker, const string &path,
                               int frameNumber) {
  loadAFrame(path, frameNumber, refMarker);
}

/* ************************************************************************* */
void EasySLAMGraph::insertMarker(vector<ARRobotMarker *> markers,
                                 int frameNumber, int refMarker) {
  double sigma = LOCAL_SIGMA;
  for (vector<ARRobotMarker *>::iterator it = markers.begin();
       it != markers.end(); it++) {
    const ARRobotMarker &marker = **it;
    if (marker.getMarkerNumber() == refMarker) {
      shared_ptr0 f(new CameraMarkerFactor0(marker, sigma));
      push_back(f);
    }
    if (refMarker == -1 && frameNumber == 0) {
      shared_ptr1 f(new CameraMarkerFactor1(marker, sigma));
      push_back(f);
    }

    if (marker.getMarkerNumber() != refMarker) {
      shared_ptr f(new CameraMarkerFactor(marker, sigma));
      push_back(f);
    }
  }
}

/* ************************************************************************* */
void EasySLAMGraph::dump(const std::string &path) {
  string result = "";
  for (size_t i = 0; i < factors.size(); i++) {
    // printf("result%d:\n%s\n", factors.size(), result.c_str());
    string temp = factors[i]->dump();
    // printf("%s\n", temp.c_str());
    result.append(temp);
    result.append("\n");
  }
  ofstream outfile(path.c_str(), ios::out);
  outfile << result.c_str();
  outfile.close();
}
/* ************************************************************************* */
void EasySLAMGraph::load_dumped(const std::string &path) {
  factors.clear();
  ifstream infile(path.c_str(), ios::in);
  while (infile) {
    int type;  // 0 means CameraMarkerFactor0 and 1 means CameraMarkerFactor
    infile >> type;
    if (!infile) break;
    int i, j;
    infile >> i;
    if (type == 1) infile >> j;
    double sigma;
    infile >> sigma;
    int vecSize;
    infile >> vecSize;
    Vector z(vecSize);
    for (int i = 0; i < vecSize; i++) infile >> z(i);
    double fx, fy, s, u0, v0;
    infile >> fx >> fy >> s >> u0 >> v0;
    Cal3_S2 K(fx, fy, s, u0, v0);
    double markerSize;
    infile >> markerSize;
    if (type == 0) {
      shared_ptr0 f(new CameraMarkerFactor0(z, sigma, i, K, markerSize));
      push_back(f);
    } else if (type == 1) {
      shared_ptr f(new CameraMarkerFactor(z, sigma, i, j, K, markerSize));
      push_back(f);
    }
  }
}
/* ************************************************************************* */
