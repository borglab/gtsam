/**
 * @file   EasySLAMConfig.cpp
 * @brief  The Config
 * @author Alireza Fathi
 * @author Frank Dellaert
 */

#include "EasySLAMConfig.h"

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <iostream>

#include "EasySLAMGraph.h"

using namespace std;
using namespace gtsam;

// trick from some reading group
#define FOREACH_PAIR(KEY, VAL, COL) BOOST_FOREACH (boost::tie(KEY, VAL), COL)

#define SIGMA 1.0

typedef boost::shared_ptr<CameraMarkerFactor0> shared_ptr0;
typedef boost::shared_ptr<CameraMarkerFactor> shared_ptr;

/* ************************************************************************* */
EasySLAMConfig::EasySLAMConfig(FGConfig& fgconfig) {
  markerPoses.clear();
  robotPoses.clear();

  for (map<std::string, Vector>::iterator it = fgconfig.begin();
       it != fgconfig.end(); it++) {
    string temp = (*it).first;
    // printf("%s\n", temp.c_str());
    if (temp[0] == 'x') {
      int robotNumber = atoi(temp.substr(1, temp.size() - 1).c_str());
      Pose3 robotPose(Pose3((*it).second));
      addRobotPose(robotNumber, robotPose);
    }
    if (temp[0] == 'm') {
      int markerNumber = atoi(temp.substr(1, temp.size() - 1).c_str());
      //::print((*it).second);
      Pose3 markerPose(Pose3((*it).second));
      addMarkerPose(markerNumber, markerPose);
    }
  }
}

/* ************************************************************************* */
// Initilaize the configuration by initializing the first pose from the
// reference marker, and then propagating the constraints to the markers,
// then to the next pose, then to its markers etc...
/* ************************************************************************* */
void EasySLAMConfig::load(int refMarker,
                          std::vector<ARRobotMarker*> measurements, Cal3_S2 K,
                          double markerSize) {
  // we will loop over frames, [from_it,to_it) is range of measurements in a
  // given frame
  vector<ARRobotMarker*>::iterator from_it = measurements.begin();
  vector<ARRobotMarker*>::iterator to_it = measurements.begin();

  // addMarkerPose((*from_it)->getMarkerNumber(), Pose3(eye(4,4)));
  // printf("Set the marker stuck to the wall to identity m%d\n",
  // (*from_it)->getMarkerNumber());

  // This looks like a loop over measurements, but really we are looping over
  // frames
  while (to_it != measurements.end()) {
    // find frame number
    int current_frame = (*from_it)->getRobotNumber();

    // measurements are sorted by frame, all measurments in current frame
    // search for the first measurement which is NOT in the current frame
    while (to_it != measurements.end() &&
           (*to_it)->getRobotNumber() == current_frame)
      to_it++;

    bool robotPoseAdded = false;
    // now [from_it,to_it) are all the measurements in the current frame, loop
    // over them
    for (vector<ARRobotMarker*>::iterator it = from_it; it != to_it; it++) {
      int markerNumber = (*it)->getMarkerNumber();
      // If this marker pose is known or is the reference marker, use it to
      // initialize the robot pose
      if (markerPoseExists(markerNumber) || markerNumber == refMarker) {
        Pose3 wTm;  // origin
        if (markerNumber != refMarker) wTm = markerPose(markerNumber);

        Pose3 cTm((*it)->getEstimatedPose());
        Pose3 mTc(cTm.inverse());
        Pose3 wTc(wTm * mTc);
        addRobotPose(current_frame, wTc);
        robotPoseAdded = true;
        break;  // jumps to LABEL
      }
    }  // end for

    // LABEL:
    // In the case there is no reference marker, we initialize the first pose to
    // the origin
    if (!robotPoseAdded && refMarker == -1) {
      Pose3 wTc;  // origin
      addRobotPose(current_frame, wTc);
      robotPoseAdded = true;
    }

    // we could not add a robot pose, print out diagnostic information
    if (!robotPoseAdded) {
      printf("----------------------------------------\n");
      printf(
          "could not add robot pose, markers are seen in one frame with index: "
          "\n");
      for (vector<ARRobotMarker*>::iterator it = from_it; it != to_it; it++) {
        int markerNumber = (*it)->getMarkerNumber();
        printf("%d\n", markerNumber);
        if (!markerPoseExists(markerNumber)) printf("does not exist\n");
      }
      printf("----------------------------------------\n");
    }

    // if robot pose added, add the markers in the current frame
    if (robotPoseAdded) {
      // add all the markers in the current frame, initialize them from the pose
      for (vector<ARRobotMarker*>::iterator it = from_it; it != to_it; it++) {
        int markerNumber = (*it)->getMarkerNumber();
        // if the marker corresponding to this measurement was not initialized
        // and is not the reference marker, initialize it from the pose
        if (!markerPoseExists(markerNumber) && markerNumber != refMarker) {
          Pose3 cTm((*it)->getEstimatedPose());
          Pose3 wTc(robotPose(current_frame));
          Pose3 wTm(wTc * cTm);
          addMarkerPose(markerNumber, wTm);
        }
      }
    }
    from_it = to_it;  // go to next frame
  }  // while

  // print();
}

/* ************************************************************************* */
void EasySLAMConfig::flush(int referenceMarker, const string& path) {
  char buffer[200];
  buffer[0];
  for (map<int, Pose3>::iterator rit = robotIteratorBegin();
       rit != robotIteratorEnd(); rit++) {
    buffer[0] = 0;
    int i = rit->first;
    sprintf(buffer, "%s/%d_robotPose%04d.txt", path.c_str(), referenceMarker,
            i);
    {
      Matrix rpm = rit->second.matrix();
      std::ofstream outfile(buffer, ios::out);
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) outfile << rpm(x, y) << " ";
        outfile << endl;
      }
      outfile.close();
    }
  }
  for (map<int, Pose3>::iterator mit = markerIteratorBegin();
       mit != markerIteratorEnd(); mit++) {
    buffer[0] = 0;
    int i = mit->first;
    sprintf(buffer, "%s/%d_markerPose%04d.txt", path.c_str(), referenceMarker,
            i);
    {
      Matrix mpm = mit->second.matrix();
      std::ofstream outfile(buffer, ios::out);
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) outfile << mpm(x, y) << " ";
        outfile << endl;
      }
      outfile.close();
    }
  }
}

/* ************************************************************************* */
void EasySLAMConfig::dump(const std::string& path) {
  std::ofstream outfile(path.c_str(), ios::out);
  for (map<int, Pose3>::iterator rit = robotIteratorBegin();
       rit != robotIteratorEnd(); rit++) {
    int i = rit->first;
    outfile << 0 << " " << i << endl;  // 0 means robot
    {
      Matrix rpm = rit->second.matrix();
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) outfile << rpm(x, y) << " ";
        outfile << endl;
      }
    }
  }
  for (map<int, Pose3>::iterator mit = markerIteratorBegin();
       mit != markerIteratorEnd(); mit++) {
    int i = mit->first;
    outfile << 1 << " " << i << endl;  // 1 means robot
    {
      Matrix mpm = mit->second.matrix();
      for (int x = 0; x < 4; x++) {
        for (int y = 0; y < 4; y++) outfile << mpm(x, y) << " ";
        outfile << endl;
      }
    }
  }
  outfile.close();
}
/* ************************************************************************* */
void EasySLAMConfig::load_dumped(const std::string& path) {
  markerPoses.clear();
  robotPoses.clear();
  ifstream infile(path.c_str(), ios::in);
  while (infile) {
    int type;
    int number;
    infile >> type;
    if (!infile) break;
    infile >> number;
    Matrix p(4, 4);
    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++) infile >> p(x, y);
    if (type == 0)
      robotPoses[number] = Pose3(p);
    else if (type == 1)
      markerPoses[number] = Pose3(p);
  }
}
/* ************************************************************************* */
void EasySLAMConfig::print(const std::string& s) {
  printf("%s:\n", s.c_str());
  printf("Robot Poses:\n");
  for (iterator it = robotIteratorBegin(); it != robotIteratorEnd(); it++) {
    printf("x%d:\n", it->first);
    it->second.print();
  }
  printf("Marker Poses:\n");
  for (iterator it = markerIteratorBegin(); it != markerIteratorEnd(); it++) {
    printf("m%d:\n", (*it).first);
    (*it).second.print();
  }
}
/* ************************************************************************* */
void EasySLAMConfig::load(string& path, int num_of_frames) {
  // loading the markerSize from the markerSize.txt, which exists in the path
  // directory
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

  vector<ARRobotMarker*> markers = loadARToolKit(path, num_of_frames);
  int refMarker = markers[0]->getMarkerNumber();
  load(refMarker, markers, K, markerSize);
}

/* ************************************************************************* */
void EasySLAMConfig::loadAFrame(string& path, int frameNumber, int refMarker) {
  // loading the markerSize from the markerSize.txt, which exists in the path
  // directory
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

  vector<ARRobotMarker*> measurements =
      loadARToolKit_oneFrame(path, frameNumber);
  load(refMarker, measurements, K, markerSize);
}

/* ************************************************************************* */
void EasySLAMConfig::loadAFrame(int refMarker, string& path, int frameNumber) {
  loadAFrame(path, frameNumber, refMarker);
}

/* ************************************************************************* */
bool EasySLAMConfig::equals(EasySLAMConfig& c, double tol) {
  // check sizes
  if (c.markerPoses.size() != markerPoses.size() ||
      c.robotPoses.size() != robotPoses.size())
    return false;

  // check individual parts
  for (iterator it = robotIteratorBegin(); it != robotIteratorEnd(); it++) {
    if (!c.robotPoseExists(it->first)) {
      return false;
    }
    Pose3 a = it->second;
    Pose3 b = c.robotPose(it->first);
    if (!(assert_equal(a.rotation(), b.rotation(), tol) ||
          assert_equal(a.translation(), b.translation(), tol)))
      return false;
  }

  for (iterator it = markerIteratorBegin(); it != markerIteratorEnd(); it++) {
    if (!c.markerPoseExists(it->first)) {
      return false;
    }
    if (!assert_equal(it->second, c.markerPose(it->first), tol)) return false;
  }

  return true;
}
/* ************************************************************************* */
void EasySLAMConfig::addRobotPose(const int i, Pose3 rp) {
  pair<int, Pose3> robot;
  robot.first = i;
  robot.second = rp;
  robotPoses.insert(robot);
}
/* ************************************************************************* */
void EasySLAMConfig::addMarkerPose(const int i, Pose3 mp) {
  pair<int, Pose3> marker;
  marker.first = i;
  marker.second = mp;
  markerPoses.insert(marker);
}
/* ************************************************************************* */
void EasySLAMConfig::removeRobotPose(const int i) {
  if (robotPoseExists(i)) robotPoses.erase(i);
}
/* ************************************************************************* */
void EasySLAMConfig::removeMarkerPose(const int i) {
  if (markerPoseExists(i)) markerPoses.erase(i);
}
/* ************************************************************************* */
EasyVectorConfig EasySLAMConfig::getFGConfig() const {
  EasyVectorConfig cfg;
  char buffer[100];
  string stbuf;
  int mn;
  Pose3 markerPose;
  FOREACH_PAIR(mn, markerPose, markerPoses) {
    buffer[0] = 0;
    sprintf(buffer, "m%d", mn);
    stbuf = string(buffer);
    cfg.insert(stbuf, markerPose.vector());
  }
  int rn;
  Pose3 robotPose;
  FOREACH_PAIR(rn, robotPose, robotPoses) {
    buffer[0] = 0;
    sprintf(buffer, "x%d", rn);
    stbuf = string(buffer);
    cfg.insert(stbuf, robotPose.vector());
  }
  return cfg;
}
/* ************************************************************************* */
EasySLAMConfig EasySLAMConfig::transform_to(const Pose3& transform) {
  EasySLAMConfig c;

  // loop through internal points, transform and add to output configuration
  pair<int, Pose3> pose;

  // copy and transform robots
  BOOST_FOREACH (pose, robotPoses) {
    Pose3 t_pose = pose.second.transformPose_to(
        transform);  // performs non-destructive transform - generates a new
                     // Pose3
    c.addRobotPose(pose.first, t_pose);
  }

  // copy and transform markers
  BOOST_FOREACH (pose, markerPoses) {
    Pose3 t_pose = pose.second.transformPose_to(
        transform);  // performs non-destructive transform - generates a new
                     // Pose3
    c.addMarkerPose(pose.first, t_pose);
  }

  return c;
}
/* ************************************************************************* */
bool assert_equal(const EasySLAMConfig& a, const EasySLAMConfig& b,
                  double tol) {
  // check sizes
  if (a.markerPoses.size() != b.markerPoses.size()) {
    cout << "Marker size mismatch: " << (int)a.markerPoses.size() << " and "
         << (int)b.markerPoses.size() << endl;
    return false;
  }
  if (a.robotPoses.size() != b.robotPoses.size()) {
    cout << "Robot size mismatch: " << (int)a.robotPoses.size() << " and "
         << (int)b.robotPoses.size() << endl;
    return false;
  }
  // check individual parts
  pair<int, Pose3> p;
  BOOST_FOREACH (p, a.robotPoses) {
    if (!b.robotPoseExists(p.first)) {
      cout << "Missing robot pose: " << p.first << endl;
      return false;
    }
    if (!assert_equal(p.second, b.robotPose(p.first), tol)) {
      cout << "Different robot pose: " << p.first << ":" << endl;
      p.second.print();
      return false;
    }
  }
  BOOST_FOREACH (p, a.markerPoses) {
    if (!b.markerPoseExists(p.first)) {
      cout << "Missing marker pose: " << p.first << endl;
      return false;
    }
    if (!assert_equal(p.second, b.markerPose(p.first), tol)) {
      cout << "Different marker pose: " << p.first << ":" << endl;
      p.second.print();
      return false;
    }
  }

  return true;
}
