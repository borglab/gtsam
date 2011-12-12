/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file VSLAMutils.cpp
 * @brief
 * @author Duy-Nguyen Ta
 */

#include "vSLAMutils.h"
#include <fstream>
#include <cstdio>

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
std::map<int, Point3> readLandMarks(const std::string& landmarkFile) {
  ifstream file(landmarkFile.c_str());
  if (!file) {
    cout << "Cannot read landmark file: " << landmarkFile << endl;
    exit(0);
  }

  int num;
  file >> num;
  std::map<int, Point3> landmarks;
  landmarks.clear();
  for (int i = 0; i<num; i++) {
    int color_id;
    float x, y, z;
    file >> color_id >> x >> y >> z;
    landmarks[color_id] = Point3(x, y, z);
  }

  file.close();
  return landmarks;
}

/* ************************************************************************* */
gtsam::Pose3 readPose(const char* Fn) {
  ifstream poseFile(Fn);
  if (!poseFile) {
    cout << "Cannot read pose file: " << Fn << endl;
    exit(0);
  }

  double v[16];
  for (int i = 0; i<16; i++)
    poseFile >> v[i];
  poseFile.close();

  Matrix T = Matrix_(4,4, v);   // row order !!!

  Pose3 pose(T);
  return pose;
}
/* ************************************************************************* */
std::map<int, gtsam::Pose3> readPoses(const std::string& baseFolder, const std::string& posesFn) {
  ifstream posesFile((baseFolder+posesFn).c_str());
  if (!posesFile) {
    cout << "Cannot read all pose file: " << posesFn << endl;
    exit(0);
  }
  int numPoses;
  posesFile >> numPoses;
  map<int, Pose3> poses;
  for (int i = 0; i<numPoses; i++) {
    int poseId;
    posesFile >> poseId;

    string poseFileName;
    posesFile >> poseFileName;

    Pose3 pose = readPose((baseFolder+poseFileName).c_str());
    poses[poseId] = pose;
  }

  return poses;
}

/* ************************************************************************* */
gtsam::shared_ptrK readCalibData(const std::string& calibFn) {
  ifstream calibFile(calibFn.c_str());
  if (!calibFile) {
    cout << "Cannot read calib file: " << calibFn << endl;
    exit(0);
  }
  int imX, imY;
  float fx, fy, ox, oy;
  calibFile >> imX >> imY >> fx >> fy >> ox >> oy;
  calibFile.close();

  return shared_ptrK(new Cal3_S2(fx, fy, 0, ox, oy));   // skew factor = 0
}

/* ************************************************************************* */
std::vector<Feature2D> readFeatures(int pose_id, const char* filename) {
  ifstream file(filename);
  if (!file) {
    cout << "Cannot read feature file: " << filename<< endl;
    exit(0);
  }

  int numFeatures;
  file >> numFeatures ;

  std::vector<Feature2D> vFeatures_;
  for (int i = 0; i < numFeatures; i++) {
    int landmark_id; double x, y;
    file >> landmark_id >> x >> y;
    vFeatures_.push_back(Feature2D(pose_id, landmark_id, Point2(x, y)));
  }

  file.close();
  return vFeatures_;
}

/* ************************************************************************* */
std::vector<Feature2D> readAllMeasurements(const std::string& baseFolder, const std::string& measurementsFn) {
  ifstream measurementsFile((baseFolder+measurementsFn).c_str());
  if (!measurementsFile) {
    cout << "Cannot read all pose file: " << baseFolder+measurementsFn << endl;
    exit(0);
  }
  int numPoses;
  measurementsFile >> numPoses;

  vector<Feature2D> allFeatures;
  allFeatures.clear();

  for (int i = 0; i<numPoses; i++) {
    int poseId;
    measurementsFile >> poseId;

    string featureFileName;
    measurementsFile >> featureFileName;
    vector<Feature2D> features = readFeatures(poseId, (baseFolder+featureFileName).c_str());
    allFeatures.insert( allFeatures.end(), features.begin(), features.end() );
  }

  return allFeatures;
}

/* ************************************************************************* */
std::map<int, std::vector<Feature2D> > readAllMeasurementsISAM(const std::string& baseFolder, const std::string& measurementsFn) {
  ifstream measurementsFile((baseFolder+measurementsFn).c_str());
  if (!measurementsFile) {
    cout << "Cannot read all pose file: " << baseFolder+measurementsFn << endl;
    exit(0);
  }
  int numPoses;
  measurementsFile >> numPoses;

  std::map<int, std::vector<Feature2D> > allFeatures;

  for (int i = 0; i<numPoses; i++) {
    int poseId;
    measurementsFile >> poseId;

    string featureFileName;
    measurementsFile >> featureFileName;
    vector<Feature2D> features = readFeatures(poseId, (baseFolder+featureFileName).c_str());
    allFeatures[poseId] = features;
  }

  return allFeatures;
}
