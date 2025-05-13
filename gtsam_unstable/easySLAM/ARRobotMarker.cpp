/**
 * @file   ARRobotMarker.cpp
 * @brief  Robot Marker pairs
 * @author Alireza Fathi
 */

#include "ARRobotMarker.h"

#include <fstream>
#include <iostream>

#include "Marker.h"
#include "homography.h"
#include "utility.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
ARRobotMarker::ARRobotMarker() {
  Vector zero(2, 0.0);
  for (int i = 0; i < 4; i++) Points.push_back(zero);
  robotNumber = markerNumber = 0;

  markerSize_ = 0.0;
  estimatedPose_ = Pose3(eye(4, 4));
}

/* ************************************************************************* */
ARRobotMarker::ARRobotMarker(int i, int j, Vector p1, Vector p2, Vector p3,
                             Vector p4, double mSize, Cal3_S2 K) {
  robotNumber = i;
  markerNumber = j;

  Points.push_back(p1);
  Points.push_back(p2);
  Points.push_back(p3);
  Points.push_back(p4);

  markerSize_ = mSize;
  K_ = K;

  estimate_pose();
}

/* ************************************************************************* */
ARRobotMarker::ARRobotMarker(int i, int j, double xa, double ya, double xb,
                             double yb, double xc, double yc, double xd,
                             double yd, double mSize, Cal3_S2 K) {
  robotNumber = i;
  markerNumber = j;

  Vector za(2), zb(2), zc(2), zd(2);
  za(0) = xa;
  za(1) = ya;
  zb(0) = xb;
  zb(1) = yb;
  zc(0) = xc;
  zc(1) = yc;
  zd(0) = xd;
  zd(1) = yd;
  Points.push_back(za);
  Points.push_back(zb);
  Points.push_back(zc);
  Points.push_back(zd);

  markerSize_ = mSize;
  K_ = K;

  estimate_pose();
}

/* ************************************************************************* */
ARRobotMarker::ARRobotMarker(int i, int j, Pose3 &pose, Pose3 &marker,
                             double mSize, Cal3_S2 K) {
  robotNumber = i;
  markerNumber = j;

  SimpleCamera camera(K, pose);

  // Calculate measurements
  /*
    Vector za = hGetP(camera, marker, mSize, 1).vector();
    Vector zb = hGetP(camera, marker, mSize, 1).vector();
    Vector zc = hGetP(camera, marker, mSize, 1).vector();
    Vector zd = hGetP(camera, marker, mSize, 1).vector();
  */
  Marker markerObj(marker, mSize);
  Vector za = hGetP(camera, markerObj, 1).vector();
  Vector zb = hGetP(camera, markerObj, 1).vector();
  Vector zc = hGetP(camera, markerObj, 1).vector();
  Vector zd = hGetP(camera, markerObj, 1).vector();
  Points.push_back(za);
  Points.push_back(zb);
  Points.push_back(zc);
  Points.push_back(zd);

  markerSize_ = mSize;
  K_ = K;

  estimate_pose();
}

/* ************************************************************************* */
ARRobotMarker::ARRobotMarker(int i, int j, SimpleCamera &camera, Pose3 &marker,
                             double mSize) {
  robotNumber = i;
  markerNumber = j;

  // Calculate measurements
  /*
          Vector za = hGetP(camera, marker, mSize, 1).vector();
    Vector zb = hGetP(camera, marker, mSize, 1).vector();
    Vector zc = hGetP(camera, marker, mSize, 1).vector();
    Vector zd = hGetP(camera, marker, mSize, 1).vector();
  */
  Marker markerObj(marker, mSize);
  Vector za = hGetP(camera, markerObj, 1).vector();
  Vector zb = hGetP(camera, markerObj, 1).vector();
  Vector zc = hGetP(camera, markerObj, 1).vector();
  Vector zd = hGetP(camera, markerObj, 1).vector();
  Points.push_back(za);
  Points.push_back(zb);
  Points.push_back(zc);
  Points.push_back(zd);

  markerSize_ = mSize;
  K_ = camera.calibration();

  estimate_pose();
}

/* ************************************************************************* */
void ARRobotMarker::print() const {
  printf("%d %d\n", getRobotNumber(), getMarkerNumber());
  for (int i = 0; i < 4; i++) ::print(Points[i]);
}

/* ************************************************************************* */
Vector ARRobotMarker::measurement() const {
  Vector result(8);
  for (int i = 0, j = 0; i < 4; i++, j += 2) {
    result(j) = Points[i](0);
    result(j + 1) = Points[i](1);
  }
  return result;
}

/* ************************************************************************* */
Matrix ARRobotMarker::estimate_H() {
  Vector v1 = getPoint(1);
  Vector v2 = getPoint(2);
  Vector v3 = getPoint(3);
  Vector v4 = getPoint(4);

  // should be done using Point2s -- FD
  Matrix m(4, 2);
  m(0, 0) = v1(0);
  m(0, 1) = v1(1);
  m(1, 0) = v2(0);
  m(1, 1) = v2(1);
  m(2, 0) = v3(0);
  m(2, 1) = v3(1);
  m(3, 0) = v4(0);
  m(3, 1) = v4(1);

  Matrix M(4, 3);
  double Msize = markerSize_ / 2.0;
  M(0, 0) = -Msize;
  M(1, 0) = -Msize;
  M(2, 0) = Msize;
  M(3, 0) = Msize;
  M(0, 1) = -Msize;
  M(1, 1) = Msize;
  M(2, 1) = Msize;
  M(3, 1) = -Msize;
  M(0, 2) = 1;
  M(1, 2) = 1;
  M(2, 2) = 1;
  M(3, 2) = 1;

  Matrix H = getH(m, M);
  return H;
}

/* ************************************************************************* */
void ARRobotMarker::estimate_pose() {
  Matrix H = estimate_H();
  estimatedPose_ = getTransformationFromMarkerToCamera(H, K_.matrix());
}

/* ************************************************************************* */

int next(int i) {
  if (i < 3) return i + 1;
  return 0;
}

int prev(int i) {
  if (i > 0) return i - 1;
  return 3;
}

int ARRobotMarker::fixOrderingOfPoints() {
  int whichIsM1 = 0;
  //  bool foundIt = false;
  Matrix averages(4, 2);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 2; j++)
      averages(i, j) = getPoint(i + 1)(j) + getPoint(next(i) + 1)(j);

  int minXind, maxXind, minYind, maxYind;
  double minX = double(INT_MAX);
  double minY = double(INT_MAX);
  double maxX = double(-INT_MAX);
  double maxY = double(-INT_MAX);

  for (int i = 0; i < 4; i++) {
    if (averages(i, 0) < minX) {
      minX = averages(i, 0);
      minXind = i;
    }
    if (averages(i, 0) > maxX) {
      maxX = averages(i, 0);
      maxXind = i;
    }
    if (averages(i, 1) < minY) {
      minY = averages(i, 1);
      minYind = i;
    }
    if (averages(i, 1) > maxY) {
      maxY = averages(i, 1);
      maxYind = i;
    }
  }

  //::print(averages);

  // printf("%d %d %d %d\n", minXind, maxXind, minYind, maxYind);

  vector<Vector> temp = Points;
  Points.clear();

  Points.push_back(temp[minXind]);
  Points.push_back(temp[minYind]);
  Points.push_back(temp[maxXind]);
  Points.push_back(temp[maxYind]);

  whichIsM1 = minXind;

  if (minXind == minYind) return -1;
  if (minXind == minYind) return -1;
  if (minXind == maxXind) return -1;
  if (minXind == maxYind) return -1;
  if (minYind == maxXind) return -1;
  if (minYind == maxYind) return -1;
  if (maxXind == maxYind) return -1;

  if (whichIsM1 != 0) estimate_pose();

  return whichIsM1;
}

/* ************************************************************************* */
bool ARRobotMarker::equals(ARRobotMarker &m) {
  if (getRobotNumber() != m.getRobotNumber()) goto fail;
  if (getMarkerNumber() != m.getMarkerNumber()) goto fail;
  for (int i = 1; i <= 4; i++)
    if (!::equal_with_abs_tol(getPoint(i), m.getPoint(i))) goto fail;

  return true;

fail:
  print();
  m.print();

  return false;
}

/* ************************************************************************* */
double ARRobotMarker::getArea() const {
  double difX1 = getPoint(1)(0) - getPoint(2)(0);
  double difY1 = getPoint(1)(1) - getPoint(2)(1);
  double difX2 = getPoint(3)(0) - getPoint(2)(0);
  double difY2 = getPoint(3)(1) - getPoint(2)(1);
  double ans = fabs(difX1 * difY2 - difX2 * difY1);
  return ans;
}

/* ************************************************************************* */
bool ARRobotMarker::isConvex() const {
  for (int i = 0; i < 4; i++) {
    Vector l1(2);
    Vector l2(2);
    l1 = getPoint(next(i) + 1) - getPoint(i + 1);
    l2 = getPoint(next(next(i)) + 1) - getPoint(next(i) + 1);
    //::print(l1);
    //::print(l2);
    Vector l1RR(2);  // l1 rotated right
    l1RR(0) = -l1(1);
    l1RR(1) = l1(0);
    double dot = l1RR(0) * l2(0) + l1RR(1) * l2(1);
    if (dot < 0) return false;
  }
  return true;
}

/* ************************************************************************* */
// loading the artoolkit provided information
vector<ARRobotMarker *> loadARToolKit(string path, int nrFrames, double DX,
                                      double DY) {
  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "%s/markerSize.txt", path.c_str());
  ifstream infile(buffer, ios::in);
  double markerSize;
  if (infile)
    infile >> markerSize;
  else {
    printf("Unable to load the markerSize for ARRobotMarker\n");
    printf("  Path: %s\n", buffer);
    exit(0);
  }
  infile.close();

  // loading other stuff such as calibration and markers locations
  Cal3_S2 K;
  K.load(path);

  double imageHeight = 320.0;
  double imageWidth = 480.0;

  int mn;
  vector<ARRobotMarker *> markers;
  for (int frame = 0; frame < nrFrames; frame++) {
    buffer[0] = 0;
    sprintf(buffer, "%s/mnum%06d.txt", path.c_str(), frame + 1);
    // printf("%s\n", buffer);
    ifstream inFileNum(buffer, ios::in);
    if (!inFileNum) throw CantOpenFile(buffer);
    buffer[0] = 0;
    sprintf(buffer, "%s/mmat%06d.txt", path.c_str(), frame + 1);
    ifstream inFileMat(buffer, ios::in);
    if (!inFileMat) throw CantOpenFile(buffer);
    // while ID file not empty,
    Vector shiftV = Vector_(2, DX, DY);
    while (!inFileNum.eof()) {
      inFileNum >> mn;  // read marker ID
      if (inFileNum.eof()) continue;
      // create and read new marker
      double xa, ya, xb, yb, xc, yc, xd, yd;
      inFileMat >> xa >> ya >> xb >> yb >> xc >> yc >> xd >> yd;

      buffer[0] = 0;
      sprintf(buffer, "%s/img%06d.jpg", path.c_str(), frame + 1);
      ARRobotMarker *newMarker = new ARRobotMarker(
          frame, mn, xa, ya, xb, yb, xc, yc, xd, yd, markerSize, K);
      markers.push_back(newMarker);
    }
    inFileNum.close();
    inFileMat.close();
  }

  return markers;
}

/* ************************************************************************* */
// loading the artoolkit provided information
void saveARToolKit(string path, vector<ARRobotMarker *> markers) {
  int frame = -1;
  vector<ARRobotMarker *>::iterator it;
  for (it = markers.begin(); it != markers.end(); it++) {
    if ((*it)->getRobotNumber() != frame) {
      frame = (*it)->getRobotNumber();
      char buffer[200];
      buffer[0] = 0.0;
      sprintf(buffer, "%s/mnum%06d.txt", path.c_str(), frame + 1);
      ofstream outFileNum(buffer, ios::out);
      if (!outFileNum) throw CantOpenFile(buffer);
      outFileNum.close();
      buffer[0] = 0.0;
      sprintf(buffer, "%s/mmat%06d.txt", path.c_str(), frame + 1);
      ofstream outFileMat(buffer, ios::out);
      if (!outFileMat) throw CantOpenFile(buffer);
      outFileMat.close();
    }

    if ((*it)->getRobotNumber() == frame) {
      char buffer[200];
      buffer[0] = 0.0;
      sprintf(buffer, "%s/mnum%06d.txt", path.c_str(), frame + 1);
      ofstream outFileNum(buffer, ios::app);
      buffer[0] = 0.0;
      sprintf(buffer, "%s/mmat%06d.txt", path.c_str(), frame + 1);
      ofstream outFileMat(buffer, ios::app);
      outFileNum << (*it)->getMarkerNumber() << endl;
      for (int p = 1; p <= 4; p++)
        outFileMat << (*it)->getPoint(p)(0) << " " << (*it)->getPoint(p)(1)
                   << endl;
      outFileNum.close();
      outFileMat.close();
    }
  }
}

/* ************************************************************************* */

vector<ARRobotMarker *> loadARToolKit_oneFrame(string path, int frameNumber,
                                               double DX, double DY) {
  char buffer[200];
  buffer[0] = 0;
  sprintf(buffer, "%s/markerSize.txt", path.c_str());
  ifstream infile(buffer, ios::in);
  double markerSize;
  if (infile)
    infile >> markerSize;
  else {
    printf("Unable to load the markerSize for ARRobotMarker\n");
    printf("  Path: %s\n", buffer);
    exit(0);
  }
  infile.close();

  // loading other stuff such as calibration and markers locations
  Cal3_S2 K;
  K.load(path);

  double imageHeight = 320.0;
  double imageWidth = 480.0;

  int mn;
  vector<ARRobotMarker *> markers;
  for (int frame = frameNumber; frame <= frameNumber; frame++) {
    char buffer[200];
    buffer[0] = 0.0;
    sprintf(buffer, "%s/mnum%06d.txt", path.c_str(), frame + 1);
    // printf("%s\n", buffer);
    ifstream inFileNum(buffer, ios::in);
    buffer[0] = 0.0;
    sprintf(buffer, "%s/mmat%06d.txt", path.c_str(), frame + 1);

    ifstream inFileMat(buffer, ios::in);
    // while ID file not empty,
    Vector shiftV = Vector_(2, DX, DY);
    while (!inFileNum.eof()) {
      inFileNum >> mn;  // read marker ID
      if (inFileNum.eof()) continue;
      // create and read new marker
      double xa, ya, xb, yb, xc, yc, xd, yd;
      inFileMat >> xa >> ya >> xb >> yb >> xc >> yc >> xd >> yd;

      buffer[0] = 0;
      sprintf(buffer, "%s/img%06d.jpg", path.c_str(), frame + 1);

      ARRobotMarker *newMarker = new ARRobotMarker(
          frame, mn, xa, ya, xb, yb, xc, yc, xd, yd, markerSize, K);

      markers.push_back(newMarker);
    }
    inFileNum.close();
    inFileMat.close();
  }
  return markers;
}

/* ************************************************************************* */
void dumpAR(const string &path, vector<ARRobotMarker *> markers) {
  ofstream outfile(path.c_str(), ios::out);
  for (vector<ARRobotMarker *>::iterator it = markers.begin();
       it != markers.end(); it++) {
    string Kdump = (*it)->getCalibration().dump();
    outfile << Kdump.c_str() << " " << (*it)->getMarkerSize() << endl;
    outfile << (*it)->getRobotNumber() << " " << (*it)->getMarkerNumber()
            << endl;
    for (int i = 1; i <= 4; i++)
      outfile << (*it)->getPoint(i)(0) << " " << (*it)->getPoint(i)(1) << endl;
  }
  outfile.close();
}
/* ************************************************************************* */
vector<ARRobotMarker *> load_dumpedAR(const string &path) {
  ifstream infile(path.c_str(), ios::in);
  vector<ARRobotMarker *> markers;
  while (infile) {
    double fx, fy, s, u0, v0;
    infile >> fx;
    if (!infile) break;
    infile >> fy >> s >> u0 >> v0;
    Cal3_S2 K(fx, fy, s, u0, v0);
    double markerSize, imageHeight, imageWidth;
    infile >> markerSize;
    int i, j;
    infile >> i;
    infile >> j;
    double xa, ya, xb, yb, xc, yc, xd, yd;
    infile >> xa >> ya >> xb >> yb >> xc >> yc >> xd >> yd;
    ARRobotMarker *m =
        new ARRobotMarker(i, j, xa, ya, xb, yb, xc, yc, xd, yd, markerSize, K);
    markers.push_back(m);
  }
  infile.close();
  return markers;
}

/*****************************************************************/
int getReferenceMarker(std::vector<ARRobotMarker *> markers,
                       int previousMarkerID) {
  if (previousMarkerID >= 0) {
    for (vector<ARRobotMarker *>::iterator it = markers.begin();
         it != markers.end(); it++)
      if ((*it)->getMarkerNumber() == previousMarkerID) return previousMarkerID;
  }
  double maxArea = 0.0;
  int bestMarker = 0;
  for (vector<ARRobotMarker *>::iterator it = markers.begin();
       it != markers.end(); it++) {
    double area = (*it)->getArea();
    if (area > maxArea) {
      bestMarker = (*it)->getMarkerNumber();
      maxArea = area;
    }
  }

  // int random = rand()%(int)(markers.size());
  // bestMarker = markers[random]->getMarkerNumber();
  return bestMarker;
}
/*****************************************************************/
