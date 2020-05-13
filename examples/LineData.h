#ifndef SLAM_LINEDATA_H
#define SLAM_LINEDATA_H

#include <vector>
#include <utility>
#include <gtsam/geometry/Line3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/CalibratedCamera.h>

using namespace std;
using namespace gtsam;
typedef CalibratedCamera Camera;

/**
 * Creates a Line3 using two points lying on the line.
 * @param p Point 1
 * @param q Point 2
 * @return Line through p and q
 */
Line3 lineFromEndPoints(Point3 p, Point3 q) {
  Unit3 v = Unit3::FromPoint3((p - q).normalized());
  Unit3 plane_normal = Unit3::FromPoint3((p.cross(q)).normalized());
  Unit3 d_unit = plane_normal.cross(v);
  Point3 d = q.dot(d_unit.point3()) * d_unit.point3();

  Unit3 z(0, 0, 1);
  Rot3 R = Rot3::AlignPair(z.cross(v), v, z);
  Point3 ab_vec = R.transpose() * d;
  return Line3(R, ab_vec[0], ab_vec[1]);
}

/**
 * Structure that defines format of the line data.
 */
struct LineData {
  vector<Camera> cameras;            // Camera locations
  vector<Line3> lines;               // Lines in the scene
  // 2 Endpoints of lines in 3D - used for visualization
  vector<Point3> linesEnd1;
  vector<Point3> linesEnd2;
  // Detected endpoints of the line in each camera for each line
  vector<vector<Point2>> tracksEnd1;
  vector<vector<Point2>> tracksEnd2;
  Cal3_S2 K;                         // Camera calibration matrix
};

/**
 * Creates a line dataset with 3 cameras  and 6 lines.
 * The 6 lines are along 3 orthogonal directions.
 * @return
 */
LineData createSixLineDataset() {
  // calibration matrix from f, H, W
  Cal3_S2 K(90.0, 512, 512);
  LineData data;
  data.K = K;

  // 3 cameras on XY plane looking at origin
  data.cameras.push_back(Camera::Lookat(Point3(0, -3, 0), Point3(), Point3(0, 0, 1)));
  data.cameras.push_back(Camera::Lookat(Point3(3, -3, 0), Point3(), Point3(0, 0, 1)));
  data.cameras.push_back(Camera::Lookat(Point3(3, 0, 0), Point3(), Point3(0, 0, 1)));

  // 6 lines, along the edges of a cube
  data.linesEnd1.push_back(Point3(-1, 1, 0.2));
  data.linesEnd2.push_back(Point3(-1, 1, 0.8));

  data.linesEnd1.push_back(Point3(-1, 0.2, 1));
  data.linesEnd2.push_back(Point3(-1, 0.8, 1));

  data.linesEnd1.push_back(Point3(-0.8, 1, 1));
  data.linesEnd2.push_back(Point3(-0.2, 1, 1));

  data.linesEnd1.push_back(Point3(1, -1, -0.2));
  data.linesEnd2.push_back(Point3(1, -1, -0.8));

  data.linesEnd1.push_back(Point3(1, -0.2, -1));
  data.linesEnd2.push_back(Point3(1, -0.8, -1));

  data.linesEnd1.push_back(Point3(0.8, -1, -1));
  data.linesEnd2.push_back(Point3(0.2, -1, -1));

  for (int i = 0; i < 6; i++) {
    data.lines.push_back(lineFromEndPoints(data.linesEnd1[i], data.linesEnd2[i]));
    vector<Point2> ends1;
    vector<Point2> ends2;
    for (int j = 0; j < 3; j++) {
      ends1.push_back(data.cameras[j].project2(data.linesEnd1[i]));
      ends2.push_back(data.cameras[j].project2(data.linesEnd2[i]));
    }
    data.tracksEnd1.push_back(ends1);
    data.tracksEnd2.push_back(ends2);
  }
  return data;
}

/**
 * Returns a random float between lo and hi.
 * @param lo
 * @param hi
 * @return
 */
double getRandom(double lo = -1, double hi = 1) {
  return lo + (static_cast<double>(rand())*(hi - lo)/RAND_MAX);
}

/**
 * Creates a dataset with 3 cameras but with lines in random directions. 
 * The number of lines in the scene is an input argument. 
 * @param numLines
 * @return 
 */
LineData createLineDatasetRandom(int numLines = 10) {
  srand(0);
  Cal3_S2 K(90.0, 512, 512);
  LineData data;
  data.K = K;

  // 3 cameras looking at origin
  data.cameras.push_back(Camera::Lookat(Point3(0, -3, 0), Point3(), Point3(0, 0, 1)));
  data.cameras.push_back(Camera::Lookat(Point3(3, -3, 0), Point3(), Point3(0, 0, 1)));
  data.cameras.push_back(Camera::Lookat(Point3(3, 0, 0), Point3(), Point3(0, 0, 1)));

  // create lines
  for (int i = 0; i < numLines; i++) {
    data.linesEnd1.push_back(Point3(getRandom(), getRandom(), getRandom()));
    data.linesEnd2.push_back(Point3(getRandom(), getRandom(), getRandom()));
  }

  // create tracks in camera
  for (int i = 0; i < numLines; i++) {
    data.lines.push_back(lineFromEndPoints(data.linesEnd1[i], data.linesEnd2[i]));
    vector<Point2> ends1;
    vector<Point2> ends2;
    for (int j = 0; j < data.cameras.size(); j++) {
      ends1.push_back(data.cameras[j].project2(data.linesEnd1[i]));
      ends2.push_back(data.cameras[j].project2(data.linesEnd2[i]));
    }
    data.tracksEnd1.push_back(ends1);
    data.tracksEnd2.push_back(ends2);
  }
  return data;
}

#endif //SLAM_LINEDATA_H