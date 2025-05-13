/**
 * @file    Marker.cpp
 * @brief   It contains the characteristics of our markers.
 * @author  Alireza Fathi
 */

#include "Marker.h"

#include <gtsam/SimpleCamera.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
Point3 get_marker_point(const double markerSize,
                        const int whichPoint /*1,2,3,4*/) {
  Point3 mX_P;  // the location of P in marker coordinate.
  switch (whichPoint) {
    case 1:
      mX_P = Point3(-markerSize / 2, -markerSize / 2, 0.0);
      break;
    case 2:
      mX_P = Point3(-markerSize / 2, markerSize / 2, 0.0);
      break;
    case 3:
      mX_P = Point3(markerSize / 2, markerSize / 2, 0.0);
      break;
    case 4:
      mX_P = Point3(markerSize / 2, -markerSize / 2, 0.0);
      break;
    default:
      throw(std::invalid_argument("get_marker_point: invalid point"));
  }
  return mX_P;
}
/* ************************************************************************* */

Point2 hGetP(const SimpleCamera& camera, Marker& markerObj, int num) {
  Point3 markerPoint = get_marker_point(markerObj.getmarkerSize(), num);
  Point3 worldPoint = transform_from(markerObj.getmarkerPose(), markerPoint);
  Point2 projection = project(camera, worldPoint);
  return projection;
}

/*Point2 hGetP(const SimpleCamera& camera, const Pose3& markerPose,
                double markerSize, int num) {
        Point3 markerPoint = get_marker_point(markerSize, num);
        Point3 worldPoint = transform_from(markerPose, markerPoint);
        Point2 projection = project(camera, worldPoint);
        return projection;
}*/

Vector hGetAll(const SimpleCamera& camera, Marker& markerObj) {
  /*
  Vector hA = hGetP(camera, markerPose, markerSize, 1).vector();
  Vector hB = hGetP(camera, markerPose, markerSize, 2).vector();
  Vector hC = hGetP(camera, markerPose, markerSize, 3).vector();
  Vector hD = hGetP(camera, markerPose, markerSize, 4).vector();
  */
  Vector hA = hGetP(camera, markerObj, 1).vector();
  Vector hB = hGetP(camera, markerObj, 2).vector();
  Vector hC = hGetP(camera, markerObj, 3).vector();
  Vector hD = hGetP(camera, markerObj, 4).vector();
  Vector hAll = concatVectors(4, &hA, &hB, &hC, &hD);
  return hAll;
}

/*Vector hGetAll(const SimpleCamera& camera, const Pose3& markerPose,
                double markerSize) {

        Marker markerObj( markerPose, markerSize);

        Vector hA = hGetP(camera, markerPose, markerSize, 1).vector();
        Vector hB = hGetP(camera, markerPose, markerSize, 2).vector();
        Vector hC = hGetP(camera, markerPose, markerSize, 3).vector();
        Vector hD = hGetP(camera, markerPose, markerSize, 4).vector();

        Vector hA = hGetP(camera, markerObj, 1).vector();
        Vector hB = hGetP(camera, markerObj, 2).vector();
        Vector hC = hGetP(camera, markerObj, 3).vector();
        Vector hD = hGetP(camera, markerObj, 4).vector();
        Vector hAll = concatVectors(4, &hA, &hB, &hC, &hD);
        return hAll;
}*/

/* ************************************************************************* */
Matrix DhGetP_pose(const SimpleCamera& camera, Marker& markerObj, int num) {
  Point3 markerPoint = get_marker_point(markerObj.getmarkerSize(), num);
  Point3 worldPoint = transform_from(markerObj.getmarkerPose(), markerPoint);

  Matrix D_projection_cameraPose = Dproject_pose(camera, worldPoint);
  return D_projection_cameraPose;
}

/* ************************************************************************* */
// derivative of hGetP with respect to markerPose
Matrix DhGetP_marker(const SimpleCamera& camera, Marker& markerObj, int num) {
  Point3 markerPoint = get_marker_point(markerObj.getmarkerSize(), num);

  Point3 worldPoint = transform_from(markerObj.getmarkerPose(), markerPoint);
  Matrix D_worldPoint_markerPose =
      Dtransform_from1(markerObj.getmarkerPose(), markerPoint);

  Matrix D_projection_worldPoint = Dproject_point(camera, worldPoint);
  Matrix D_projection_markerPose =
      D_projection_worldPoint * D_worldPoint_markerPose;
  return D_projection_markerPose;
}
// derivative of hGetP with respect to cameraPose
/*
Matrix DhGetP_pose(const SimpleCamera& camera, const Pose3& markerPose,
                double markerSize, int num)
{
  Point3 markerPoint = get_marker_point(markerSize, num);
  Point3 worldPoint = transform_from(markerPose, markerPoint);

  Matrix D_projection_cameraPose = Dproject_pose(camera, worldPoint);
  return D_projection_cameraPose;
}

 *************************************************************************
// derivative of hGetP with respect to markerPose
Matrix DhGetP_marker(const SimpleCamera& camera, const Pose3& markerPose,
                double markerSize, int num)
{
  Point3 markerPoint = get_marker_point(markerSize, num);

  Point3 worldPoint = transform_from(markerPose, markerPoint);
  Matrix D_worldPoint_markerPose = Dtransform_from1(markerPose, markerPoint);

  Matrix D_projection_worldPoint = Dproject_point(camera, worldPoint);
  Matrix D_projection_markerPose = D_projection_worldPoint *
D_worldPoint_markerPose; return D_projection_markerPose;
}
*/

/* ************************************************************************* */
/*void DhGetP(const SimpleCamera& camera, const Pose3& markerPose,
                double markerSize, int num, Point2& projection,
                Matrix& D_projection_cameraPose, Matrix&
D_projection_markerPose) {

        Point3 markerPoint = get_marker_point(markerSize, num);

        Point3 worldPoint = transform_from(markerPose, markerPoint);
        Matrix D_worldPoint_markerPose = Dtransform_from1(markerPose,
markerPoint);

        // Call superduper combined derivative from SimpleCamera here...
        Matrix D_projection_worldPoint;
        Dproject_pose_point(camera, worldPoint, projection,
D_projection_cameraPose, D_projection_worldPoint);

        D_projection_markerPose = D_projection_worldPoint *
D_worldPoint_markerPose;
}*/

void DhGetP(const SimpleCamera& camera, Marker& markerObj, int num,
            Point2& projection, Matrix& D_projection_cameraPose,
            Matrix& D_projection_markerPose) {
  Point3 markerPoint = get_marker_point(markerObj.getmarkerSize(), num);

  Point3 worldPoint = transform_from(markerObj.getmarkerPose(), markerPoint);
  Matrix D_worldPoint_markerPose =
      Dtransform_from1(markerObj.getmarkerPose(), markerPoint);

  // Call superduper combined derivative from SimpleCamera here...
  Matrix D_projection_worldPoint;
  Dproject_pose_point(camera, worldPoint, projection, D_projection_cameraPose,
                      D_projection_worldPoint);

  D_projection_markerPose = D_projection_worldPoint * D_worldPoint_markerPose;
}
