/**
 * @file    Marker.h
 * @brief   It containts the characteristics of our markers.
 * @author  Alireza Fathi
 */

#pragma once

#include <gtsam/SimpleCamera.h>

class Marker {
  gtsam::Pose3* markerPose_;
  double markerSize_;

 public:
  Marker(const gtsam::Pose3 markerPose, double markerSize) {
    markerPose_ = new gtsam::Pose3(markerPose);
    markerSize_ = markerSize;
  }

  const double getmarkerSize(void) { return markerSize_; }

  const gtsam::Pose3 getmarkerPose(void) { return (*markerPose_); }

  ~Marker() { delete markerPose_; }
};

/**
 * returns the location of a point (from the 4 corners of the marker)
 * in the marker coordinate frame.
 */
gtsam::Point3 get_marker_point(const double markerSize, const int whichPoint);

/* ************************************************************************* */
// measurement functions
/* ************************************************************************* */

/**
 * For the given camera and marker pose, return the location a point
 * is supposed to appear in the image
 * @param camera
 * @param markerPose
 * @param markerSize
 * @param num selects one of the 4 points on the marker, as follows:
 * 		1: left,  bottom       2-------3
 * 		2: left,  top          |       |
 * 		3: right, top          |       |
 * 		4: right, bottom       1-------4
 */

gtsam::Point2 hGetP(const gtsam::SimpleCamera& camera, Marker& markerObj,
                    int num);

/*
gtsam::Point2 hGetP(const gtsam::SimpleCamera& camera,
                const gtsam::Pose3& markerPose, double markerSize, int num);
*/

/**
 * Convenience version that returns all 4 in a vector
 */
Vector hGetAll(const gtsam::SimpleCamera& camera, Marker& markerObj);

/*
Vector hGetAll(const gtsam::SimpleCamera& camera,
                const gtsam::Pose3& markerPose, double markerSize);
*/

/**
 * Derivatives of hGetP.
 * @param num selects one of the 4 points on the marker
 */
/*
Matrix DhGetP_pose(const gtsam::SimpleCamera&camera,
                const gtsam::Pose3& markerPose, double markerSize, int num);
Matrix DhGetP_marker(const gtsam::SimpleCamera& camera,
                const gtsam::Pose3& markerPose, double markerSize, int num);
*/

Matrix DhGetP_pose(const gtsam::SimpleCamera& camera, Marker& markerObj,
                   int num);
Matrix DhGetP_marker(const gtsam::SimpleCamera& camera, Marker& markerObj,
                     int num);

/**
 * super-duper combined evaluation + derivatives
 * saves a lot of time because a lot of computation is shared
 * @param num selects one of the 4 points on the marker
 */
/*
void DhGetP(const gtsam::SimpleCamera& camera, const gtsam::Pose3& markerPose,
                double markerSize, int num, gtsam::Point2& projection,
                Matrix& D_projection_cameraPose, Matrix&
D_projection_markerPose);
*/
void DhGetP(const gtsam::SimpleCamera& camera, Marker& markerObj, int num,
            gtsam::Point2& projection, Matrix& D_projection_cameraPose,
            Matrix& D_projection_markerPose);
