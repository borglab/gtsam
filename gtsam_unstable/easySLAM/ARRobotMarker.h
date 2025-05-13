/************************************************************************************
ARRobotMarker class,
Author: Alireza Fathi
Date: 10/08/2008
This class is created to keep the information loaded from ARToolkit.
ARToolKit takes the images and returns the marker numbers in each frame,
and their 4 corners in this order:

M2  M3

M1  M4

************************************************************************************/
#pragma once

#include <gtsam/SimpleCamera.h>

class ARRobotMarker {
 private:            // TODO: should all be const
  int markerNumber;  // The marker number
  int robotNumber;   // The Robot number, usually the frame number if one robot.
  std::vector<Vector> Points;  // The measurements (TODO: should be Point2s)
  double markerSize_;          // marker size
  gtsam::Cal3_S2 K_;           // Camera calibration
  gtsam::Pose3
      estimatedPose_;  // Temporary pose for estimate, FD: fix this hack

 public:
  // constructors

  /**
   * default constructor, all measurement are zero
   * FD says: dangerous, obsolete ??
   */
  ARRobotMarker();

  /**
   * constructor that takes the 4 measurements in order
   * @param i robot or camera pose index
   * @param j marker index
   */
  ARRobotMarker(int i, int j, Vector p1, Vector p2, Vector p3, Vector p4,
                double mSize, gtsam::Cal3_S2 K);

  /**
   * constructor that takes the 8 doubles in order
   */
  ARRobotMarker(int i, int j, double xa, double ya, double xb, double yb,
                double xc, double yc, double xd, double yd, double mSize,
                gtsam::Cal3_S2 K);

  /**
   * constructor for creating ground truth measurements
   */
  ARRobotMarker(int i, int j, gtsam::Pose3& pose, gtsam::Pose3& marker,
                double mSize, gtsam::Cal3_S2 K);

  /**
   * Camera version. TODO: always use this one, kill previous one
   */
  ARRobotMarker(int i, int j, gtsam::SimpleCamera& camera, gtsam::Pose3& marker,
                double mSize);

  // get and set functions

  void setRobotNumber(int i) { robotNumber = i; }
  int getRobotNumber() const { return robotNumber; }

  void setMarkerNumber(int j) { markerNumber = j; }
  int getMarkerNumber() const { return markerNumber; }

  void setPoint(int i, const Vector p) { Points[i - 1] = p; }
  const Vector& getPoint(int i) const { return Points[i - 1]; }
  std::vector<Vector> getPoints() { return Points; }

  double getMarkerSize() const { return markerSize_; }

  gtsam::Cal3_S2 getCalibration() const { return K_; }

  void setEstimatedPose(gtsam::Pose3& p) { estimatedPose_ = p; }  // huh ?
  gtsam::Pose3 getEstimatedPose() const { return estimatedPose_; }

  double getArea() const;
  bool isConvex() const;

  /**
   * ARToolKit doesn't return the right odering of points. we are
   * going to fix the ordering here.
   * TODO: FD says imperative is bad !!
   */
  int fixOrderingOfPoints();

  void print() const;

  /**
   * return 8D measurement vector
   */
  Vector measurement() const;

  /**
   * compute homography from marker to image
   */
  Matrix estimate_H();

  /**
   * compute pose cTm from marker to camera
   */
  void estimate_pose();

  bool equals(ARRobotMarker& m);
};

/* ************************************************************************* */
/**
 * load markers from file
 */
std::vector<ARRobotMarker*> loadARToolKit(std::string path, int nrFrames,
                                          double DX = 0.0, double DY = 0.0);
/* ************************************************************************* */
void saveARToolKit(std::string path, std::vector<ARRobotMarker*> markers);
/* ************************************************************************* */
std::vector<ARRobotMarker*> loadARToolKit_oneFrame(std::string path,
                                                   int frameNumber,
                                                   double DX = 0.0,
                                                   double DY = 0.0);
/* ************************************************************************* */
void dumpAR(const std::string& path, std::vector<ARRobotMarker*> markers);
/* ************************************************************************* */
std::vector<ARRobotMarker*> load_dumpedAR(const std::string& path);
/* ************************************************************************* */
/**
 * Among a set of markers returns the one with the maximum area
 * previousMarker is the marker that was reference in the previous frame,
 * if set to -1, it ignores it.
 */
int getReferenceMarker(std::vector<ARRobotMarker*> markers, int previousMarker);
