/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SfmData.cpp
 * @date January 2022
 * @author Frank dellaert
 * @brief Data structure for dealing with Structure from Motion data
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/GeneralSFMFactor.h>

#include <fstream>
#include <iostream>

namespace gtsam {

using std::cout;
using std::endl;

using gtsam::symbol_shorthand::P;

/* ************************************************************************** */
void SfmData::print(const std::string &s) const {
  std::cout << "Number of cameras = " << cameras.size() << std::endl;
  std::cout << "Number of tracks = " << tracks.size() << std::endl;
}

/* ************************************************************************** */
bool SfmData::equals(const SfmData &sfmData, double tol) const {
  // check number of cameras and tracks
  if (cameras.size() != sfmData.cameras.size() ||
      tracks.size() != sfmData.tracks.size()) {
    return false;
  }

  // check each camera
  for (size_t i = 0; i < cameras.size(); ++i) {
    if (!camera(i).equals(sfmData.camera(i), tol)) {
      return false;
    }
  }

  // check each track
  for (size_t j = 0; j < tracks.size(); ++j) {
    if (!track(j).equals(sfmData.track(j), tol)) {
      return false;
    }
  }

  return true;
}

/* ************************************************************************* */
Rot3 openGLFixedRotation() {  // this is due to different convention for
                              // cameras in gtsam and openGL
  /* R = [ 1   0   0
   *       0  -1   0
   *       0   0  -1]
   */
  Matrix3 R_mat = Matrix3::Zero(3, 3);
  R_mat(0, 0) = 1.0;
  R_mat(1, 1) = -1.0;
  R_mat(2, 2) = -1.0;
  return Rot3(R_mat);
}

/* ************************************************************************* */
Pose3 openGL2gtsam(const Rot3 &R, double tx, double ty, double tz) {
  Rot3 R90 = openGLFixedRotation();
  Rot3 wRc = (R.inverse()).compose(R90);

  // Our camera-to-world translation wTc = -R'*t
  return Pose3(wRc, R.unrotate(Point3(-tx, -ty, -tz)));
}

/* ************************************************************************* */
Pose3 gtsam2openGL(const Rot3 &R, double tx, double ty, double tz) {
  Rot3 R90 = openGLFixedRotation();
  Rot3 cRw_openGL = R90.compose(R.inverse());
  Point3 t_openGL = cRw_openGL.rotate(Point3(-tx, -ty, -tz));
  return Pose3(cRw_openGL, t_openGL);
}

/* ************************************************************************* */
Pose3 gtsam2openGL(const Pose3 &PoseGTSAM) {
  return gtsam2openGL(PoseGTSAM.rotation(), PoseGTSAM.x(), PoseGTSAM.y(),
                      PoseGTSAM.z());
}

/* ************************************************************************** */
SfmData SfmData::FromBundlerFile(const std::string &filename) {
  // Load the data file
  std::ifstream is(filename.c_str(), std::ifstream::in);
  if (!is) {
    throw std::runtime_error(
        "Error in FromBundlerFile: can not find the file!!");
  }

  SfmData sfmData;

  // Ignore the first line
  char aux[500];
  is.getline(aux, 500);

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints;
  is >> nrPoses >> nrPoints;

  // Get the information for the camera poses
  for (size_t i = 0; i < nrPoses; i++) {
    // Get the focal length and the radial distortion parameters
    float f, k1, k2;
    is >> f >> k1 >> k2;
    Cal3Bundler K(f, k1, k2);

    // Get the rotation matrix
    float r11, r12, r13;
    float r21, r22, r23;
    float r31, r32, r33;
    is >> r11 >> r12 >> r13 >> r21 >> r22 >> r23 >> r31 >> r32 >> r33;

    // Bundler-OpenGL rotation matrix
    Rot3 R(r11, r12, r13, r21, r22, r23, r31, r32, r33);

    // Check for all-zero R, in which case quit
    if (r11 == 0 && r12 == 0 && r13 == 0) {
      throw std::runtime_error(
          "Error in FromBundlerFile: zero rotation matrix");
    }

    // Get the translation vector
    float tx, ty, tz;
    is >> tx >> ty >> tz;

    Pose3 pose = openGL2gtsam(R, tx, ty, tz);

    sfmData.cameras.emplace_back(pose, K);
  }

  // Get the information for the 3D points
  sfmData.tracks.reserve(nrPoints);
  for (size_t j = 0; j < nrPoints; j++) {
    SfmTrack track;

    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    track.p = Point3(x, y, z);

    // Get the color information
    float r, g, b;
    is >> r >> g >> b;
    track.r = r / 255.f;
    track.g = g / 255.f;
    track.b = b / 255.f;

    // Now get the visibility information
    size_t nvisible = 0;
    is >> nvisible;

    track.measurements.reserve(nvisible);
    track.siftIndices.reserve(nvisible);
    for (size_t k = 0; k < nvisible; k++) {
      size_t cam_idx = 0, point_idx = 0;
      float u, v;
      is >> cam_idx >> point_idx >> u >> v;
      track.measurements.emplace_back(cam_idx, Point2(u, -v));
      track.siftIndices.emplace_back(cam_idx, point_idx);
    }

    sfmData.tracks.push_back(track);
  }

  return sfmData;
}

/* ************************************************************************** */
SfmData SfmData::FromBalFile(const std::string &filename) {
  // Load the data file
  std::ifstream is(filename.c_str(), std::ifstream::in);
  if (!is) {
    throw std::runtime_error("Error in FromBalFile: can not find the file!!");
  }

  SfmData sfmData;

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints, nrObservations;
  is >> nrPoses >> nrPoints >> nrObservations;

  sfmData.tracks.resize(nrPoints);

  // Get the information for the observations
  for (size_t k = 0; k < nrObservations; k++) {
    size_t i = 0, j = 0;
    float u, v;
    is >> i >> j >> u >> v;
    sfmData.tracks[j].measurements.emplace_back(i, Point2(u, -v));
  }

  // Get the information for the camera poses
  for (size_t i = 0; i < nrPoses; i++) {
    // Get the Rodrigues vector
    float wx, wy, wz;
    is >> wx >> wy >> wz;
    Rot3 R = Rot3::Rodrigues(wx, wy, wz);  // BAL-OpenGL rotation matrix

    // Get the translation vector
    float tx, ty, tz;
    is >> tx >> ty >> tz;

    Pose3 pose = openGL2gtsam(R, tx, ty, tz);

    // Get the focal length and the radial distortion parameters
    float f, k1, k2;
    is >> f >> k1 >> k2;
    Cal3Bundler K(f, k1, k2);

    sfmData.cameras.emplace_back(pose, K);
  }

  // Get the information for the 3D points
  for (size_t j = 0; j < nrPoints; j++) {
    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    SfmTrack &track = sfmData.tracks[j];
    track.p = Point3(x, y, z);
    track.r = 0.4f;
    track.g = 0.4f;
    track.b = 0.4f;
  }

  return sfmData;
}

/* ************************************************************************** */
bool writeBAL(const std::string &filename, const SfmData &data) {
  // Open the output file
  std::ofstream os;
  os.open(filename.c_str());
  os.precision(20);
  if (!os.is_open()) {
    cout << "Error in writeBAL: can not open the file!!" << endl;
    return false;
  }

  // Write the number of camera poses and 3D points
  size_t nrObservations = 0;
  for (size_t j = 0; j < data.tracks.size(); j++) {
    nrObservations += data.tracks[j].numberMeasurements();
  }

  // Write observations
  os << data.cameras.size() << " " << data.tracks.size() << " "
     << nrObservations << endl;
  os << endl;

  for (size_t j = 0; j < data.tracks.size(); j++) {  // for each 3D point j
    const SfmTrack &track = data.tracks[j];

    for (size_t k = 0; k < track.numberMeasurements();
         k++) {  // for each observation of the 3D point j
      size_t i = track.measurements[k].first;  // camera id
      double u0 = data.cameras[i].calibration().px();
      double v0 = data.cameras[i].calibration().py();

      if (u0 != 0 || v0 != 0) {
        cout << "writeBAL has not been tested for calibration with nonzero "
                "(u0,v0)"
             << endl;
      }

      double pixelBALx = track.measurements[k].second.x() -
                         u0;  // center of image is the origin
      double pixelBALy = -(track.measurements[k].second.y() -
                           v0);  // center of image is the origin
      Point2 pixelMeasurement(pixelBALx, pixelBALy);
      os << i /*camera id*/ << " " << j /*point id*/ << " "
         << pixelMeasurement.x() /*u of the pixel*/ << " "
         << pixelMeasurement.y() /*v of the pixel*/ << endl;
    }
  }
  os << endl;

  // Write cameras
  for (size_t i = 0; i < data.cameras.size(); i++) {  // for each camera
    Pose3 poseGTSAM = data.cameras[i].pose();
    Cal3Bundler cameraCalibration = data.cameras[i].calibration();
    Pose3 poseOpenGL = gtsam2openGL(poseGTSAM);
    os << Rot3::Logmap(poseOpenGL.rotation()) << endl;
    os << poseOpenGL.translation().x() << endl;
    os << poseOpenGL.translation().y() << endl;
    os << poseOpenGL.translation().z() << endl;
    os << cameraCalibration.fx() << endl;
    os << cameraCalibration.k1() << endl;
    os << cameraCalibration.k2() << endl;
    os << endl;
  }

  // Write the points
  for (size_t j = 0; j < data.tracks.size(); j++) {  // for each 3D point j
    Point3 point = data.tracks[j].p;
    os << point.x() << endl;
    os << point.y() << endl;
    os << point.z() << endl;
    os << endl;
  }

  os.close();
  return true;
}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V42
bool readBundler(const std::string &filename, SfmData &data) {
  try {
    data = SfmData::FromBundlerFile(filename);
    return true;
  } catch (const std::exception & /* e */) {
    return false;
  }
}
bool readBAL(const std::string &filename, SfmData &data) {
  try {
    data = SfmData::FromBalFile(filename);
    return true;
  } catch (const std::exception & /* e */) {
    return false;
  }
}
#endif

SfmData readBal(const std::string &filename) {
  return SfmData::FromBalFile(filename);
}

/* ************************************************************************** */
bool writeBALfromValues(const std::string &filename, const SfmData &data,
                        const Values &values) {
  using Camera = PinholeCamera<Cal3Bundler>;
  SfmData dataValues = data;

  // Store poses or cameras in SfmData
  size_t nrPoses = values.count<Pose3>();
  if (nrPoses == dataValues.cameras.size()) {  // we only estimated camera poses
    for (size_t i = 0; i < dataValues.cameras.size(); i++) {  // for each camera
      Pose3 pose = values.at<Pose3>(i);
      Cal3Bundler K = dataValues.cameras[i].calibration();
      Camera camera(pose, K);
      dataValues.cameras[i] = camera;
    }
  } else {
    size_t nrCameras = values.count<Camera>();
    if (nrCameras == dataValues.cameras.size()) {  // we only estimated camera
                                                   // poses and calibration
      for (size_t i = 0; i < nrCameras; i++) {     // for each camera
        Key cameraKey = i;                         // symbol('c',i);
        Camera camera = values.at<Camera>(cameraKey);
        dataValues.cameras[i] = camera;
      }
    } else {
      cout << "writeBALfromValues: different number of cameras in "
              "SfM_dataValues (#cameras "
           << dataValues.cameras.size() << ") and values (#cameras " << nrPoses
           << ", #poses " << nrCameras << ")!!" << endl;
      return false;
    }
  }

  // Store 3D points in SfmData
  size_t nrPoints = values.count<Point3>(), nrTracks = dataValues.tracks.size();
  if (nrPoints != nrTracks) {
    cout << "writeBALfromValues: different number of points in "
            "SfM_dataValues (#points= "
         << nrTracks << ") and values (#points " << nrPoints << ")!!" << endl;
  }

  for (size_t j = 0; j < nrTracks; j++) {  // for each point
    Key pointKey = P(j);
    if (values.exists(pointKey)) {
      Point3 point = values.at<Point3>(pointKey);
      dataValues.tracks[j].p = point;
    } else {
      dataValues.tracks[j].r = 1.0;
      dataValues.tracks[j].g = 0.0;
      dataValues.tracks[j].b = 0.0;
      dataValues.tracks[j].p = Point3(0, 0, 0);
    }
  }

  // Write SfmData to file
  return writeBAL(filename, dataValues);
}

/* ************************************************************************** */
NonlinearFactorGraph SfmData::generalSfmFactors(
    const SharedNoiseModel &model) const {
  using ProjectionFactor = GeneralSFMFactor<SfmCamera, Point3>;
  NonlinearFactorGraph factors;

  size_t j = 0;
  for (const SfmTrack &track : tracks) {
    for (const SfmMeasurement &m : track.measurements) {
      size_t i = m.first;
      Point2 uv = m.second;
      factors.emplace_shared<ProjectionFactor>(uv, model, i, P(j));
    }
    j += 1;
  }

  return factors;
}

/* ************************************************************************** */
NonlinearFactorGraph SfmData::sfmFactorGraph(
    const SharedNoiseModel &model, std::optional<size_t> fixedCamera,
    std::optional<size_t> fixedPoint) const {
  NonlinearFactorGraph graph = generalSfmFactors(model);
  using noiseModel::Constrained;
  if (fixedCamera) {
    graph.addPrior(*fixedCamera, cameras[0], Constrained::All(9));
  }
  if (fixedPoint) {
    graph.addPrior(P(*fixedPoint), tracks[0].p, Constrained::All(3));
  }
  return graph;
}

/* ************************************************************************** */
Values initialCamerasEstimate(const SfmData &db) {
  Values initial;
  size_t i = 0;  // NO POINTS:  j = 0;
  for (const SfmCamera &camera : db.cameras) initial.insert(i++, camera);
  return initial;
}

/* ************************************************************************** */
Values initialCamerasAndPointsEstimate(const SfmData &db) {
  Values initial;
  size_t i = 0, j = 0;
  for (const SfmCamera &camera : db.cameras) initial.insert(i++, camera);
  for (const SfmTrack &track : db.tracks) initial.insert(P(j++), track.p);
  return initial;
}

/* ************************************************************************** */

}  // namespace gtsam
