/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TriangulationLOSTExample.cpp
 * @author Akshay Krishnan
 * @brief This example runs triangulation several times using 3 different
 * approaches: LOST, DLT, and DLT with optimization. It reports the covariance
 * and the runtime for each approach.
 *
 * @date 2022-07-10
 */
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/triangulation.h>

#include <chrono>
#include <iostream>
#include <random>
#include <optional>

using namespace std;
using namespace gtsam;

static std::mt19937 rng(42);

void PrintCovarianceStats(const Matrix& mat, const std::string& method) {
  Matrix centered = mat.rowwise() - mat.colwise().mean();
  Matrix cov = (centered.adjoint() * centered) / double(mat.rows() - 1);
  std::cout << method << " covariance: " << std::endl;
  std::cout << cov << std::endl;
  std::cout << "Trace sqrt: " << sqrt(cov.trace()) << std::endl << std::endl;
}

void PrintDuration(const std::chrono::nanoseconds dur, double num_samples,
                   const std::string& method) {
  double nanoseconds = dur.count() / num_samples;
  std::cout << "Time taken by " << method << ": " << nanoseconds * 1e-3
            << std::endl;
}

void GetLargeCamerasDataset(CameraSet<PinholeCamera<Cal3_S2>>* cameras,
                            std::vector<Pose3>* poses, Point3* point,
                            Point2Vector* measurements) {
  const double minXY = -10, maxXY = 10;
  const double minZ = -20, maxZ = 0;
  const int nrCameras = 500;
  cameras->reserve(nrCameras);
  poses->reserve(nrCameras);
  measurements->reserve(nrCameras);
  *point = Point3(0.0, 0.0, 10.0);

  std::uniform_real_distribution<double> rand_xy(minXY, maxXY);
  std::uniform_real_distribution<double> rand_z(minZ, maxZ);
  Cal3_S2 identityK;

  for (int i = 0; i < nrCameras; ++i) {
    Point3 wti(rand_xy(rng), rand_xy(rng), rand_z(rng));
    Pose3 wTi(Rot3(), wti);

    poses->push_back(wTi);
    cameras->emplace_back(wTi, identityK);
    measurements->push_back(cameras->back().project(*point));
  }
}

void GetSmallCamerasDataset(CameraSet<PinholeCamera<Cal3_S2>>* cameras,
                            std::vector<Pose3>* poses, Point3* point,
                            Point2Vector* measurements) {
  Pose3 pose1;
  Pose3 pose2(Rot3(), Point3(5., 0., -5.));
  Cal3_S2 identityK;
  PinholeCamera<Cal3_S2> camera1(pose1, identityK);
  PinholeCamera<Cal3_S2> camera2(pose2, identityK);

  *point = Point3(0, 0, 1);
  cameras->push_back(camera1);
  cameras->push_back(camera2);
  *poses = {pose1, pose2};
  *measurements = {camera1.project(*point), camera2.project(*point)};
}

Point2Vector AddNoiseToMeasurements(const Point2Vector& measurements,
                                    const double measurementSigma) {
  std::normal_distribution<double> normal(0.0, measurementSigma);

  Point2Vector noisyMeasurements;
  noisyMeasurements.reserve(measurements.size());
  for (const auto& p : measurements) {
    noisyMeasurements.emplace_back(p.x() + normal(rng), p.y() + normal(rng));
  }
  return noisyMeasurements;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  CameraSet<PinholeCamera<Cal3_S2>> cameras;
  std::vector<Pose3> poses;
  Point3 landmark;
  Point2Vector measurements;
  GetLargeCamerasDataset(&cameras, &poses, &landmark, &measurements);
  // GetSmallCamerasDataset(&cameras, &poses, &landmark, &measurements);
  const double measurementSigma = 1e-2;
  SharedNoiseModel measurementNoise =
      noiseModel::Isotropic::Sigma(2, measurementSigma);

  const long int nrTrials = 1000;
  Matrix errorsDLT = Matrix::Zero(nrTrials, 3);
  Matrix errorsLOST = Matrix::Zero(nrTrials, 3);
  Matrix errorsDLTOpt = Matrix::Zero(nrTrials, 3);

  double rank_tol = 1e-9;
  std::shared_ptr<Cal3_S2> calib = std::make_shared<Cal3_S2>();
  std::chrono::nanoseconds durationDLT;
  std::chrono::nanoseconds durationDLTOpt;
  std::chrono::nanoseconds durationLOST;

  for (int i = 0; i < nrTrials; i++) {
    Point2Vector noisyMeasurements =
        AddNoiseToMeasurements(measurements, measurementSigma);

    auto lostStart = std::chrono::high_resolution_clock::now();
    auto estimateLOST = triangulatePoint3<Cal3_S2>(
        cameras, noisyMeasurements, rank_tol, false, measurementNoise, true);
    durationLOST += std::chrono::high_resolution_clock::now() - lostStart;

    auto dltStart = std::chrono::high_resolution_clock::now();
    auto estimateDLT = triangulatePoint3<Cal3_S2>(
        cameras, noisyMeasurements, rank_tol, false, measurementNoise, false);
    durationDLT += std::chrono::high_resolution_clock::now() - dltStart;

    auto dltOptStart = std::chrono::high_resolution_clock::now();
    auto estimateDLTOpt = triangulatePoint3<Cal3_S2>(
        cameras, noisyMeasurements, rank_tol, true, measurementNoise, false);
    durationDLTOpt += std::chrono::high_resolution_clock::now() - dltOptStart;

    errorsLOST.row(i) = estimateLOST - landmark;
    errorsDLT.row(i) = estimateDLT - landmark;
    errorsDLTOpt.row(i) = estimateDLTOpt - landmark;
  }
  PrintCovarianceStats(errorsLOST, "LOST");
  PrintCovarianceStats(errorsDLT, "DLT");
  PrintCovarianceStats(errorsDLTOpt, "DLT_OPT");

  PrintDuration(durationLOST, nrTrials, "LOST");
  PrintDuration(durationDLT, nrTrials, "DLT");
  PrintDuration(durationDLTOpt, nrTrials, "DLT_OPT");
}
