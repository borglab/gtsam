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
  const double min_xy = -10, max_xy = 10;
  const double min_z = -20, max_z = 0;
  const int num_cameras = 500;
  cameras->reserve(num_cameras);
  poses->reserve(num_cameras);
  measurements->reserve(num_cameras);
  *point = Point3(0.0, 0.0, 10.0);

  std::uniform_real_distribution<double> rand_xy(min_xy, max_xy);
  std::uniform_real_distribution<double> rand_z(min_z, max_z);
  Cal3_S2 identity_K;

  for (int i = 0; i < num_cameras; ++i) {
    Point3 wti(rand_xy(rng), rand_xy(rng), rand_z(rng));
    Pose3 wTi(Rot3(), wti);

    poses->push_back(wTi);
    cameras->emplace_back(wTi, identity_K);
    measurements->push_back(cameras->back().project(*point));
  }
}

void GetSmallCamerasDataset(CameraSet<PinholeCamera<Cal3_S2>>* cameras,
                            std::vector<Pose3>* poses, Point3* point,
                            Point2Vector* measurements) {
  Pose3 pose_1;
  Pose3 pose_2(Rot3(), Point3(5., 0., -5.));
  Cal3_S2 identity_K;
  PinholeCamera<Cal3_S2> camera_1(pose_1, identity_K);
  PinholeCamera<Cal3_S2> camera_2(pose_2, identity_K);

  *point = Point3(0, 0, 1);
  cameras->push_back(camera_1);
  cameras->push_back(camera_2);
  *poses = {pose_1, pose_2};
  *measurements = {camera_1.project(*point), camera_2.project(*point)};
}

Point2Vector AddNoiseToMeasurements(const Point2Vector& measurements,
                                    const double measurement_sigma) {
  std::normal_distribution<double> normal(0.0, measurement_sigma);

  Point2Vector noisy_measurements;
  noisy_measurements.reserve(measurements.size());
  for (const auto& p : measurements) {
    noisy_measurements.emplace_back(p.x() + normal(rng), p.y() + normal(rng));
  }
  return noisy_measurements;
}

/* ************************************************************************* */
int main(int argc, char* argv[]) {
  CameraSet<PinholeCamera<Cal3_S2>> cameras;
  std::vector<Pose3> poses;
  Point3 gt_point;
  Point2Vector measurements;
  GetLargeCamerasDataset(&cameras, &poses, &gt_point, &measurements);
  // GetSmallCamerasDataset(&cameras, &poses, &gt_point, &measurements);
  const double measurement_sigma = 1e-2;
  SharedNoiseModel measurement_noise =
      noiseModel::Isotropic::Sigma(2, measurement_sigma);

  const long int num_trials = 1000;
  Matrix dlt_errors = Matrix::Zero(num_trials, 3);
  Matrix lost_errors = Matrix::Zero(num_trials, 3);
  Matrix dlt_opt_errors = Matrix::Zero(num_trials, 3);

  double rank_tol = 1e-9;
  boost::shared_ptr<Cal3_S2> calib = boost::make_shared<Cal3_S2>();
  std::chrono::nanoseconds dlt_duration;
  std::chrono::nanoseconds dlt_opt_duration;
  std::chrono::nanoseconds lost_duration;
  std::chrono::nanoseconds lost_tls_duration;

  for (int i = 0; i < num_trials; i++) {
    Point2Vector noisy_measurements =
        AddNoiseToMeasurements(measurements, measurement_sigma);

    auto lost_start = std::chrono::high_resolution_clock::now();
    boost::optional<Point3> estimate_lost = triangulatePoint3<Cal3_S2>(
        cameras, noisy_measurements, rank_tol, false, measurement_noise, true);
    lost_duration += std::chrono::high_resolution_clock::now() - lost_start;

    auto dlt_start = std::chrono::high_resolution_clock::now();
    boost::optional<Point3> estimate_dlt = triangulatePoint3<Cal3_S2>(
        cameras, noisy_measurements, rank_tol, false, measurement_noise, false);
    dlt_duration += std::chrono::high_resolution_clock::now() - dlt_start;

    auto dlt_opt_start = std::chrono::high_resolution_clock::now();
    boost::optional<Point3> estimate_dlt_opt = triangulatePoint3<Cal3_S2>(
        cameras, noisy_measurements, rank_tol, true, measurement_noise, false);
    dlt_opt_duration +=
        std::chrono::high_resolution_clock::now() - dlt_opt_start;

    lost_errors.row(i) = *estimate_lost - gt_point;
    dlt_errors.row(i) = *estimate_dlt - gt_point;
    dlt_opt_errors.row(i) = *estimate_dlt_opt - gt_point;
  }
  PrintCovarianceStats(lost_errors, "LOST");
  PrintCovarianceStats(dlt_errors, "DLT");
  PrintCovarianceStats(dlt_opt_errors, "DLT_OPT");

  PrintDuration(lost_duration, num_trials, "LOST");
  PrintDuration(dlt_duration, num_trials, "DLT");
  PrintDuration(dlt_opt_duration, num_trials, "DLT_OPT");
}