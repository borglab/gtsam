/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file triangulation.h
 * @brief Functions for triangulation
 * @date July 31, 2013
 * @author Chris Beall
 * @author Akshay Krishnan
 */

#include <gtsam/geometry/triangulation.h>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

Vector4 triangulateHomogeneousDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>&
        projection_matrices,
    const Point2Vector& measurements, double rank_tol) {
  // number of cameras
  size_t m = projection_matrices.size();

  // Allocate DLT matrix
  Matrix A = Matrix::Zero(m * 2, 4);

  for (size_t i = 0; i < m; i++) {
    size_t row = i * 2;
    const Matrix34& projection = projection_matrices.at(i);
    const Point2& p = measurements.at(i);

    // build system of equations
    // [A_1; A_2; A_3] x = [b_1; b_2; b_3]
    // [b_3 * A_1 - b_1 * A_3] x = 0
    // [b_3 * A_2 - b_2 * A_3] x = 0
    // A' x = 0
    // A' 2x4 = [b_3 * A_1 - b_1 * A_3; b_3 * A_2 - b_2 * A_3]
    A.row(row) = p.x() * projection.row(2) - projection.row(0);
    A.row(row + 1) = p.y() * projection.row(2) - projection.row(1);
  }
  const auto [rank, error, v] = DLT(A, rank_tol);

  if (rank < 3) throw(TriangulationUnderconstrainedException());

  return v;
}

Vector4 triangulateHomogeneousDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>&
        projection_matrices,
    const std::vector<Unit3>& measurements, double rank_tol) {
  // number of cameras
  size_t m = projection_matrices.size();

  // Allocate DLT matrix
  Matrix A = Matrix::Zero(m * 2, 4);

  for (size_t i = 0; i < m; i++) {
    size_t row = i * 2;
    const Matrix34& projection = projection_matrices.at(i);
    const Point3& p =
        measurements.at(i)
            .point3();  // to get access to x,y,z of the bearing vector

    // build system of equations
    A.row(row) = p.x() * projection.row(2) - p.z() * projection.row(0);
    A.row(row + 1) = p.y() * projection.row(2) - p.z() * projection.row(1);
  }
  const auto [rank, error, v] = DLT(A, rank_tol);

  if (rank < 3) throw(TriangulationUnderconstrainedException());

  return v;
}

Point3 triangulateLOST(const std::vector<Pose3>& poses,
                       const Point3Vector& calibratedMeasurements,
                       const SharedIsotropic& measurementNoise,
                       double rank_tol) {
  size_t m = calibratedMeasurements.size();
  assert(m == poses.size());

  // Construct the system matrices.
  Matrix A = Matrix::Zero(m * 2, 3);
  Matrix b = Matrix::Zero(m * 2, 1);

  for (size_t i = 0; i < m; i++) {
    const Pose3& wTi = poses[i];
    // TODO(akshay-krishnan): are there better ways to select j?
    int j = (i + 1) % m;
    const Pose3& wTj = poses[j];

    Point3 d_ij = wTj.translation() - wTi.translation();
    Point3 wZi = wTi.rotation().rotate(calibratedMeasurements[i]);
    Point3 wZj = wTj.rotation().rotate(calibratedMeasurements[j]);
    double num_i = wZi.cross(wZj).norm();
    double den_i = d_ij.cross(wZj).norm();

    // Handle q_i = 0 (or NaN), which arises if the measurement vectors, wZi and 
    // wZj, coincide (or the baseline vector coincides with the jth measurement 
    // vector).
    if (num_i == 0 || den_i == 0) {
      bool success = false;
      for (size_t k = 2; k < m; k++) {
        j = (i + k) % m;
        const Pose3& wTj = poses[j];

        d_ij = wTj.translation() - wTi.translation();
        wZj = wTj.rotation().rotate(calibratedMeasurements[j]);
        num_i = wZi.cross(wZj).norm();
        den_i = d_ij.cross(wZj).norm();
        if (num_i > 0 && den_i > 0) {
          success = true;
          break;
        }
      }
      if (!success) throw(TriangulationUnderconstrainedException());
    }

    // Note: Setting q_i = 1.0 gives same results as DLT.
    const double q_i = num_i / (measurementNoise->sigma() * den_i);

    const Matrix23 coefficientMat =
        q_i * skewSymmetric(calibratedMeasurements[i]).topLeftCorner(2, 3) *
        wTi.rotation().matrix().transpose();

    A.block<2, 3>(2 * i, 0) << coefficientMat;
    b.block<2, 1>(2 * i, 0) << coefficientMat * wTi.translation();
  }

  Eigen::ColPivHouseholderQR<Matrix> A_Qr = A.colPivHouseholderQr();
  A_Qr.setThreshold(rank_tol);

  if (A_Qr.rank() < 3) throw(TriangulationUnderconstrainedException());

  return A_Qr.solve(b);
}

Point3 triangulateDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>&
        projection_matrices,
    const Point2Vector& measurements, double rank_tol) {
  Vector4 v = 
      triangulateHomogeneousDLT(projection_matrices, measurements, rank_tol);
  // Create 3D point from homogeneous coordinates
  return Point3(v.head<3>() / v[3]);
}

Point3 triangulateDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& 
    projection_matrices,
    const std::vector<Unit3>& measurements, double rank_tol) {
  // contrary to previous triangulateDLT, this is now taking Unit3 inputs
  Vector4 v =
      triangulateHomogeneousDLT(projection_matrices, measurements, rank_tol);
  // Create 3D point from homogeneous coordinates
  return Point3(v.head<3>() / v[3]);
}

/**
 * Optimize for triangulation
 * @param graph nonlinear factors for projection
 * @param values initial values
 * @param landmarkKey to refer to landmark
 * @return refined Point3
 */
Point3 optimize(const NonlinearFactorGraph& graph, const Values& values,
                Key landmarkKey) {
  // Maybe we should consider Gauss-Newton?
  LevenbergMarquardtParams params;
  params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
  params.verbosity = NonlinearOptimizerParams::ERROR;
  params.lambdaInitial = 1;
  params.lambdaFactor = 10;
  params.maxIterations = 100;
  params.absoluteErrorTol = 1.0;
  params.verbosityLM = LevenbergMarquardtParams::SILENT;
  params.verbosity = NonlinearOptimizerParams::SILENT;
  params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;

  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();

  return result.at<Point3>(landmarkKey);
}

}  // namespace gtsam
