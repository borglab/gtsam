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
 */

#include <gtsam/geometry/triangulation.h>

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtsam {

Vector4 triangulateHomogeneousDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
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
  int rank;
  double error;
  Vector v;
  boost::tie(rank, error, v) = DLT(A, rank_tol);

  if (rank < 3)
    throw(TriangulationUnderconstrainedException());

  return v;
}

Vector3 triangulateLOSTHomogeneous(
    const std::vector<Pose3>& poses,
    const std::vector<Point3>& calibrated_measurements, 
    const double measurement_sigma) {
  size_t m = calibrated_measurements.size();
  assert(m == poses.size());

  // Construct the system matrices.
  Matrix A = Matrix::Zero(m * 2, 3);
  Matrix b = Matrix::Zero(m * 2, 1);

  for (size_t i = 0; i < m; i++) {
    const Pose3& wTi = poses[i];
    // TODO(akshay-krishnan): are there better ways to select j?
    const int j = (i + 1) % m;
    const Pose3& wTj = poses[j];

    Point3 d_ij = wTj.translation() - wTi.translation();

    Point3 w_measurement_i = wTi.rotation().rotate(calibrated_measurements[i]);
    Point3 w_measurement_j = wTj.rotation().rotate(calibrated_measurements[j]);

    double numerator = w_measurement_i.cross(w_measurement_j).norm();
    double denominator = d_ij.cross(w_measurement_j).norm();

    double q_i = numerator / (measurement_sigma * denominator);
    Matrix23 coefficient_mat =
        q_i * skewSymmetric(calibrated_measurements[i]).topLeftCorner(2, 3) *
        wTi.rotation().matrix().transpose();

    A.row(2 * i) = coefficient_mat.row(0);
    A.row(2 * i + 1) = coefficient_mat.row(1);
    Point2 p = coefficient_mat * wTi.translation();
    b(2 * i) = p.x();
    b(2 * i + 1) = p.y();
  }
  // Solve Ax = b, using QR decomposition
  return A.colPivHouseholderQr().solve(b);
}

Vector4 triangulateHomogeneousDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const std::vector<Unit3>& measurements, double rank_tol) {

  // number of cameras
  size_t m = projection_matrices.size();

  // Allocate DLT matrix
  Matrix A = Matrix::Zero(m * 2, 4);

  for (size_t i = 0; i < m; i++) {
    size_t row = i * 2;
    const Matrix34& projection = projection_matrices.at(i);
    const Point3& p = measurements.at(i).point3(); // to get access to x,y,z of the bearing vector

    // build system of equations
    A.row(row) = p.x() * projection.row(2) - p.z() * projection.row(0);
    A.row(row + 1) = p.y() * projection.row(2) - p.z() * projection.row(1);
  }
  int rank;
  double error;
  Vector v;
  boost::tie(rank, error, v) = DLT(A, rank_tol);

  if (rank < 3)
    throw(TriangulationUnderconstrainedException());

  return v;
}

Point3 triangulateDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const Point2Vector& measurements, double rank_tol) {

  Vector4 v = triangulateHomogeneousDLT(projection_matrices, measurements,
                                        rank_tol);
  // Create 3D point from homogeneous coordinates
  return Point3(v.head<3>() / v[3]);
}

Point3 triangulateDLT(
    const std::vector<Matrix34, Eigen::aligned_allocator<Matrix34>>& projection_matrices,
    const std::vector<Unit3>& measurements, double rank_tol) {

  // contrary to previous triangulateDLT, this is now taking Unit3 inputs
  Vector4 v = triangulateHomogeneousDLT(projection_matrices, measurements,
                                         rank_tol);
   // Create 3D point from homogeneous coordinates
   return Point3(v.head<3>() / v[3]);
}

///
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

}  // \namespace gtsam
