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

/**
 * DLT triangulation: See Hartley and Zisserman, 2nd Ed., page 312
 * @param projection_matrices Projection matrices (K*P^-1)
 * @param measurements 2D measurements
 * @param rank_tol SVD rank tolerance
 * @return Triangulated Point3
 */
Point3 triangulateDLT(const std::vector<Matrix>& projection_matrices,
    const std::vector<Point2>& measurements, double rank_tol) {

  // number of cameras
  size_t m = projection_matrices.size();

  // Allocate DLT matrix
  Matrix A = zeros(m * 2, 4);

  for (size_t i = 0; i < m; i++) {
    size_t row = i * 2;
    const Matrix& projection = projection_matrices.at(i);
    const Point2& p = measurements.at(i);

    // build system of equations
    A.row(row) = p.x() * projection.row(2) - projection.row(0);
    A.row(row + 1) = p.y() * projection.row(2) - projection.row(1);
  }
  int rank;
  double error;
  Vector v;
  boost::tie(rank, error, v) = DLT(A, rank_tol);
  //  std::cout << "s " << s.transpose() << std:endl;

  if (rank < 3)
    throw(TriangulationUnderconstrainedException());

  // Create 3D point from eigenvector
  return Point3(sub((v / v(3)), 0, 3));
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


} // \namespace gtsam

