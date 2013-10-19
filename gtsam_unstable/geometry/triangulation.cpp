/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file triangulation.cpp
 * @brief Functions for triangulation
 * @author Chris Beall
 */

#include <gtsam_unstable/geometry/triangulation.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/slam/ProjectionFactor.h>

using namespace std;
using namespace boost::assign;

namespace gtsam {

using symbol_shorthand::X;
using symbol_shorthand::L;

typedef GenericProjectionFactor<Pose3, Point3, Cal3_S2> ProjectionFactor;
typedef PriorFactor<Pose3> Pose3Prior;

/* ************************************************************************* */
// See Hartley and Zisserman, 2nd Ed., page 312
Point3 triangulateDLT(const std::vector<Pose3>& poses, const vector<Matrix>& projection_matrices,
    const vector<Point2>& measurements, const Cal3_S2 &K, double rank_tol, bool optimize) {

  Matrix A = zeros(projection_matrices.size() *2, 4);

  for(size_t i=0; i< projection_matrices.size(); i++) {
    size_t row = i*2;
    const Matrix& projection = projection_matrices.at(i);
    const Point2& p = measurements.at(i);

    // build system of equations
    A.row(row) = p.x() * projection.row(2) - projection.row(0);
    A.row(row+1) = p.y() * projection.row(2) - projection.row(1);
  }
  int rank;
  double error;
  Vector v;
  boost::tie(rank, error, v) = DLT(A, rank_tol);
  //  std::cout << "s " << s.transpose() << std:endl;

  if(rank < 3)
    throw(TriangulationUnderconstrainedException());

  Point3 point = Point3(sub( (v / v(3)),0,3));
  if (optimize) {
      NonlinearFactorGraph graph;
      gtsam::Values::shared_ptr values(new gtsam::Values());
      static SharedNoiseModel noise(noiseModel::Unit::Create(2));
      static SharedNoiseModel prior_model(noiseModel::Diagonal::Sigmas(Vector_(6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6)));
      int ij = 0;
      BOOST_FOREACH(const Point2 &measurement, measurements) {
          // Factor for pose i
          ProjectionFactor *projectionFactor = new ProjectionFactor(measurement, noise, X(ij), L(0), boost::make_shared<Cal3_S2>(K));
          graph.push_back( boost::make_shared<ProjectionFactor>(*projectionFactor) );

          // Prior on pose
          graph.push_back(Pose3Prior(X(ij), poses[ij], prior_model));

          // Initial pose values
          values->insert( X(ij), poses[ij]);

          ij++;
      }

      // Initial landmark value
      values->insert(L(0), point);

      // Optimize
      LevenbergMarquardtParams params;
      params.verbosityLM = LevenbergMarquardtParams::TRYLAMBDA;
      params.verbosity = NonlinearOptimizerParams::ERROR;
      params.lambdaInitial = 1;
      params.lambdaFactor = 10;
      params.maxIterations = 100;
      params.absoluteErrorTol = 1.0;
      params.verbosityLM = LevenbergMarquardtParams::SILENT;
      params.verbosity = NonlinearOptimizerParams::SILENT;
      params.linearSolverType = SuccessiveLinearizationParams::MULTIFRONTAL_CHOLESKY;
      LevenbergMarquardtOptimizer optimizer(graph, *values, params);
      Values result = optimizer.optimize();

      point = result.at<Point3>(L(0));

  }
  return point;
}

/* ************************************************************************* */

} // namespace gtsam
