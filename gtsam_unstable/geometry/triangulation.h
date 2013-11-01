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

#pragma once

#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/assign/std/vector.hpp>
#include <gtsam_unstable/base/dllexport.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam/slam/ProjectionFactor.h>

namespace gtsam {

/// Exception thrown by triangulateDLT when SVD returns rank < 3
class GTSAM_UNSTABLE_EXPORT TriangulationUnderconstrainedException: public std::runtime_error {
public:
  TriangulationUnderconstrainedException() :
      std::runtime_error("Triangulation Underconstrained Exception.") {
  }
};

/// Exception thrown by triangulateDLT when landmark is behind one or more of the cameras
class GTSAM_UNSTABLE_EXPORT TriangulationCheiralityException: public std::runtime_error {
public:
  TriangulationCheiralityException() :
      std::runtime_error(
          "Triangulation Cheirality Exception: The resulting landmark is behind one or more cameras.") {
  }
};

/* ************************************************************************* */
// See Hartley and Zisserman, 2nd Ed., page 312
/**
 *
 * @param poses Camera poses
 * @param projection_matrices Projection matrices (K*P^-1)
 * @param measurements 2D measurements
 * @param Ks vector of calibrations
 * @param rank_tol SVD rank tolerance
 * @param Flag to turn on nonlinear refinement of triangulation
 * @return Triangulated Point3
 */
template<class CALIBRATION>
Point3 triangulateDLT(const std::vector<Pose3>& poses,
    const std::vector<Matrix>& projection_matrices,
    const std::vector<Point2>& measurements,
    const std::vector<boost::shared_ptr<CALIBRATION> >& Ks, double rank_tol,
    bool optimize) {

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
  Point3 point = Point3(sub((v / v(3)), 0, 3));

  if (optimize) {
    // Create a factor graph
    NonlinearFactorGraph graph;
    gtsam::Values values;
    static SharedNoiseModel unit2(noiseModel::Unit::Create(2));
    static SharedNoiseModel prior_model(noiseModel::Isotropic::Sigma(6, 1e-6));

    // Initial landmark value
    Key landmarkKey = Symbol('p', 0);
    values.insert(landmarkKey, point);

    // Create all projection factors, as well as priors on poses
    Key i = 0;
    BOOST_FOREACH(const Point2 &z_i, measurements) {
      // Factor for pose i
      typedef GenericProjectionFactor<Pose3, Point3, CALIBRATION> ProjectionFactor;
      ProjectionFactor projectionFactor(z_i, unit2, i, landmarkKey, Ks[i]);
      graph.push_back(projectionFactor);

      // Prior on pose
      // Frank says: this is a terrible idea: we turn a 3dof problem into a much more difficult problem
      typedef PriorFactor<Pose3> Pose3Prior;
      graph.push_back(Pose3Prior(i, poses[i], prior_model));

      // Initial pose values
      values.insert(i, poses[i]);
      i++;
    }

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
    params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;
    LevenbergMarquardtOptimizer optimizer(graph, values, params);
    Values result = optimizer.optimize();

    point = result.at<Point3>(landmarkKey);
  }

  return point;
}

/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. The function checks that the
 * resulting point lies in front of all cameras, but has no other checks
 * to verify the quality of the triangulation.
 * @param poses A vector of camera poses
 * @param measurements A vector of camera measurements
 * @param K The camera calibration (Same for all cameras involved)
 * @param rank tolerance, default 1e-9
 * @param optimize Flag to turn on nonlinear refinement of triangulation
 * @return Returns a Point3 on success, boost::none otherwise.
 */
template<class CALIBRATION>
Point3 triangulatePoint3(const std::vector<Pose3>& poses,
    const std::vector<Point2>& measurements, const CALIBRATION& K,
    double rank_tol = 1e-9, bool optimize = false) {

  assert(poses.size() == measurements.size());

  if (poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  std::vector<Matrix> projection_matrices;

  // construct projection matrices from poses & calibration
  BOOST_FOREACH(const Pose3& pose, poses) {
    projection_matrices.push_back(
        K.K() * sub(pose.inverse().matrix(), 0, 3, 0, 4));
    // std::cout << "Calibration i \n" << K.K() << std::endl;
    // std::cout << "rank_tol i \n" << rank_tol << std::endl;
  }

  // create vector with shared pointer to calibration (all the same in this case)
  boost::shared_ptr<CALIBRATION> sharedK = boost::make_shared<CALIBRATION>(K);
  std::vector<boost::shared_ptr<CALIBRATION> > Ks(poses.size(), sharedK);

  Point3 triangulated_point = triangulateDLT(poses, projection_matrices,
      measurements, Ks, rank_tol, optimize);

#ifdef GTSAM_THROW_CHEIRALITY_EXCEPTION
  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const Pose3& pose, poses) {
    const Point3& p_local = pose.transform_to(triangulated_point);
    if(p_local.z() <= 0)
    throw(TriangulationCheiralityException());
  }
#endif

  return triangulated_point;
}

/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. This function is similar to the one
 * above, except that each camera has its own calibration. The function
 * checks that the resulting point lies in front of all cameras, but has
 * no other checks to verify the quality of the triangulation.
 * @param poses A vector of camera poses
 * @param measurements A vector of camera measurements
 * @param Ks Vector of camera calibrations
 * @param rank tolerance, default 1e-9
 * @param optimize Flag to turn on nonlinear refinement of triangulation
 * @return Returns a Point3 on success, boost::none otherwise.
 */
template<class CALIBRATION>
Point3 triangulatePoint3(const std::vector<Pose3>& poses,
    const std::vector<Point2>& measurements,
    const std::vector<boost::shared_ptr<CALIBRATION> >& Ks, double rank_tol =
        1e-9, bool optimize = false) {

  assert(poses.size() == measurements.size());
  assert(poses.size() == Ks.size());

  if (poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  std::vector<Matrix> projection_matrices;

  // construct projection matrices from poses & calibration
  for (size_t i = 0; i < poses.size(); i++) {
    projection_matrices.push_back(
        Ks.at(i)->K() * sub(poses.at(i).inverse().matrix(), 0, 3, 0, 4));
    // std::cout << "2Calibration i \n" << Ks.at(i)->K() << std::endl;
    // std::cout << "2rank_tol i \n" << rank_tol << std::endl;
  }

  Point3 triangulated_point = triangulateDLT(poses, projection_matrices,
      measurements, Ks, rank_tol, optimize);

  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const Pose3& pose, poses) {
    const Point3& p_local = pose.transform_to(triangulated_point);
    if (p_local.z() <= 0)
      throw(TriangulationCheiralityException());
  }

  return triangulated_point;
}

} // \namespace gtsam

