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

using namespace std;
using namespace boost::assign;

namespace gtsam {

/* ************************************************************************* */
// See Hartley and Zisserman, 2nd Ed., page 312
Point3 triangulateDLT(const vector<Matrix>& projection_matrices,
    const vector<Point2>& measurements, double rank_tol) {

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


  if(rank < 3)
    throw(TriangulationUnderconstrainedException());

  return Point3(sub( (v / v(3)),0,3));
}

/* ************************************************************************* */
Point3 triangulatePoint3(const vector<Pose3>& poses,
    const vector<Point2>& measurements, const Cal3_S2& K, double rank_tol) {

  assert(poses.size() == measurements.size());

  if(poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  vector<Matrix> projection_matrices;

  // construct projection matrices from poses & calibration
  BOOST_FOREACH(const Pose3& pose, poses)
    projection_matrices += K.matrix() * sub(pose.inverse().matrix(),0,3,0,4);

  Point3 triangulated_point = triangulateDLT(projection_matrices, measurements, rank_tol);

  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const Pose3& pose, poses) {
    const Point3& p_local = pose.transform_to(triangulated_point);
    if(p_local.z() <= 0)
      throw(TriangulationCheiralityException());
  }

  return triangulated_point;
}

/* ************************************************************************* */

} // namespace gtsam
