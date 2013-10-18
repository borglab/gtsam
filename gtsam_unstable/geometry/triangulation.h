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

Point3 triangulateDLT(const std::vector<Pose3>& poses, const std::vector<Matrix>& projection_matrices,
    const std::vector<Point2>& measurements, const Cal3_S2 &K, double rank_tol, bool optimize);

Point3 triangulateDLT(const std::vector<Pose3>& poses, const std::vector<Matrix>& projection_matrices,
    const std::vector<Point2>& measurements, const std::vector<Cal3_S2::shared_ptr> &Ks, double rank_tol, bool optimize);


/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. The function checks that the
 * resulting point lies in front of all cameras, but has no other checks
 * to verify the quality of the triangulation.
 * @param poses A vector of camera poses
 * @param measurements A vector of camera measurements
 * @param K The camera calibration
 * @param rank tolerance, default 1e-9
 * @return Returns a Point3 on success, boost::none otherwise.
 */
template<class CALIBRATION>
GTSAM_UNSTABLE_EXPORT Point3 triangulatePoint3(const std::vector<Pose3>& poses,
    const std::vector<Point2>& measurements, const CALIBRATION& K, double rank_tol =  1e-9, bool optimize = false){

  assert(poses.size() == measurements.size());

  if(poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  std::vector<Matrix> projection_matrices;

  // construct projection matrices from poses & calibration
  BOOST_FOREACH(const Pose3& pose, poses){
  projection_matrices.push_back( K.K() * sub(pose.inverse().matrix(),0,3,0,4) );
  // std::cout << "Calibration i \n" << K.K() << std::endl;
  // std::cout << "rank_tol i \n" << rank_tol << std::endl;
  }

  Point3 triangulated_point = triangulateDLT(poses, projection_matrices, measurements, K, rank_tol, optimize);

  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const Pose3& pose, poses) {
    const Point3& p_local = pose.transform_to(triangulated_point);
    if(p_local.z() <= 0)
      throw(TriangulationCheiralityException());
  }

  return triangulated_point;
}

/* ************************************************************************* */
template<class CALIBRATION>
Point3 triangulatePoint3(const std::vector<Pose3>& poses,
    const  std::vector<Point2>& measurements, const  std::vector<boost::shared_ptr<CALIBRATION> >& Ks, double rank_tol,
    bool optimize = false) {

  assert(poses.size() == measurements.size());
  assert(poses.size() == Ks.size());

  if(poses.size() < 2)
    throw(TriangulationUnderconstrainedException());

  std::vector<Matrix> projection_matrices;

  // construct projection matrices from poses & calibration
  for(size_t i = 0; i<poses.size(); i++){
    projection_matrices.push_back( Ks.at(i)->K() * sub(poses.at(i).inverse().matrix(),0,3,0,4) );
    // std::cout << "2Calibration i \n" << Ks.at(i)->K() << std::endl;
    // std::cout << "2rank_tol i \n" << rank_tol << std::endl;
  }

  Point3 triangulated_point = triangulateDLT(poses, projection_matrices, measurements, Ks, rank_tol, optimize);

  // verify that the triangulated point lies infront of all cameras
  BOOST_FOREACH(const Pose3& pose, poses) {
    const Point3& p_local = pose.transform_to(triangulated_point);
    if(p_local.z() <= 0)
      throw(TriangulationCheiralityException());
  }

  return triangulated_point;
}


} // \namespace gtsam


