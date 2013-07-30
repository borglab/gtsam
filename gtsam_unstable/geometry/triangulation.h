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

namespace gtsam {

/**
 * Function to triangulate 3D landmark point from an arbitrary number
 * of poses (at least 2) using the DLT. The function checks that the
 * resulting point lies in front of all cameras, but has no other checks
 * to verify the quality of the triangulation.
 * @param poses A vector of camera poses
 * @param measurements A vector of camera measurements
 * @param K The camera calibration
 * @return Returns a Point3 on success, boost::none otherwise.
 */
boost::optional<Point3> triangulatePoint3(const std::vector<Pose3>& poses,
    const std::vector<Point2>& measurements, const Cal3_S2& K);


} // \namespace gtsam


