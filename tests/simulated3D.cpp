/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
* @file   Simulated3D.cpp
* @brief  measurement functions and derivatives for simulated 3D robot
* @author Alex Cunningham
**/

#include <tests/simulated3D.h>

namespace gtsam {

namespace simulated3D {

Point3 prior (const Point3& x, boost::optional<Matrix&> H) {
	if (H) *H = eye(3);
	return x;
}

Point3 odo(const Point3& x1, const Point3& x2,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
	if (H1) *H1 = -1 * eye(3);
	if (H2) *H2 = eye(3);
	return x2 - x1;
}

Point3 mea(const Point3& x,  const Point3& l,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) {
	if (H1) *H1 = -1 * eye(3);
	if (H2) *H2 = eye(3);
	return l - x;
}

}} // namespace gtsam::simulated3D
