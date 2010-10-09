/**
* @file   Simulated3D.cpp
* @brief  measurement functions and derivatives for simulated 3D robot
* @author Alex Cunningham
**/

#include <gtsam/slam/Simulated3D.h>
#include <gtsam/nonlinear/LieValues-inl.h>
#include <gtsam/nonlinear/TupleValues-inl.h>

namespace gtsam {

using namespace simulated3D;
INSTANTIATE_LIE_CONFIG(PointKey)
INSTANTIATE_LIE_CONFIG(PoseKey)
INSTANTIATE_TUPLE_CONFIG2(PoseConfig, PointConfig)

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
