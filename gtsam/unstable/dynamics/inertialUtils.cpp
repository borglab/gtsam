/**
 * @file inertialUtils.cpp
 *
 * @date Nov 28, 2011
 * @author Alex Cunningham
 */

#include <gtsam2/dynamics/inertialUtils.h>

namespace gtsam {
namespace dynamics {

/* ************************************************************************* */
Matrix RRTMbn(const Vector& euler) {
	assert(euler.size() == 3);
	const double s1 = sin(euler(1-1)), c1 = cos(euler(1-1));
	const double t2 = tan(euler(2-1)), c2 = cos(euler(2-1));
	Matrix Ebn(3,3);
	Ebn << 1.0, s1 * t2, c1 * t2,
			   0.0,      c1,     -s1,
			   0.0, s1 / c2, c1 / c2;
	return Ebn;
}

/* ************************************************************************* */
Matrix RRTMbn(const Rot3& att) {
	return RRTMbn(att.rpy());
}

/* ************************************************************************* */
Matrix RRTMnb(const Vector& euler) {
	assert(euler.size() == 3);
	Matrix Enb(3,3);
	const double s1 = sin(euler(1-1)), c1 = cos(euler(1-1));
	const double s2 = sin(euler(2-1)), c2 = cos(euler(2-1));
	Enb << 1.0, 0.0,   -s2,
         0.0,  c1, s1*c2,
         0.0, -s1, c1*c2;
	return Enb;
}

/* ************************************************************************* */
Matrix RRTMnb(const Rot3& att) {
	return RRTMnb(att.rpy());
}

} // \namespace dynamics
} // \namespace gtsam



