/**
 * @file   Cal3_S2.cpp
 * @brief  The most common 5DOF 3D->2D calibration
 * @author Frank Dellaert
 */

#include <fstream>
#include <iostream>

#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam {
using namespace std;

/* ************************************************************************* */
Cal3_S2::Cal3_S2(const std::string &path) {

	char buffer[200];
	buffer[0] = 0;
	sprintf(buffer, "%s/calibration_info.txt", path.c_str());
	std::ifstream infile(buffer, std::ios::in);

	if (infile)
		infile >> fx_ >> fy_ >> s_ >> u0_ >> v0_;
	else {
		printf("Unable to load the calibration\n");
		exit(0);
	}

	infile.close();
}

/* ************************************************************************* */

bool Cal3_S2::equals(const Cal3_S2& K, double tol) const {
	if (fabs(fx_ - K.fx_) > tol) return false;
	if (fabs(fy_ - K.fy_) > tol) return false;
	if (fabs(s_ - K.s_) > tol) return false;
	if (fabs(u0_ - K.u0_) > tol) return false;
	if (fabs(v0_ - K.v0_) > tol) return false;
	return true;
}

/* ************************************************************************* */
Point2 Cal3_S2::uncalibrate(const Point2& p,
		boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {
	if (H1) *H1 = Matrix_(2, 5, p.x(), 000.0, p.y(), 1.0, 0.0, 0.000, p.y(), 0.000, 0.0,1.0);
	if (H2) *H2 = Matrix_(2, 2, fx_, s_, 0.000, fy_);
	const double x = p.x(), y = p.y();
	return Point2(fx_ * x + s_ * y + u0_, fy_ * y + v0_);
}

/* ************************************************************************* */

} // namespace gtsam
