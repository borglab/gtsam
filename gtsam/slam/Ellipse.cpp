/**
 * @file Ellipse.cpp
 * @author Alex Cunningham
 */

#include <boost/foreach.hpp>
#include <gtsam/slam/Ellipse.h>

namespace gtsam {

/* ***************************************************************** */
void Ellipse::calculateMoments(const Matrix& covariance) {
	// CODE SOURCE: pulled from MastSLAM linearDrawing, derived from Richard's code
	// eigenvalue decomposition
	Matrix U,V;
	Vector S;
	svd(covariance,U,S,V);

	// invert and sqrt diagonal of S
	// We also arbitrarily choose sign to make result have positive signs
	for(size_t i = 0; i<2; i++)
		S(i) = pow(S(i), 0.5);
	Matrix eig = vector_scale(U, S); // U*S;

	// draw ellipse
	moment1_ = Point2(eig(0,0), eig(1,0));
	moment2_ = Point2(eig(0,1), eig(1,1));
}

/* ***************************************************************** */
Ellipse::Ellipse(const Point2& m, const Matrix& covariance)
 : mean_(m), cov_(covariance)
{
	calculateMoments(cov_);
	for (double angle = 0.0; angle < 2.0 * M_PI; angle += (2.0 * M_PI / 50.0))
		unit_circle_.push_back(Point2(cos(angle), sin(angle)));
}

/* ***************************************************************** */
Ellipse::Ellipse(const Pose2& m, const Matrix& covariance)
 : mean_(m.t()), cov_(sub(covariance, 0, 2, 0, 2))
{
	calculateMoments(cov_);
	for (double angle = 0.0; angle < 2.0 * M_PI; angle += (2.0 * M_PI / 50.0))
		unit_circle_.push_back(Point2(cos(angle), sin(angle)));
}

/* ***************************************************************** */
void Ellipse::print(const std::string& s) const {
	mean_.print(s + std::string(" mean"));
	gtsam::print(cov_, "covariance");
	moment1_.print("moment1");
	moment2_.print("moment2");
}

} // \namespace gtsam
