/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Cal3Bundler.cpp
 * @date Sep 25, 2010
 * @author ydjian
 */

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3Bundler.h>

namespace gtsam {

/* ************************************************************************* */
Cal3Bundler::Cal3Bundler() : f_(1), k1_(0), k2_(0) {}

/* ************************************************************************* */
Cal3Bundler::Cal3Bundler(const Vector &v) : f_(v(0)), k1_(v(1)), k2_(v(2)) {}

/* ************************************************************************* */
Cal3Bundler::Cal3Bundler(const double f, const double k1, const double k2) : f_(f), k1_(k1), k2_(k2) {}

/* ************************************************************************* */
Matrix Cal3Bundler::K() const { return Matrix_(3,3, f_, 0.0, 0.0, 0.0, f_, 0.0, 0.0, 0.0, 1.0); }

/* ************************************************************************* */
Vector Cal3Bundler::k() const { return Vector_(4, k1_, k2_, 0, 0) ; }

/* ************************************************************************* */
Vector Cal3Bundler::vector() const { return Vector_(3, f_, k1_, k2_) ; }

/* ************************************************************************* */
void Cal3Bundler::print(const std::string& s) const { gtsam::print(vector(), s + ".K") ; }

/* ************************************************************************* */
bool Cal3Bundler::equals(const Cal3Bundler& K, double tol) const {
	if (fabs(f_ - K.f_) > tol || fabs(k1_ - K.k1_) > tol || fabs(k2_ - K.k2_) > tol)
		return false;
	return true ;
}

/* ************************************************************************* */
Point2 Cal3Bundler::uncalibrate(const Point2& p,
		   boost::optional<Matrix&> H1,
		   boost::optional<Matrix&> H2) const {
//	r = x^2 + y^2 ;
//	g = (1 + k(1)*r + k(2)*r^2) ;
//	pi(:,i) = g * pn(:,i)
	const double x = p.x(), y = p.y() ;
	const double r = x*x + y*y ;
	const double r2 = r*r ;
	const double g = 1 + k1_ * r + k2_*r2 ; // save one multiply
	const double fg = f_*g ;

	// semantic meaningful version
	//if (H1) *H1 = D2d_calibration(p) ;
	//if (H2) *H2 = D2d_intrinsic(p) ;

	// unrolled version, much faster
	if ( H1 || H2 ) {

		const double fx = f_*x, fy = f_*y ;
		if ( H1 ) {
			*H1 = Matrix_(2, 3,
				g*x, fx*r , fx*r2,
				g*y, fy*r , fy*r2) ;
		}

		if ( H2 ) {
			const double dr_dx = 2*x ;
			const double dr_dy = 2*y ;
			const double dg_dx = k1_*dr_dx + k2_*2*r*dr_dx ;
			const double dg_dy = k1_*dr_dy + k2_*2*r*dr_dy ;
			*H2 = Matrix_(2, 2,
					fg + fx*dg_dx, fx*dg_dy,
					fy*dg_dx , fg + fy*dg_dy) ;
		}
	}

	return Point2(fg * x, fg * y) ;
}

/* ************************************************************************* */
Matrix Cal3Bundler::D2d_intrinsic(const Point2& p) const {
	const double x = p.x(), y = p.y(), xx = x*x, yy = y*y;
	const double r = xx + yy ;
	const double dr_dx = 2*x ;
	const double dr_dy = 2*y ;
	const double g = 1 + (k1_+k2_*r) * r ; // save one multiply
	//const double g = 1 + k1_*r + k2_*r*r ;
	const double dg_dx = k1_*dr_dx + k2_*2*r*dr_dx ;
	const double dg_dy = k1_*dr_dy + k2_*2*r*dr_dy ;

	Matrix DK = Matrix_(2, 2, f_, 0.0, 0.0, f_);
	Matrix DR = Matrix_(2, 2,
			g + x*dg_dx, x*dg_dy,
			y*dg_dx , g + y*dg_dy) ;
	return DK * DR;
}

/* ************************************************************************* */
Matrix Cal3Bundler::D2d_calibration(const Point2& p) const {

	const double x = p.x(), y = p.y(), xx = x*x, yy = y*y ;
	const double r = xx + yy ;
	const double r2 = r*r ;
	const double f = f_ ;
	const double g = 1 + (k1_+k2_*r) * r ; // save one multiply
	//const double g = (1+k1_*r+k2_*r2) ;

	return Matrix_(2, 3,
	g*x, f*x*r , f*x*r2,
	g*y, f*y*r , f*y*r2);
}

/* ************************************************************************* */
Matrix Cal3Bundler::D2d_intrinsic_calibration(const Point2& p) const {

	const double x = p.x(), y = p.y(), xx = x*x, yy = y*y;
	const double r = xx + yy ;
	const double r2 = r*r;
	const double dr_dx = 2*x ;
	const double dr_dy = 2*y ;
	const double g = 1 + (k1_*r) + (k2_*r2) ;
	const double f = f_ ;
	const double dg_dx = k1_*dr_dx + k2_*2*r*dr_dx ;
	const double dg_dy = k1_*dr_dy + k2_*2*r*dr_dy ;

	Matrix H = Matrix_(2,5,
			f*(g + x*dg_dx), f*(x*dg_dy), g*x, f*x*r , f*x*r2,
			f*(y*dg_dx) , f*(g + y*dg_dy), g*y, f*y*r , f*y*r2);

	return H ;
}

/* ************************************************************************* */
Cal3Bundler Cal3Bundler::retract(const Vector& d) const { return Cal3Bundler(vector() + d) ; }

/* ************************************************************************* */
Vector Cal3Bundler::localCoordinates(const Cal3Bundler& T2) const { return vector() - T2.vector(); }

}
