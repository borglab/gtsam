/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * projectiveGeometry.cpp
 * @brief Projective geometry, implemented using tensor library
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#include <boost/foreach.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <gtsam/base/Matrix.h>
typedef boost::numeric::ublas::matrix_row<Matrix> Row;

#include <gtsam/geometry/tensorInterface.h>
#include <gtsam/geometry/projectiveGeometry.h>

namespace gtsam {

	using namespace std;
	using namespace tensors;

	static Eta3 eta;
	static Index<3, 'a'> a, _a;
	static Index<3, 'b'> b, _b;
	static Index<3, 'c'> c, _c;
	static Index<3, 'd'> d, _d;
	static Index<3, 'd'> e, _e;
	static Index<3, 'f'> f, _f;
	static Index<3, 'g'> g, _g;

	/* ************************************************************************* */
	Point2h point2h(double x, double y, double w) {
		double data[3];
		data[0] = x;
		data[1] = y;
		data[2] = w;
		return data;
	}

	/* ************************************************************************* */
	Line2h line2h(double a, double b, double c) {
		double data[3];
		data[0] = a;
		data[1] = b;
		data[2] = c;
		return data;
	}

	/* ************************************************************************* */
	Point3h point3h(double X, double Y, double Z, double W) {
		double data[4];
		data[0] = X;
		data[1] = Y;
		data[2] = Z;
		data[3] = W;
		return data;
	}

	/* ************************************************************************* */
	Plane3h plane3h(double a, double b, double c, double d) {
		double data[4];
		data[0] = a;
		data[1] = b;
		data[2] = c;
		data[3] = d;
		return data;
	}


	/* ************************************************************************* */

} // namespace gtsam
