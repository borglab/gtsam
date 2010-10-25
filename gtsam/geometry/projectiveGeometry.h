/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * projectiveGeometry.h
 * @brief Projective geometry, implemented using tensor library
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#pragma once

#include <list>
#include <gtsam/geometry/tensors.h>

namespace gtsam {

	/** 2D Point */
	typedef tensors::Tensor1<3> Point2h;
	Point2h point2h(double x, double y, double w);

	/** 2D Line */
	typedef tensors::Tensor1<3> Line2h;
	Line2h line2h(double a, double b, double c);

	/** 2D Point corrrespondence */
	struct Correspondence {
		Point2h first, second;
		Correspondence(const Point2h &p1, const Point2h &p2) :
			first(p1), second(p2) {
		}
		Correspondence swap() const {
			return Correspondence(second, first);
		}
		void print() {
			tensors::Index<3, 'i'> i;
			tensors::print(first(i), "first :");
			tensors::print(second(i), "second:");
		}
	};

	/** 2D-2D Homography */
	typedef tensors::Tensor2<3, 3> Homography2;

	/** Fundamental Matrix */
	typedef tensors::Tensor2<3, 3> FundamentalMatrix;

	/** Triplet of points */
	struct Triplet {
		Point2h first, second, third;
		Triplet(const Point2h &p1, const Point2h &p2, const Point2h &p3) :
			first(p1), second(p2), third(p3) {
		}
		void print() {
			tensors::Index<3, 'i'> i;
			tensors::print(first(i), "first :");
			tensors::print(second(i), "second:");
			tensors::print(third(i), "third :");
		}
	};

	/** Trifocal Tensor */
	typedef tensors::Tensor3<3, 3, 3> TrifocalTensor;

	/** 3D Point */
	typedef tensors::Tensor1<4> Point3h;
	Point3h point3h(double X, double Y, double Z, double W);

	/** 3D Plane */
	typedef tensors::Tensor1<4> Plane3h;
	Plane3h plane3h(double a, double b, double c, double d);

	/** 3D to 2D projective camera */
	typedef tensors::Tensor2<3, 4> ProjectiveCamera;

} // namespace gtsam
