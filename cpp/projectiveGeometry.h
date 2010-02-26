/*
 * projectiveGeometry.h
 * @brief Projective geometry, implemented using tensor library
 * Created on: Feb 12, 2010
 * @author: Frank Dellaert
 */

#pragma once

#include <list>
#include "tensors.h"

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

	/**
	 * Estimate homography from point correspondences
	 * Result is H(_a,b) s.t. p.second(b) = H(_a,b) * p.first(a) for all p
	 */
	Homography2 estimateHomography2(
			const std::list<Correspondence>& correspondences);

	/** Fundamental Matrix */
	typedef tensors::Tensor2<3, 3> FundamentalMatrix;

	/**
	 * Estimate fundamental matrix from point correspondences
	 * Result is F(_a,_b) s.t. H(_a,_b) * p.first(a) * p.second(b) == 0 for all p
	 */
	FundamentalMatrix estimateFundamentalMatrix(
			const std::list<Correspondence>& correspondences);

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

	/**
	 * Estimate trifocal Tensor from point triplets
	 * Result is T(_a,b,c)
	 */
	TrifocalTensor estimateTrifocalTensor(const std::list<Triplet>& triplets);

	/** 3D Point */
	typedef tensors::Tensor1<4> Point3h;
	Point3h point3h(double X, double Y, double Z, double W);

	/** 3D Plane */
	typedef tensors::Tensor1<4> Plane3h;
	Plane3h plane3h(double a, double b, double c, double d);

	/** 3D to 2D projective camera */
	typedef tensors::Tensor2<3, 4> ProjectiveCamera;

} // namespace gtsam
