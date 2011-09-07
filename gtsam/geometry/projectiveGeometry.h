/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file projectiveGeometry.h
 * @brief Projective geometry, implemented using tensor library
 * @date Feb 12, 2010
 * @author Frank Dellaert
 */

#pragma once

#include <list>
#include <gtsam/geometry/tensors.h>

namespace gtsam {

	/**
	 * 2D Point in homogeneous coordinates
	 * @ingroup geometry
	 */
	typedef tensors::Tensor1<3> Point2h;
	Point2h point2h(double x, double y, double w); ///< create Point2h

	/**
	 * 2D Line in homogeneous coordinates
	 * @ingroup geometry
	 */
	typedef tensors::Tensor1<3> Line2h;
	Line2h line2h(double a, double b, double c); ///< create Line2h

	/**
	 * 2D (homegeneous) Point correspondence
	 * @ingroup geometry
	 */
	struct Correspondence {
		Point2h first;  ///< First point
		Point2h second; ///< Second point

		/// Create a correspondence pair
		Correspondence(const Point2h &p1, const Point2h &p2) :
			first(p1), second(p2) {
		}
		/// Swap points
		Correspondence swap() const {
			return Correspondence(second, first);
		}
		/// print
		void print() {
			tensors::Index<3, 'i'> i;
			tensors::print(first(i), "first :");
			tensors::print(second(i), "second:");
		}
	};

	/**
	 * 2D-2D Homography
	 * @ingroup geometry
	 */
	typedef tensors::Tensor2<3, 3> Homography2;

	/**
	 * Fundamental Matrix
	 * @ingroup geometry
	 */
	typedef tensors::Tensor2<3, 3> FundamentalMatrix;

	/**
	 * Triplet of (homogeneous) 2D points
	 * @ingroup geometry
	 */
	struct Triplet {
		Point2h first;  ///< First point
		Point2h second; ///< Second point
		Point2h third;  ///< Third point

		/// Create a Triplet correspondence
		Triplet(const Point2h &p1, const Point2h &p2, const Point2h &p3) :
			first(p1), second(p2), third(p3) {
		}
		/// print
		void print() {
			tensors::Index<3, 'i'> i;
			tensors::print(first(i), "first :");
			tensors::print(second(i), "second:");
			tensors::print(third(i), "third :");
		}
	};

	/**
	 * Trifocal Tensor
	 * @ingroup geometry
	 */
	typedef tensors::Tensor3<3, 3, 3> TrifocalTensor;

	/**
	 * 3D Point in homogeneous coordinates
	 * @ingroup geometry
	 */
	typedef tensors::Tensor1<4> Point3h;
	Point3h point3h(double X, double Y, double Z, double W); ///< create Point3h

	/**
	 * 3D Plane in homogeneous coordinates
	 * @ingroup geometry
	 */
	typedef tensors::Tensor1<4> Plane3h;
	Plane3h plane3h(double a, double b, double c, double d); ///< create Plane3h

	/**
	 * 3D to 2D projective camera
	 * @ingroup geometry
	 */
	typedef tensors::Tensor2<3, 4> ProjectiveCamera;

} // namespace gtsam
