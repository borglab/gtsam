/* ----------------------------------------------------------------------------

*GTSAM Copyright 2010, Georgia Tech Research Corporation,
*Atlanta, Georgia 30332-0415
*All Rights Reserved
*Authors: Frank Dellaert, et al. (see THANKS for the full author list)

*See LICENSE for the license information

*-------------------------------------------------------------------------- */

/**
*@file    StereoCamera.h
*@brief   A Stereo Camera based on two Simple Cameras
*@author  Chris Beall
 */

#include <gtsam/geometry/StereoCamera.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

	/* ************************************************************************* */
	StereoCamera::StereoCamera(const Pose3& leftCamPose,
			const Cal3_S2Stereo::shared_ptr K) :
			leftCamPose_(leftCamPose), K_(K) {
	}

	/* ************************************************************************* */
	StereoPoint2 StereoCamera::project(const Point3& point,
			boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

#ifdef STEREOCAMERA_CHAIN_RULE
		const Point3 q = leftCamPose_.transform_to(point, H1, H2);
#else
		// omit derivatives
		const Point3 q = leftCamPose_.transform_to(point);
#endif

		// get calibration
		const Cal3_S2Stereo& K = *K_;
		const double fx = K.fx(), fy = K.fy(), b = K.baseline();

		// calculate scaled but not translated image coordinates
		const double d = 1.0 / q.z();
		const double x = q.x(), y = q.y();
		const double dfx = d*fx, dfy = d*fy;
		const double uL = dfx*x;
		const double uR = dfx*(x - b);
		const double v  = dfy*y;

		// check if derivatives need to be computed
		if (H1 || H2) {
#ifdef STEREOCAMERA_CHAIN_RULE
			// just implement chain rule
			Matrix D_project_point = Dproject_to_stereo_camera1(q); // 3x3 Jacobian
			if (H1) *H1 = D_project_point*(*H1);
			if (H2) *H2 = D_project_point*(*H2);
#else
			// optimized version, see StereoCamera.nb
			if (H1) {
				const double v1 = v/fy, v2 = fx*v1, dx=d*x;
				*H1 = Matrix_(3, 6,
							  uL*v1, -fx-dx*uL,     v2, -dfx,  0.0, d*uL,
							  uR*v1, -fx-dx*uR,     v2, -dfx,  0.0, d*uR,
						fy + v*v1,    -dx*v , -x*dfy,  0.0, -dfy, d*v
					);
			}
			if (H2) {
				const Matrix R(leftCamPose_.rotation().matrix());
				*H2 = d * Matrix_(3, 3,
						 fx*R(0, 0) - R(0, 2)*uL, fx*R(1, 0) - R(1, 2)*uL, fx*R(2, 0) - R(2, 2)*uL,
						 fx*R(0, 0) - R(0, 2)*uR, fx*R(1, 0) - R(1, 2)*uR, fx*R(2, 0) - R(2, 2)*uR,
						 fy*R(0, 1) - R(0, 2)*v , fy*R(1, 1) - R(1, 2)*v , fy*R(2, 1) - R(2, 2)*v
				 );
			}
#endif
		}

		// finally translate
		return StereoPoint2(K.px() + uL, K.px() + uR, K.py() + v);
	}

	/* ************************************************************************* */
	Matrix StereoCamera::Dproject_to_stereo_camera1(const Point3& P) const {
		double d = 1.0 / P.z(), d2 = d*d;
		const Cal3_S2Stereo& K = *K_;
		double f_x = K.fx(), f_y = K.fy(), b = K.baseline();
		return Matrix_(3, 3,
				 f_x*d,   0.0, -d2*f_x* P.x(),
				 f_x*d,   0.0, -d2*f_x*(P.x() - b),
				   0.0, f_y*d, -d2*f_y* P.y()
		);
	}

}
