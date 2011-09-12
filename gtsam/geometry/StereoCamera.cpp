/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StereoCamera.h
 * @brief   A Stereo Camera based on two Simple Cameras
 * @author  Chris Beall
 */

#include <gtsam/geometry/StereoCamera.h>

using namespace std;
using namespace gtsam;

namespace gtsam {

/* ************************************************************************* */
StereoCamera::StereoCamera(const Pose3& leftCamPose, const Cal3_S2Stereo::shared_ptr K) :
	leftCamPose_(leftCamPose), K_(K) {
}

/* ************************************************************************* */
StereoPoint2 StereoCamera::project(const Point3& point,
		boost::optional<Matrix&> H1,
		boost::optional<Matrix&> H2) const {

	Point3 cameraPoint = leftCamPose_.transform_to(point, H1, H2);

	if(H1 || H2) {
		Matrix D_project_point = Dproject_to_stereo_camera1(cameraPoint); // 3x3 Jacobian

	  if (H1)
	  	*H1 =  D_project_point * *H1;
	  if (H2)
	  	*H2 =  D_project_point * *H2;
	}

	const Cal3_S2Stereo& K = *K_;
	double f_x = K.fx(), f_y = K.fy(), b=K.baseline();
	double d = 1.0 / cameraPoint.z();
	double uL = K.px() + d * f_x * cameraPoint.x();
	double uR = K.px() + d * f_x * (cameraPoint.x() - b);
	double v = K.py() + d * f_y * cameraPoint.y();
	return StereoPoint2(uL, uR, v);
}

/* ************************************************************************* */
Matrix StereoCamera::Dproject_to_stereo_camera1(const Point3& P) const {
	double d = 1.0 / P.z(), d2 = d * d;
	const Cal3_S2Stereo& K = *K_;
	double f_x = K.fx(), f_y = K.fy(), b=K.baseline();
	return Matrix_(3, 3,
			f_x*d, 0.0, -f_x *P.x() * d2,
			f_x*d, 0.0, -f_x *(P.x() - b) * d2,
			0.0, f_y*d, -f_y*P.y() * d2);
}

}
