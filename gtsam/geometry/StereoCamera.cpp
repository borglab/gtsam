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
StereoCamera::StereoCamera(const Pose3& leftCamPose, const Cal3_S2& K,
		double baseline) :
	leftCamPose_(leftCamPose), K_(K), baseline_(baseline) {
	Vector calibration = K_.vector();
	fx_ = calibration(0);
	fy_ = calibration(1);
	cx_ = calibration(3);
	cy_ = calibration(4);
}

/* ************************************************************************* */
StereoPoint2 StereoCamera::project(const Point3& point,
		boost::optional<Matrix&> Dproject_stereo_pose,
		boost::optional<Matrix&> Dproject_stereo_point) const {

	Point3 cameraPoint = leftCamPose_.transform_to(point);

	if (Dproject_stereo_pose) {
		Matrix D_cameraPoint_pose;
		Point3 cameraPoint = pose().transform_to(point, D_cameraPoint_pose,
				boost::none);

		Matrix D_intrinsic_cameraPoint = Dproject_to_stereo_camera1(cameraPoint); // 3x3 Jacobian
		Matrix D_intrinsic_pose = D_intrinsic_cameraPoint * D_cameraPoint_pose;

		Matrix D_projection_intrinsic = Duncalibrate2(K()); // 3x3
		*Dproject_stereo_pose = D_projection_intrinsic * D_intrinsic_pose;
	}

	if (Dproject_stereo_point) {
		Matrix D_cameraPoint_point;
		Point3 cameraPoint = pose().transform_to(point, boost::none,
				D_cameraPoint_point);

		Matrix D_intrinsic_cameraPoint = Dproject_to_stereo_camera1(cameraPoint); // 3x3 Jacobian
		Matrix D_intrinsic_point = D_intrinsic_cameraPoint * D_cameraPoint_point;

		Matrix D_projection_intrinsic = Duncalibrate2(K()); // 3x3
		*Dproject_stereo_point = D_projection_intrinsic * D_intrinsic_point;
	}

	double d = 1.0 / cameraPoint.z();
	double uL = cx_ + d * fx_ * cameraPoint.x();
	double uR = cx_ + d * fx_ * (cameraPoint.x() - baseline_);
	double v = cy_ + d * fy_ * cameraPoint.y();
	return StereoPoint2(uL, uR, v);
}

/* ************************************************************************* */
Matrix StereoCamera::Dproject_to_stereo_camera1(const Point3& P) const {
	double d = 1.0 / P.z(), d2 = d * d;
	return Matrix_(3, 3, d, 0.0, -P.x() * d2, d, 0.0, -(P.x() - baseline()) * d2,
			0.0, d, -P.y() * d2);
}

/* ************************************************************************* */
Matrix StereoCamera::Duncalibrate2(const Cal3_S2& K) {
	Vector calibration = K.vector();
	Vector calibration2(3);
	calibration2(0) = calibration(0);
	calibration2(1) = calibration(0);
	calibration2(2) = calibration(1);
	return diag(calibration2);

}

}
