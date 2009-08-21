/*
 * CalibratedCamera.cpp
 *
 *  Created on: Aug 17, 2009
 *      Author: dellaert
 */

#include "CalibratedCamera.h"

namespace gtsam {

	/* ************************************************************************* */
	// Auxiliary functions
	/* ************************************************************************* */

	Point2 project_to_camera(const Point3& P) {
		return Point2(P.x() / P.z(), P.y() / P.z());
	}

	Matrix Dproject_to_camera1(const Point3& P) {
		double d = 1.0 / P.z(), d2 = d * d;
		return Matrix_(2, 3, d, 0.0, -P.x() * d2, 0.0, d, -P.y() * d2);
	}

	/* ************************************************************************* */

	CalibratedCamera::CalibratedCamera(const Pose3& pose) :
		pose_(pose) {
	}

	CalibratedCamera::~CalibratedCamera() {
	}

	Point2 CalibratedCamera::project(const Point3 & P) const {
		Point3 cameraPoint = transform_to(pose_, P);
		Point2 intrinsic = project_to_camera(cameraPoint);
		return intrinsic;
	}

	/* ************************************************************************* */
	Point2 project(const CalibratedCamera& camera, const Point3& point) {
		return camera.project(point);
	}

	/* ************************************************************************* */
	Matrix Dproject_pose(const CalibratedCamera& camera, const Point3& point) {
		Point3 cameraPoint = transform_to(camera.pose(), point);
		Matrix D_cameraPoint_pose = Dtransform_to1(camera.pose(), point);

		Point2 intrinsic = project_to_camera(cameraPoint);
		Matrix D_intrinsic_cameraPoint = Dproject_to_camera1(cameraPoint);

		Matrix D_intrinsic_pose = D_intrinsic_cameraPoint * D_cameraPoint_pose;
		return D_intrinsic_pose;
	}

	/* ************************************************************************* */
	Matrix Dproject_point(const CalibratedCamera& camera, const Point3& point) {
		Point3 cameraPoint = transform_to(camera.pose(), point);
		Matrix D_cameraPoint_point = Dtransform_to2(camera.pose());

		Point2 intrinsic = project_to_camera(cameraPoint);
		Matrix D_intrinsic_cameraPoint = Dproject_to_camera1(cameraPoint);

		Matrix D_intrinsic_point = D_intrinsic_cameraPoint * D_cameraPoint_point;
		return D_intrinsic_point;
	}

	/* ************************************************************************* */
	void Dproject_pose_point(const CalibratedCamera& camera, const Point3& point,
			Point2& intrinsic, Matrix& D_intrinsic_pose, Matrix& D_intrinsic_point) {

		Point3 cameraPoint = transform_to(camera.pose(), point);
		Matrix D_cameraPoint_pose = Dtransform_to1(camera.pose(), point); // 3*6
		Matrix D_cameraPoint_point = Dtransform_to2(camera.pose()); // 3*3

		intrinsic = project_to_camera(cameraPoint);
		Matrix D_intrinsic_cameraPoint = Dproject_to_camera1(cameraPoint);

		D_intrinsic_pose = D_intrinsic_cameraPoint * D_cameraPoint_pose; // 2*6
		D_intrinsic_point = D_intrinsic_cameraPoint * D_cameraPoint_point; // 2*3
	}

/* ************************************************************************* */
}
