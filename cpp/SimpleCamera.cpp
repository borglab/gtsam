/*
 * SimpleCamera.cpp
 *
 *  Created on: Aug 16, 2009
 *      Author: dellaert
 */

#include "SimpleCamera.h"
#include "CalibratedCamera.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */

	SimpleCamera::SimpleCamera(const Cal3_S2& K,
			const CalibratedCamera& calibrated) :
		calibrated_(calibrated), K_(K) {
	}

	SimpleCamera::SimpleCamera(const Cal3_S2& K, const Pose3& pose) :
		calibrated_(pose), K_(K) {
	}

	SimpleCamera::~SimpleCamera() {
	}

	pair<Point2, bool> SimpleCamera::projectSafe(const Point3& P) const {
		Point3 cameraPoint = transform_to(calibrated_.pose(), P);
		Point2 intrinsic = project_to_camera(cameraPoint);
		Point2 projection = uncalibrate(K_, intrinsic);
		return pair<Point2, bool>(projection, cameraPoint.z() > 0);
	}

	Point2 SimpleCamera::project(const Point3 & P) const {
		pair<Point2, bool> projected = projectSafe(P);
		return projected.first;
	}

	SimpleCamera SimpleCamera::level(const Cal3_S2& K, const Pose2& pose2, double height) {
		return SimpleCamera(K, CalibratedCamera::level(pose2, height));
	}

	/* ************************************************************************* */
	// measurement functions and derivatives
	/* ************************************************************************* */

	pair<Point2, bool> projectSafe(const SimpleCamera& camera, const Point3& point) {
		return camera.projectSafe(point);
	}

	Point2 project(const SimpleCamera& camera, const Point3& point) {
		return camera.project(point);
	}

	/* ************************************************************************* */
	Matrix Dproject_pose(const SimpleCamera& camera, const Point3& point) {
		Point2 intrinsic = project(camera.calibrated_, point);
		Matrix D_intrinsic_pose = Dproject_pose(camera.calibrated_, point);
		Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);
		Matrix D_projection_pose = D_projection_intrinsic * D_intrinsic_pose;
		return D_projection_pose;
	}

	/* ************************************************************************* */
	Matrix Dproject_point(const SimpleCamera& camera, const Point3& point) {
		Point2 intrinsic = project(camera.calibrated_, point);
		Matrix D_intrinsic_point = Dproject_point(camera.calibrated_, point);
		Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);
		Matrix D_projection_point = D_projection_intrinsic * D_intrinsic_point;
		return D_projection_point;
	}

	/* ************************************************************************* */
	Point2 Dproject_pose_point(const SimpleCamera& camera, const Point3& point,
			Matrix& D_projection_pose, Matrix& D_projection_point) {

		Point2 intrinsic = project(camera.calibrated_, point);
		Matrix D_intrinsic_pose = Dproject_pose(camera.calibrated_, point);
		Matrix D_intrinsic_point = Dproject_point(camera.calibrated_, point);

		Point2 projection = uncalibrate(camera.K_, intrinsic);
		Matrix D_projection_intrinsic = Duncalibrate2(camera.K_, intrinsic);

		D_projection_pose = D_projection_intrinsic * D_intrinsic_pose;
		D_projection_point = D_projection_intrinsic * D_intrinsic_point;

		return projection;
	}

/* ************************************************************************* */
}
