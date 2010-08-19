/*
 * CalibratedCamera.cpp
 *
 *  Created on: Aug 17, 2009
 *      Author: dellaert
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/CalibratedCamera.h>

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

	Point3 backproject_from_camera(const Point2& p, const double scale) {
		return Point3(p.x() * scale, p.y() * scale, scale);
	}

	/* ************************************************************************* */
	// Methods
	/* ************************************************************************* */

	CalibratedCamera::CalibratedCamera(const Pose3& pose) :
		pose_(pose) {
	}

	CalibratedCamera::CalibratedCamera(const Vector &v) : pose_(expmap<Pose3>(v)) {}

	CalibratedCamera::~CalibratedCamera() {}

	CalibratedCamera CalibratedCamera::level(const Pose2& pose2, double height) {
		double st = sin(pose2.theta()), ct = cos(pose2.theta());
		Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
		Rot3 wRc(x, y, z);
		Point3 t(pose2.x(), pose2.y(), height);
		Pose3 pose3(wRc, t);
		return CalibratedCamera(pose3);
	}

	Point2 CalibratedCamera::project(const Point3 & P) const {
		Point3 cameraPoint = transform_to(pose_, P);
		Point2 intrinsic = project_to_camera(cameraPoint);
		return intrinsic;
	}

	/* ************************************************************************* */
	// measurement functions and derivatives
	/* ************************************************************************* */

	Point2 project(const CalibratedCamera& camera, const Point3& point) {
		return camera.project(point);
	}

	/* ************************************************************************* */
	Matrix Dproject_pose(const CalibratedCamera& camera, const Point3& point) {
		const Pose3& pose = camera.pose();
		const Rot3& R = pose.rotation();
		const Point3& r1 = R.r1(), r2 = R.r2(), r3 = R.r3();
		Point3 q = transform_to(pose, point);
		double X = q.x(), Y = q.y(), Z = q.z();
		double d = 1.0 / Z, d2 = d * d, Xd2 = X*d2, Yd2 = Y*d2;
		return Matrix_(2,6,
				X*Yd2, -Z*d-X*Xd2,  d*Y, -d*r1.x()+r3.x()*Xd2, -d*r1.y()+r3.y()*Xd2, -d*r1.z()+r3.z()*Xd2,
				d*Z+Y*Yd2, -X*Yd2, -d*X, -d*r2.x()+r3.x()*Yd2, -d*r2.y()+r3.y()*Yd2, -d*r2.z()+r3.z()*Yd2);
	}

	/* ************************************************************************* */
	Matrix Dproject_point(const CalibratedCamera& camera, const Point3& point) {
		const Pose3& pose = camera.pose();
		const Rot3& R = pose.rotation();
		const Point3& r1 = R.r1(), r2 = R.r2(), r3 = R.r3();
		Point3 q = transform_to(pose, point);
		double X = q.x(), Y = q.y(), Z = q.z();
		double d = 1.0 / Z, d2 = d * d, Xd2 = X*d2, Yd2 = Y*d2;
		return Matrix_(2,3,
				d*r1.x()-r3.x()*Xd2, d*r1.y()-r3.y()*Xd2, d*r1.z()-r3.z()*Xd2,
				d*r2.x()-r3.x()*Yd2, d*r2.y()-r3.y()*Yd2, d*r2.z()-r3.z()*Yd2);
	}

	/* ************************************************************************* */
	Point2 Dproject_pose_point(const CalibratedCamera& camera, const Point3& point,
			Matrix& D_intrinsic_pose, Matrix& D_intrinsic_point) {

		const Pose3& pose = camera.pose();
		const Rot3& R = pose.rotation();
		const Point3& r1 = R.r1(), r2 = R.r2(), r3 = R.r3();
		Point3 q = transform_to(pose, point);
		double X = q.x(), Y = q.y(), Z = q.z();
		double d = 1.0 / Z, d2 = d * d, Xd2 = X*d2, Yd2 = Y*d2;
		double dp11 = d*r1.x()-r3.x()*Xd2, dp12 = d*r1.y()-r3.y()*Xd2, dp13 = d*r1.z()-r3.z()*Xd2;
		double dp21 = d*r2.x()-r3.x()*Yd2, dp22 = d*r2.y()-r3.y()*Yd2, dp23 = d*r2.z()-r3.z()*Yd2;
		D_intrinsic_pose = Matrix_(2,6,
				X*Yd2, -Z*d-X*Xd2,  d*Y, -dp11, -dp12, -dp13,
				d*Z+Y*Yd2, -X*Yd2, -d*X, -dp21, -dp22, -dp23);
		D_intrinsic_point = Matrix_(2,3,
						dp11, dp12, dp13,
						dp21, dp22, dp23);
		return project_to_camera(q);
	}

/* ************************************************************************* */
}
