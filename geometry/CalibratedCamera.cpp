/*
 * CalibratedCamera.cpp
 *
 *  Created on: Aug 17, 2009
 *      Author: dellaert
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/CalibratedCamera.h>

namespace gtsam {

	CalibratedCamera()::CalibratedCamera() {}

	CalibratedCamera::CalibratedCamera(const Pose3& pose) :
		pose_(pose) {
	}

	CalibratedCamera::CalibratedCamera(const Vector &v) : pose_(Pose3::Expmap(v)) {}

	CalibratedCamera::~CalibratedCamera() {}

	Point2 CalibratedCamera::project_to_camera(const Point3& P, boost::optional<Matrix&> H1) {
		if (H1) {
			double d = 1.0 / P.z(), d2 = d * d;
			*H1 = Matrix_(2, 3, d, 0.0, -P.x() * d2, 0.0, d, -P.y() * d2);
		}
		return Point2(P.x() / P.z(), P.y() / P.z());
	}

	Point3 CalibratedCamera::backproject_from_camera(const Point2& p, const double scale) {
		return Point3(p.x() * scale, p.y() * scale, scale);
	}

	CalibratedCamera CalibratedCamera::level(const Pose2& pose2, double height) {
		double st = sin(pose2.theta()), ct = cos(pose2.theta());
		Point3 x(st, -ct, 0), y(0, 0, -1), z(ct, st, 0);
		Rot3 wRc(x, y, z);
		Point3 t(pose2.x(), pose2.y(), height);
		Pose3 pose3(wRc, t);
		return CalibratedCamera(pose3);
	}

	Point2 CalibratedCamera::project(const Point3& point,
		    boost::optional<Matrix&> D_intrinsic_pose,
		    boost::optional<Matrix&> D_intrinsic_point) const {
		const Pose3& pose = pose_;
		const Rot3& R = pose.rotation();
		const Point3& r1 = R.r1(), r2 = R.r2(), r3 = R.r3();
		Point3 q = pose.transform_to(point);

		if (D_intrinsic_pose || D_intrinsic_point) {
			double X = q.x(), Y = q.y(), Z = q.z();
			double d = 1.0 / Z, d2 = d * d, Xd2 = X*d2, Yd2 = Y*d2;
			double dp11 = d*r1.x()-r3.x()*Xd2, dp12 = d*r1.y()-r3.y()*Xd2, dp13 = d*r1.z()-r3.z()*Xd2;
			double dp21 = d*r2.x()-r3.x()*Yd2, dp22 = d*r2.y()-r3.y()*Yd2, dp23 = d*r2.z()-r3.z()*Yd2;
			if (D_intrinsic_pose)
				*D_intrinsic_pose = Matrix_(2,6,
						X*Yd2, -Z*d-X*Xd2,  d*Y, -dp11, -dp12, -dp13,
						d*Z+Y*Yd2, -X*Yd2, -d*X, -dp21, -dp22, -dp23);
			if (D_intrinsic_point)
				*D_intrinsic_point = Matrix_(2,3,
						dp11, dp12, dp13,
						dp21, dp22, dp23);
		}
		return project_to_camera(q);
	}
/* ************************************************************************* */
}
