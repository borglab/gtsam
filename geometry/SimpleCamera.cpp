/*
 * SimpleCamera.cpp
 *
 *  Created on: Aug 16, 2009
 *      Author: dellaert
 */

#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/CalibratedCamera.h>

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
		Point3 cameraPoint = calibrated_.pose().transform_to(P);
		Point2 intrinsic = CalibratedCamera::project_to_camera(cameraPoint);
		Point2 projection = K_.uncalibrate(intrinsic);
		return pair<Point2, bool>(projection, cameraPoint.z() > 0);
	}

	Point2 SimpleCamera::project(const Point3& point,
		    boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

		Point2 intrinsic = calibrated_.project(point, H1, H2);
		if (!H1 && !H2)
			return K_.uncalibrate(intrinsic);

		Matrix D_projection_intrinsic;
		Point2 projection = K_.uncalibrate(intrinsic, boost::none, D_projection_intrinsic);

		if (H1) {
			*H1 = D_projection_intrinsic * (*H1);
		}
		if (H2) {
			*H2 = D_projection_intrinsic * (*H2);
		}
		return projection;
	}

	Point3 SimpleCamera::backproject(const Point2& projection, const double scale) const {
		Point2 intrinsic = K_.calibrate(projection);
		Point3 cameraPoint = CalibratedCamera::backproject_from_camera(intrinsic, scale);
		return calibrated_.pose().transform_from(cameraPoint);
	}

	SimpleCamera SimpleCamera::level(const Cal3_S2& K, const Pose2& pose2, double height) {
		return SimpleCamera(K, CalibratedCamera::level(pose2, height));
	}

/* ************************************************************************* */
}
