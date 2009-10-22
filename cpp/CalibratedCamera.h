/*
 * CalibratedCamera.h
 *
 *  Created on: Aug 17, 2009
 *      Author: dellaert
 */

#ifndef CalibratedCAMERA_H_
#define CalibratedCAMERA_H_

#include "Pose2.h"
#include "Pose3.h"

namespace gtsam {

	class Point2;

	/**
	 * projects a 3-dimensional point in camera coordinates into the
	 * camera and returns a 2-dimensional point, no calibration applied
	 */
	Point2 project_to_camera(const Point3& cameraPoint);

	/**
	 * Derivative of project_to_camera
	 */
	Matrix Dproject_to_camera1(const Point3& cameraPoint); /*2by3 <--*/

	/**
	 * A Calibrated camera class [R|-R't], calibration K=I.
	 * If calibration is known, it is more computationally efficient
	 * to calibrate the measurements rather than try to predict in pixels.
	 */
	class CalibratedCamera {
	private:
		Pose3 pose_; // 6DOF pose

	public:
		CalibratedCamera(const Pose3& pose);
		virtual ~CalibratedCamera();

		const Pose3& pose() const {
			return pose_;
		}

		/**
		 * Create a level camera at the given 2D pose and height
		 * @param pose2 specifies the location and viewing direction
		 * (theta 0 = looking in direction of positive X axis)
		 */
		static CalibratedCamera level(const Pose2& pose2, double height);

		Point2 project(const Point3& P) const;
	};

	/* ************************************************************************* */
	// measurement functions and derivatives
	/* ************************************************************************* */

	/**
	 * This function receives the camera pose and the landmark location and
	 * returns the location the point is supposed to appear in the image
	 * @param camera the CalibratedCamera
	 * @param point a 3D point to be projected
	 * @return the intrinsic coordinates of the projected point
	 */
	Point2 project(const CalibratedCamera& camera, const Point3& point);

	/**
	 * Derivatives of project, same paramaters as project
	 */
	Matrix Dproject_pose(const CalibratedCamera& camera, const Point3& point);
	Matrix Dproject_point(const CalibratedCamera& camera, const Point3& point);

	/**
	 * super-duper combined evaluation + derivatives
	 * saves a lot of time because a lot of computation is shared
	 * @return project(camera,point)
	 */
	Point2 Dproject_pose_point(const CalibratedCamera& camera,
			const Point3& point, Matrix& D_intrinsic_pose, Matrix& D_intrinsic_point);
}

#endif /* CalibratedCAMERA_H_ */
