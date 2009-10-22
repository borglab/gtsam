/*
 * SimpleCamera.h
 *
 *  Created on: Aug 16, 2009
 *      Author: dellaert
 */

#ifndef SIMPLECAMERA_H_
#define SIMPLECAMERA_H_

#include "CalibratedCamera.h"
#include "Cal3_S2.h"

namespace gtsam {

	/**
	 * A simple camera class with a Cal3_S2 calibration
	 * Basically takes a Calibrated camera and adds calibration information
	 * to produce measurements in pixels.
	 * Not a sublass as a SimpleCamera *is not* a CalibratedCamera.
	 */
	class SimpleCamera {
	private:
		CalibratedCamera calibrated_; // Calibrated camera
		Cal3_S2 K_; // Calibration

	public:
		SimpleCamera(const Cal3_S2& K, const CalibratedCamera& calibrated);
		SimpleCamera(const Cal3_S2& K, const Pose3& pose);
		virtual ~SimpleCamera();

		const Pose3& pose() const {
			return calibrated_.pose();
		}

		const Cal3_S2& calibration() const {
			return K_;
		}


		/**
		 * project a 3d point to the camera and also check the depth
		 */
		std::pair<Point2,bool> projectSafe(const Point3& P) const;

		/**
		 * project a 3d point to the camera
		 */
		Point2 project(const Point3& P) const;

		/**
		 * Create a level camera at the given 2D pose and height
		 * @param pose2 specifies the location and viewing direction
		 * (theta 0 = looking in direction of positive X axis)
		 */
		static SimpleCamera level(const Cal3_S2& K, const Pose2& pose2, double height);

		// Friends
		friend Matrix Dproject_pose(const SimpleCamera& camera,  const Point3& point);
		friend Matrix Dproject_point(const SimpleCamera& camera, const Point3& point);
		friend Point2	Dproject_pose_point(const SimpleCamera& camera, const Point3& point,
					Matrix& D_projection_pose, Matrix& D_projection_point);

	};

	/* ************************************************************************* */
	// measurement functions and derivatives
	/* ************************************************************************* */

	/**
	 * This function receives the camera pose and the landmark location and
	 * returns the location the point is supposed to appear in the image
	 * as well as the sign of the depth.
	 */
	std::pair<Point2, bool> projectSafe(const SimpleCamera& camera, const Point3& point);

	/**
	 * This function receives the camera pose and the landmark location and
	 * returns the location the point is supposed to appear in the image
	 */
	Point2 project(const SimpleCamera& camera, const Point3& point);

	/**
	 * Derivatives of project.
	 */
	Matrix Dproject_pose(const SimpleCamera& camera, const Point3& point);
	Matrix Dproject_point(const SimpleCamera& camera, const Point3& point);

	/**
	 * super-duper combined evaluation + derivatives
	 * saves a lot of time because a lot of computation is shared
	 */
	Point2 Dproject_pose_point(const SimpleCamera& camera, const Point3& point,
					Matrix& D_projection_pose, Matrix& D_projection_point);

}

#endif /* SIMPLECAMERA_H_ */
