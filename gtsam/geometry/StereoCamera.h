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

#pragma once

#include "boost/tuple/tuple.hpp"
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Lie.h>
#include <gtsam/geometry/StereoPoint2.h>

namespace gtsam {

	/**
	 * A stereo camera class
	 */
	class StereoCamera {

	private:
		Pose3 leftCamPose_;
		Cal3_S2Stereo K_;

	public:
		StereoCamera() {
		}

		StereoCamera(const Pose3& leftCamPose, const Cal3_S2Stereo& K);

		const Cal3_S2Stereo& calibration() const {
			return K_;
		}

		const Pose3& pose() const {
			return leftCamPose_;
		}

		const double baseline() const {
			return K_.baseline();
		}

		StereoPoint2 project(const Point3& point,
				boost::optional<Matrix&> Dproject_stereo_pose = boost::none,
				boost::optional<Matrix&> Dproject_stereo_point = boost::none) const;

		/** Dimensionality of the tangent space */
		inline size_t dim() const {
			return 6;
		}

		/** Exponential map around p0 */
		inline StereoCamera expmap(const Vector& d) const {
			return StereoCamera(pose().expmap(d),calibration());
		}

		Vector logmap(const StereoCamera &camera) const {
				const Vector v1(leftCamPose_.logmap(camera.leftCamPose_));
				return v1;
		}

		bool equals (const StereoCamera &camera, double tol = 1e-9) const {
			return leftCamPose_.equals(camera.leftCamPose_, tol) &&
				   K_.equals(camera.K_, tol);
		}

		void print(const std::string& s = "") const {
			leftCamPose_.print(s + ".camera.") ;
			K_.print(s + ".calibration.") ;
		}

	private:
		/** utility functions */
		Matrix Dproject_to_stereo_camera1(const Point3& P) const;
		static Matrix Duncalibrate2(const Cal3_S2& K);

	};

}
