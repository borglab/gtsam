/**
 * @file    StereoCamera.h
 * @brief   A Stereo Camera based on two Simple Cameras
 * @author  Chris Beall
 */

#pragma once

#include "boost/tuple/tuple.hpp"
#include <gtsam/geometry/Cal3_S2.h>
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
		Cal3_S2 K_;
		double baseline_;
		double fx_, fy_;
		double cx_, cy_;
	public:
		StereoCamera() {
		}

		StereoCamera(const Pose3& leftCamPose, const Cal3_S2& K, double baseline);

		const Cal3_S2& K() const {
			return K_;
		}

		const Pose3& pose() const {
			return leftCamPose_;
		}

		const double baseline() const {
			return baseline_;
		}

		StereoPoint2 project(const Point3& point) const;

	};

	/** Dimensionality of the tangent space */
	inline size_t dim(const StereoCamera& obj) {
		return 6;
	}

  /** Exponential map around p0 */
  template<> inline StereoCamera expmap<StereoCamera>(const StereoCamera& p0, const Vector& d) {
    return StereoCamera(expmap(p0.pose(),d),p0.K(),p0.baseline());
  }

	inline StereoPoint2 project(const StereoCamera& camera, const Point3& point) {
		return camera.project(point);
	}

	Matrix Dproject_stereo_pose(const StereoCamera& camera, const Point3& point);
	Matrix Dproject_stereo_point(const StereoCamera& camera, const Point3& point);

	Matrix
			Dproject_to_stereo_camera1(const StereoCamera& camera, const Point3& P);
	Matrix Duncalibrate2(const Cal3_S2& K);

}
