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

#include <gtsam/base/Lie.h>
#include "boost/tuple/tuple.hpp"
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
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


		/*
		 * project 3D point and compute optional derivatives
		 */
		StereoPoint2 project(const Point3& point,
				boost::optional<Matrix&> H1 = boost::none,
				boost::optional<Matrix&> H2 = boost::none) const;

		/*
		 * to accomodate tsam's assumption that K is estimated, too
		 */
		StereoPoint2 project(const Point3& point,
					boost::optional<Matrix&> H1,
					boost::optional<Matrix&> H1_k,
					boost::optional<Matrix&> H2) const {
			return project(point, H1, H2);
		}

		/*
		 * backproject using left camera calibration, up to scale only
		 * i.e. does not rely on baseline
		 */
		Point3 backproject(const Point2& projection, const double scale) const {
			Point2 intrinsic = K_.calibrate(projection);
			Point3 cameraPoint = Point3(intrinsic.x() * scale, intrinsic.y() * scale, scale);;
			return pose().transform_from(cameraPoint);
		}

		/** Dimensionality of the tangent space */
		inline size_t dim() const {
			return 6;
		}

		/** Dimensionality of the tangent space */
		static inline size_t Dim() {
			return 6;
		}

		/** Exponential map around p0 */
		inline StereoCamera expmap(const Vector& d) const {
			return StereoCamera(pose().expmap(d), calibration());
		}

		Vector logmap(const StereoCamera &camera) const {
			const Vector v1(leftCamPose_.logmap(camera.leftCamPose_));
			return v1;
		}

		bool equals(const StereoCamera &camera, double tol = 1e-9) const {
			return leftCamPose_.equals(camera.leftCamPose_, tol) && K_.equals(
					camera.K_, tol);
		}

		Pose3 between(const StereoCamera &camera,
		    		boost::optional<Matrix&> H1=boost::none,
		    		boost::optional<Matrix&> H2=boost::none) const {
			return leftCamPose_.between(camera.pose(), H1, H2);
		}

		void print(const std::string& s = "") const {
			leftCamPose_.print(s + ".camera.");
			K_.print(s + ".calibration.");
		}

	private:
		/** utility functions */
		Matrix Dproject_to_stereo_camera1(const Point3& P) const;
		static Matrix Duncalibrate2(const Cal3_S2& K);

		friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version) {
			ar & BOOST_SERIALIZATION_NVP(leftCamPose_);
			ar & BOOST_SERIALIZATION_NVP(K_);
		}

	};

}
