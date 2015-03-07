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

#include <boost/tuple/tuple.hpp>

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/StereoPoint2.h>

namespace gtsam {

/**
 * A stereo camera class, parameterize by left camera pose and stereo calibration
 * @addtogroup geometry
 */
class StereoCamera  : public DerivedValue<StereoCamera> {

private:
	Pose3 leftCamPose_;
	Cal3_S2Stereo::shared_ptr K_;

public:

	/// @name Standard Constructors
	/// @{

	StereoCamera() {
	}

	StereoCamera(const Pose3& leftCamPose, const Cal3_S2Stereo::shared_ptr K);

	const Cal3_S2Stereo::shared_ptr calibration() const {
		return K_;
	}

  /// @}
  /// @name Testable
  /// @{

	void print(const std::string& s = "") const {
		leftCamPose_.print(s + ".camera.");
		K_->print(s + ".calibration.");
	}

	bool equals(const StereoCamera &camera, double tol = 1e-9) const {
		return leftCamPose_.equals(camera.leftCamPose_, tol) && K_->equals(
				*camera.K_, tol);
	}

  /// @}
  /// @name Manifold
  /// @{

	/** Dimensionality of the tangent space */
	inline size_t dim() const {
		return 6;
	}

	/** Dimensionality of the tangent space */
	static inline size_t Dim() {
		return 6;
	}

	/// Updates a with tangent space delta
	inline StereoCamera retract(const Vector& v) const {
		return StereoCamera(pose().retract(v), K_);
	}

	/// Local coordinates of manifold neighborhood around current value
	inline Vector localCoordinates(const StereoCamera& t2) const {
		return Vector(leftCamPose_.localCoordinates(t2.leftCamPose_));
	}

	Pose3 between(const StereoCamera &camera,
			boost::optional<Matrix&> H1=boost::none,
			boost::optional<Matrix&> H2=boost::none) const {
		return leftCamPose_.between(camera.pose(), H1, H2);
	}

  /// @}
	/// @name Standard Interface
	/// @{

	const Pose3& pose() const {
		return leftCamPose_;
	}

	double baseline() const {
		return K_->baseline();
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

	Point3 backproject(const StereoPoint2& z) const {
		Vector measured = z.vector();
		double Z = K_->baseline()*K_->fx()/(measured[0]-measured[1]);
		double X = Z *(measured[0]- K_->px()) / K_->fx();
		double Y = Z *(measured[2]- K_->py()) / K_->fy();
		Point3 world_point = leftCamPose_.transform_from(Point3(X, Y, Z));
		return world_point;
	}

	/// @}

private:
	/** utility functions */
	Matrix Dproject_to_stereo_camera1(const Point3& P) const;

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
    ar & boost::serialization::make_nvp("StereoCamera",
       boost::serialization::base_object<Value>(*this));
	  ar & BOOST_SERIALIZATION_NVP(leftCamPose_);
		ar & BOOST_SERIALIZATION_NVP(K_);
	}

};

}
