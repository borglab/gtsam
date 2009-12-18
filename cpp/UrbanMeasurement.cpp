/**
 * @file    UrbanMeasurement.cpp
 * @brief   A non-linear factor specialized to the Visual SLAM problem
 * @author  Frank Dellaert
 * @author  Viorela Ila
 */

#include "UrbanMeasurement.h"
#include "Pose3.h"

using namespace std;

namespace gtsam {

	/* ************************************************************************* */
	Point2 transform_to(const Pose3& pose, const Point2& p) {
		// create a 3D point at our height (Z is assumed up)
		Point3 global3D(p.x(),p.y(),pose.translation().z());
		// transform into local 3D point
		Point3 local3D = transform_to(pose,global3D);
		// take x and y as the local measurement
		Point2 local2D(local3D.x(),local3D.y());
		return local2D;
	}

	/* ************************************************************************* */
	Matrix Dtransform_to1(const Pose3& pose, const Point2& p) {
		return zeros(2, 6);
	}

	/* ************************************************************************* */
	Matrix Dtransform_to2(const Pose3& pose, const Point2& p) {
		return zeros(2, 2);
	}

	/* ************************************************************************* */
	UrbanMeasurement::UrbanMeasurement() {
		/// Arbitrary values
		robotPoseNumber_ = 111;
		landmarkNumber_ = 222;
		robotPoseName_ = symbol('x', robotPoseNumber_);
		landmarkName_ = symbol('l', landmarkNumber_);
		keys_.push_back(robotPoseName_);
		keys_.push_back(landmarkName_);
	}

	/* ************************************************************************* */
	UrbanMeasurement::UrbanMeasurement(const Point2& z, double sigma, int i,
			int j) :
		UrbanFactor(z.vector(), sigma) {
		robotPoseNumber_ = i;
		landmarkNumber_ = j;
		robotPoseName_ = symbol('x', robotPoseNumber_);
		landmarkName_ = symbol('l', landmarkNumber_);
		keys_.push_back(robotPoseName_);
		keys_.push_back(landmarkName_);
	}

	/* ************************************************************************* */
	void UrbanMeasurement::print(const std::string& s) const {
		printf("%s %s %s\n", s.c_str(), robotPoseName_.c_str(),
				landmarkName_.c_str());
		gtsam::print(this->z_, s + ".z");
	}

	/* ************************************************************************* */
	bool UrbanMeasurement::equals(const UrbanMeasurement& p, double tol) const {
		if (&p == NULL) return false;
		if (robotPoseNumber_ != p.robotPoseNumber_ || landmarkNumber_
				!= p.landmarkNumber_) return false;
		if (!equal_with_abs_tol(this->z_, p.z_, tol)) return false;
		return true;
	}

	/* ************************************************************************* */
	Vector UrbanMeasurement::error_vector(const UrbanConfig& x) const {
		Pose3 pose = x.robotPose(robotPoseNumber_);
		Point2 landmark = x.landmarkPoint(landmarkNumber_);
		// Right-hand-side b = (z - h(x))/sigma
		Point2 hx = transform_to(pose,landmark);
		return (z_ - hx.vector());
	}

/* ************************************************************************* */

} // namespace gtsam
