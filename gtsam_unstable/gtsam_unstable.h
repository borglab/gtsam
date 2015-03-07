/**
 * Matlab toolbox interface definition for gtsam_unstable
 */

// specify the classes from gtsam we are using
virtual class gtsam::Value;
virtual class gtsam::Point3;
virtual class gtsam::Rot3;
virtual class gtsam::Pose2;
virtual class gtsam::Pose3;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;

namespace gtsam {

#include <gtsam_unstable/base/Dummy.h>
class Dummy {
  Dummy();
  void print(string s) const;
  unsigned char dummyTwoVar(unsigned char a) const;
};

#include <gtsam_unstable/dynamics/PoseRTV.h>
virtual class PoseRTV : gtsam::Value {
	PoseRTV();
	PoseRTV(Vector rtv);
	PoseRTV(const gtsam::Point3& pt, const gtsam::Rot3& rot, const gtsam::Point3& vel);
	PoseRTV(const gtsam::Rot3& rot, const gtsam::Point3& pt, const gtsam::Point3& vel);
	PoseRTV(const gtsam::Pose3& pose, const gtsam::Point3& vel);
	PoseRTV(const gtsam::Pose3& pose);
	PoseRTV(double roll, double pitch, double yaw, double x, double y, double z, double vx, double vy, double vz);

	// testable
	bool equals(const gtsam::PoseRTV& other, double tol) const;
	void print(string s) const;

	// access
	gtsam::Point3 translation() const;
	gtsam::Rot3 rotation() const;
	gtsam::Point3 velocity() const;
	gtsam::Pose3 pose() const;

	// Vector interfaces
	Vector vector() const;
	Vector translationVec() const;
	Vector velocityVec() const;

	// manifold/Lie
	static size_t Dim();
	size_t dim() const;
	gtsam::PoseRTV retract(Vector v) const;
	Vector localCoordinates(const gtsam::PoseRTV& p) const;
	static gtsam::PoseRTV Expmap(Vector v);
	static Vector Logmap(const gtsam::PoseRTV& p);
	gtsam::PoseRTV inverse() const;
	gtsam::PoseRTV compose(const gtsam::PoseRTV& p) const;
	gtsam::PoseRTV between(const gtsam::PoseRTV& p) const;

	// measurement
	double range(const gtsam::PoseRTV& other) const;
	gtsam::PoseRTV transformed_from(const gtsam::Pose3& trans) const;

	// IMU/dynamics
	gtsam::PoseRTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;
	gtsam::PoseRTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;
	gtsam::PoseRTV generalDynamics(Vector accel, Vector gyro, double dt) const;
	Vector imuPrediction(const gtsam::PoseRTV& x2, double dt) const;
	gtsam::Point3 translationIntegration(const gtsam::PoseRTV& x2, double dt) const;
	Vector translationIntegrationVec(const gtsam::PoseRTV& x2, double dt) const;
};


// Nonlinear factors from gtsam, for our Value types
#include <gtsam/slam/PriorFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class PriorFactor : gtsam::NonlinearFactor {
	PriorFactor(size_t key, const T& prior, const gtsam::noiseModel::Base* noiseModel);
};


#include <gtsam/slam/BetweenFactor.h>
template<T = {gtsam::PoseRTV}>
virtual class BetweenFactor : gtsam::NonlinearFactor {
	BetweenFactor(size_t key1, size_t key2, const T& relativePose, const gtsam::noiseModel::Base* noiseModel);
};


#include <gtsam/slam/RangeFactor.h>
template<POSE, POINT>
virtual class RangeFactor : gtsam::NonlinearFactor {
	RangeFactor(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> RangeFactorRTV;


#include <gtsam/nonlinear/NonlinearEquality.h>
template<T = {gtsam::PoseRTV}>
virtual class NonlinearEquality : gtsam::NonlinearFactor {
	// Constructor - forces exact evaluation
	NonlinearEquality(size_t j, const T& feasible);
	// Constructor - allows inexact evaluation
	NonlinearEquality(size_t j, const T& feasible, double error_gain);
};

#include <gtsam_unstable/dynamics/IMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class IMUFactor : gtsam::NonlinearFactor {
	/** Standard constructor */
	IMUFactor(Vector accel, Vector gyro,
		double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

	/** Full IMU vector specification */
	IMUFactor(Vector imu_vector,
		double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

	Vector gyro() const;
	Vector accel() const;
	Vector z() const;
	size_t key1() const;
	size_t key2() const;
};

#include <gtsam_unstable/dynamics/FullIMUFactor.h>
template<POSE = {gtsam::PoseRTV}>
virtual class FullIMUFactor : gtsam::NonlinearFactor {
	/** Standard constructor */
	FullIMUFactor(Vector accel, Vector gyro,
		double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

	/** Single IMU vector - imu = [accel, gyro] */
	FullIMUFactor(Vector imu,
		double dt, size_t key1, size_t key2, const gtsam::noiseModel::Base* model);

	Vector gyro() const;
	Vector accel() const;
	Vector z() const;
	size_t key1() const;
	size_t key2() const;
};


#include <gtsam_unstable/dynamics/DynamicsPriors.h>
virtual class DHeightPrior : gtsam::NonlinearFactor {
	DHeightPrior(size_t key, double height, const gtsam::noiseModel::Base* model);
};


virtual class DRollPrior : gtsam::NonlinearFactor {
	/** allows for explicit roll parameterization - uses canonical coordinate */
	DRollPrior(size_t key, double wx, const gtsam::noiseModel::Base* model);
	/** Forces roll to zero */
	DRollPrior(size_t key, const gtsam::noiseModel::Base* model);
};


virtual class VelocityPrior : gtsam::NonlinearFactor {
	VelocityPrior(size_t key, Vector vel, const gtsam::noiseModel::Base* model);
};


virtual class DGroundConstraint : gtsam::NonlinearFactor {
	// Primary constructor allows for variable height of the "floor"
  DGroundConstraint(size_t key, double height, const gtsam::noiseModel::Base* model);
	// Fully specify vector - use only for debugging
  DGroundConstraint(size_t key, Vector constraint, const gtsam::noiseModel::Base* model);
};

//*************************************************************************
// General nonlinear factor types
//*************************************************************************
#include <gtsam/geometry/Pose2.h>

#include <gtsam_unstable/slam/PoseTranslationPrior.h>
template<POSE>
virtual class PoseTranslationPrior : gtsam::NonlinearFactor {
	PoseTranslationPrior(size_t key, const POSE& pose_z, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseTranslationPrior<gtsam::Pose2> PoseTranslationPrior2D;
typedef gtsam::PoseTranslationPrior<gtsam::Pose3> PoseTranslationPrior3D;

#include <gtsam_unstable/slam/PoseRotationPrior.h>
template<POSE>
virtual class PoseRotationPrior : gtsam::NonlinearFactor {
	PoseRotationPrior(size_t key, const POSE& pose_z, const gtsam::noiseModel::Base* noiseModel);
};

typedef gtsam::PoseRotationPrior<gtsam::Pose2> PoseRotationPrior2D;
typedef gtsam::PoseRotationPrior<gtsam::Pose3> PoseRotationPrior3D;

#include <gtsam_unstable/slam/RelativeElevationFactor.h>
virtual class RelativeElevationFactor: gtsam::NonlinearFactor {
	RelativeElevationFactor();
	RelativeElevationFactor(size_t poseKey, size_t pointKey, double measured,
			const gtsam::noiseModel::Base* model);

	double measured() const;
	void print(string s) const;
};

} //\namespace gtsam
