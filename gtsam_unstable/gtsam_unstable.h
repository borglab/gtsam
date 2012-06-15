/**
 * Matlab toolbox interface definition for gtsam_unstable
 */

// specify the classes from gtsam we are using
class gtsam::Point3;
class gtsam::Rot3;
class gtsam::Pose3;
class gtsam::SharedNoiseModel;

namespace gtsam {

#include <gtsam_unstable/base/Dummy.h>
class Dummy {
  Dummy();
  void print(string s) const;
};

#include <gtsam_unstable/dynamics/PoseRTV.h>
class PoseRTV {
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

}///\namespace gtsam

#include <gtsam_unstable/dynamics/imuSystem.h>
namespace imu {

class Values {
	Values();
	void print(string s) const;

	void insertPose(size_t key, const gtsam::PoseRTV& pose);
	gtsam::PoseRTV pose(size_t key) const;
};

class Graph {
	Graph();
	void print(string s) const;

	// prior factors
	void addPrior(size_t key, const gtsam::PoseRTV& pose, const gtsam::SharedNoiseModel& noiseModel);
	void addConstraint(size_t key, const gtsam::PoseRTV& pose);
	void addHeightPrior(size_t key, double z, const gtsam::SharedNoiseModel& noiseModel);

	// inertial factors
	void addFullIMUMeasurement(size_t key1, size_t key2, const Vector& accel, const Vector& gyro, double dt, const gtsam::SharedNoiseModel& noiseModel);
	void addIMUMeasurement(size_t key1, size_t key2, const Vector& accel, const Vector& gyro, double dt, const gtsam::SharedNoiseModel& noiseModel);
	void addVelocityConstraint(size_t key1, size_t key2, double dt, const gtsam::SharedNoiseModel& noiseModel);

	// other measurements
	void addBetween(size_t key1, size_t key2, const gtsam::PoseRTV& z, const gtsam::SharedNoiseModel& noiseModel);
	void addRange(size_t key1, size_t key2, double z, const gtsam::SharedNoiseModel& noiseModel);

	// optimization
	imu::Values optimize(const imu::Values& init) const;
};

}///\namespace imu
