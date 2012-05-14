/**
 * Matlab toolbox interface definition for gtsam_unstable
 */

// Most things are in the gtsam namespace
using namespace gtsam;

// specify the classes from gtsam we are using
class gtsam::Point3;
class gtsam::Rot3;
class gtsam::Pose3;
class gtsam::SharedNoiseModel;

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
	bool equals(const PoseRTV& other, double tol) const;
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
	PoseRTV retract(Vector v) const;
	Vector localCoordinates(const PoseRTV& p) const;
	static PoseRTV Expmap(Vector v);
	static Vector Logmap(const PoseRTV& p);
	PoseRTV inverse() const;
	PoseRTV compose(const PoseRTV& p) const;
	PoseRTV between(const PoseRTV& p) const;

	// measurement
	double range(const PoseRTV& other) const;
	PoseRTV transformed_from(const gtsam::Pose3& trans) const;

	// IMU/dynamics
	PoseRTV planarDynamics(double vel_rate, double heading_rate, double max_accel, double dt) const;
	PoseRTV flyingDynamics(double pitch_rate, double heading_rate, double lift_control, double dt) const;
	PoseRTV generalDynamics(Vector accel, Vector gyro, double dt) const;
	Vector imuPrediction(const PoseRTV& x2, double dt) const;
	gtsam::Point3 translationIntegration(const PoseRTV& x2, double dt) const;
	Vector translationIntegrationVec(const PoseRTV& x2, double dt) const;
};

#include <gtsam_unstable/dynamics/imuSystem.h>
namespace imu {

class Values {
	Values();
	void print(string s) const;

	void insertPose(size_t key, const PoseRTV& pose);
	PoseRTV pose(size_t key) const;
};

class Graph {
	Graph();
	void print(string s) const;

	// prior factors
	void addPrior(size_t key, const PoseRTV& pose, const gtsam::SharedNoiseModel& noiseModel);
	void addConstraint(size_t key, const PoseRTV& pose);
	void addHeightPrior(size_t key, double z, const gtsam::SharedNoiseModel& noiseModel);

	// inertial factors
	void addFullIMUMeasurement(size_t key1, size_t key2, const Vector& accel, const Vector& gyro, double dt, const gtsam::SharedNoiseModel& noiseModel);
	void addIMUMeasurement(size_t key1, size_t key2, const Vector& accel, const Vector& gyro, double dt, const gtsam::SharedNoiseModel& noiseModel);
	void addVelocityConstraint(size_t key1, size_t key2, double dt, const gtsam::SharedNoiseModel& noiseModel);

	// other measurements
	void addBetween(size_t key1, size_t key2, const PoseRTV& z, const gtsam::SharedNoiseModel& noiseModel);
	void addRange(size_t key1, size_t key2, double z, const gtsam::SharedNoiseModel& noiseModel);

	// optimization
	imu::Values optimize(const imu::Values& init) const;
};

}///\namespace imu
