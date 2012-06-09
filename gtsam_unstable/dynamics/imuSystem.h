/**
 * @file imuSystem.h
 * @brief A 3D Dynamic system domain as a demonstration of IMU factors
 * @author Alex Cunningham
 */

#pragma once

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/PartialPriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_unstable/dynamics/PoseRTV.h>
#include <gtsam_unstable/dynamics/IMUFactor.h>
#include <gtsam_unstable/dynamics/FullIMUFactor.h>
#include <gtsam_unstable/dynamics/VelocityConstraint.h>
#include <gtsam_unstable/dynamics/DynamicsPriors.h>

/**
 * This domain focuses on a single class of variables: PoseRTV, which
 * models a dynamic pose operating with IMU measurements and assorted priors.
 *
 * There are also partial priors that constraint certain components of the
 * poses, as well as between and range factors to model other between-pose
 * information.
 */
namespace imu {

struct Values : public gtsam::Values {
	typedef gtsam::Values Base;

	Values() {}
	Values(const Values& values) : Base(values) {}
	Values(const Base& values) : Base(values) {}

	void insertPose(gtsam::Key key, const gtsam::PoseRTV& pose) { insert(key, pose); }
	gtsam::PoseRTV pose(gtsam::Key key) const { return at<gtsam::PoseRTV>(key); }
};

// factors
typedef gtsam::IMUFactor<gtsam::PoseRTV> IMUMeasurement; // IMU between measurements
typedef gtsam::FullIMUFactor<gtsam::PoseRTV> FullIMUMeasurement; // Full-state IMU between measurements
typedef gtsam::BetweenFactor<gtsam::PoseRTV> Between;   // full odometry (including velocity)
typedef gtsam::NonlinearEquality<gtsam::PoseRTV> Constraint;
typedef gtsam::PriorFactor<gtsam::PoseRTV> Prior;
typedef gtsam::RangeFactor<gtsam::PoseRTV, gtsam::PoseRTV> Range;

// graph components
struct Graph : public gtsam::NonlinearFactorGraph {
	typedef gtsam::NonlinearFactorGraph Base;

	Graph() {}
	Graph(const Base& graph) : Base(graph) {}
	Graph(const Graph& graph) : Base(graph) {}

	// prior factors
	void addPrior(size_t key, const gtsam::PoseRTV& pose, const gtsam::SharedNoiseModel& noiseModel);
	void addConstraint(size_t key, const gtsam::PoseRTV& pose);
	void addHeightPrior(size_t key, double z, const gtsam::SharedNoiseModel& noiseModel);

	// inertial factors
	void addFullIMUMeasurement(size_t key1, size_t key2, const gtsam::Vector& accel, const gtsam::Vector& gyro, double dt, const gtsam::SharedNoiseModel& noiseModel);
	void addIMUMeasurement(size_t key1, size_t key2, const gtsam::Vector& accel, const gtsam::Vector& gyro, double dt, const gtsam::SharedNoiseModel& noiseModel);
	void addVelocityConstraint(size_t key1, size_t key2, double dt, const gtsam::SharedNoiseModel& noiseModel);

	// other measurements
	void addBetween(size_t key1, size_t key2, const gtsam::PoseRTV& z, const gtsam::SharedNoiseModel& noiseModel);
	void addRange(size_t key1, size_t key2, double z, const gtsam::SharedNoiseModel& noiseModel);

	// optimization
	Values optimize(const Values& init) const;
};

} // \namespace imu

