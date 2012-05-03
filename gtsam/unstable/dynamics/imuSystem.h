/**
 * @file imuSystem.h
 * @brief A 3D Dynamic system domain as a demonstration of IMU factors
 * @author Alex Cunningham
 */

#pragma once

#include <boost/shared_ptr.hpp>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Key.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/RangeFactor.h>
#include <gtsam/slam/PartialPriorFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam2/dynamics/PoseRTV.h>
#include <gtsam2/dynamics/IMUFactor.h>
#include <gtsam2/dynamics/FullIMUFactor.h>
#include <gtsam2/dynamics/VelocityConstraint.h>
#include <gtsam2/dynamics/DynamicsPriors.h>

/**
 * This domain focuses on a single class of variables: PoseRTV, which
 * models a dynamic pose operating with IMU measurements and assorted priors.
 *
 * There are also partial priors that constraint certain components of the
 * poses, as well as between and range factors to model other between-pose
 * information.
 */
namespace imu {

using namespace gtsam;

// Values: just poses
inline Symbol PoseKey(Index j) { return Symbol('x', j); }

struct Values : public gtsam::Values {
	typedef gtsam::Values Base;

	Values() {}
	Values(const Values& values) : Base(values) {}
	Values(const Base& values) : Base(values) {}

	void insertPose(Index key, const PoseRTV& pose) { insert(PoseKey(key), pose); }
	PoseRTV pose(Index key) const { return at<PoseRTV>(PoseKey(key)); }
};

// factors
typedef IMUFactor<PoseRTV> IMUMeasurement; // IMU between measurements
typedef FullIMUFactor<PoseRTV> FullIMUMeasurement; // Full-state IMU between measurements
typedef BetweenFactor<PoseRTV> Between;   // full odometry (including velocity)
typedef NonlinearEquality<PoseRTV> Constraint;
typedef PriorFactor<PoseRTV> Prior;
typedef RangeFactor<PoseRTV, PoseRTV> Range;

// graph components
struct Graph : public NonlinearFactorGraph {
	typedef NonlinearFactorGraph Base;

	Graph() {}
	Graph(const Base& graph) : Base(graph) {}
	Graph(const Graph& graph) : Base(graph) {}

	// prior factors
	void addPrior(size_t key, const PoseRTV& pose, const SharedNoiseModel& noiseModel);
	void addConstraint(size_t key, const PoseRTV& pose);
	void addHeightPrior(size_t key, double z, const SharedNoiseModel& noiseModel);

	// inertial factors
	void addFullIMUMeasurement(size_t key1, size_t key2, const Vector& accel, const Vector& gyro, double dt, const SharedNoiseModel& noiseModel);
	void addIMUMeasurement(size_t key1, size_t key2, const Vector& accel, const Vector& gyro, double dt, const SharedNoiseModel& noiseModel);
	void addVelocityConstraint(size_t key1, size_t key2, double dt, const SharedNoiseModel& noiseModel);

	// other measurements
	void addBetween(size_t key1, size_t key2, const PoseRTV& z, const SharedNoiseModel& noiseModel);
	void addRange(size_t key1, size_t key2, double z, const SharedNoiseModel& noiseModel);

	// optimization
	Values optimize(const Values& init) const;
};

} // \namespace imu

