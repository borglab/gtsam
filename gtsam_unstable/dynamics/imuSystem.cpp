/**
 * @file ilm3DSystem.cpp
 * @brief Implementations of ilm 3D domain
 * @author Alex Cunningham
 */

#include <gtsam_unstable/dynamics/imuSystem.h>

#include <gtsam/nonlinear/NonlinearOptimization.h>

namespace imu {

using namespace gtsam;

/* ************************************************************************* */
void Graph::addPrior(Key key, const PoseRTV& pose, const SharedNoiseModel& noiseModel) {
	add(Prior(PoseKey(key), pose, noiseModel));
}

/* ************************************************************************* */
void Graph::addConstraint(Key key, const PoseRTV& pose) {
	add(Constraint(PoseKey(key), pose));
}

/* ************************************************************************* */
void Graph::addHeightPrior(Key key, double z, const SharedNoiseModel& noiseModel) {
	add(DHeightPrior(PoseKey(key), z, noiseModel));
}

/* ************************************************************************* */
void Graph::addFullIMUMeasurement(Key key1, Key key2,
		const Vector& accel, const Vector& gyro, double dt, const SharedNoiseModel& noiseModel) {
	add(FullIMUMeasurement(accel, gyro, dt, PoseKey(key1), PoseKey(key2), noiseModel));
}

/* ************************************************************************* */
void Graph::addIMUMeasurement(Key key1, Key key2,
		const Vector& accel, const Vector& gyro, double dt, const SharedNoiseModel& noiseModel) {
	add(IMUMeasurement(accel, gyro, dt, PoseKey(key1), PoseKey(key2), noiseModel));
}

/* ************************************************************************* */
void Graph::addVelocityConstraint(Key key1, Key key2, double dt, const SharedNoiseModel& noiseModel) {
	add(VelocityConstraint(PoseKey(key1), PoseKey(key2), dt, noiseModel));
}

/* ************************************************************************* */
void Graph::addBetween(Key key1, Key key2, const PoseRTV& z, const SharedNoiseModel& noiseModel) {
	add(Between(PoseKey(key1), PoseKey(key2), z, noiseModel));
}

/* ************************************************************************* */
void Graph::addRange(Key key1, Key key2, double z, const SharedNoiseModel& noiseModel) {
	add(Range(PoseKey(key1), PoseKey(key2), z, noiseModel));
}

/* ************************************************************************* */
Values Graph::optimize(const Values& init) const {
	return gtsam::optimize<Graph>(*this, init);
}
/* ************************************************************************* */

} // \namespace imu

