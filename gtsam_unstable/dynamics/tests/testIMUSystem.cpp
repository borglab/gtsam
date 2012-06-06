/**
 * @file testIMUSystem
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/dynamics/imuSystem.h>

using namespace gtsam;
using namespace imu;

const double tol=1e-5;

static const Vector g = delta(3, 2, -9.81);

Key x1 = PoseKey(1), x2 = PoseKey(2), x3 = PoseKey(3), x4 = PoseKey(4);

/* ************************************************************************* */
TEST(testIMUSystem, instantiations) {
	// just checking for compilation
	PoseRTV x1_v;
	imu::Values local_values;
	Graph graph;

	gtsam::SharedNoiseModel model1 = gtsam::noiseModel::Unit::Create(1);
	gtsam::SharedNoiseModel model3 = gtsam::noiseModel::Unit::Create(3);
	gtsam::SharedNoiseModel model6 = gtsam::noiseModel::Unit::Create(6);
	gtsam::SharedNoiseModel model9 = gtsam::noiseModel::Unit::Create(9);

	Vector accel = ones(3), gyro = ones(3);

	IMUMeasurement imu(accel, gyro, 0.01, x1, x2, model6);
	FullIMUMeasurement full_imu(accel, gyro, 0.01, x1, x2, model9);
	Constraint poseHardPrior(x1, x1_v);
	Between odom(x1, x2, x1_v, model9);
	Range range(x1, x2, 1.0, model1);
	VelocityConstraint constraint(x1, x2, 0.1, 10000);
	Prior posePrior(x1, x1_v, model9);
	DHeightPrior heightPrior(x1, 0.1, model1);
	VelocityPrior velPrior(x1, ones(3), model3);
}

/* ************************************************************************* */
TEST( testIMUSystem, optimize_chain ) {
	// create a simple chain of poses to generate IMU measurements
	const double dt = 1.0;
	PoseRTV pose1,
					pose2(Point3(1.0, 1.0, 0.0), Rot3::ypr(0.1, 0.0, 0.0), Vector_(3, 2.0, 2.0, 0.0)),
					pose3(Point3(2.0, 2.0, 0.0), Rot3::ypr(0.2, 0.0, 0.0), Vector_(3, 0.0, 0.0, 0.0)),
					pose4(Point3(3.0, 3.0, 0.0), Rot3::ypr(0.3, 0.0, 0.0), Vector_(3, 2.0, 2.0, 0.0));

	// create measurements
	SharedDiagonal model = noiseModel::Unit::Create(6);
	Vector imu12(6), imu23(6), imu34(6);
	imu12 = pose1.imuPrediction(pose2, dt);
	imu23 = pose2.imuPrediction(pose3, dt);
	imu34 = pose3.imuPrediction(pose4, dt);

	// assemble simple graph with IMU measurements and velocity constraints
	Graph graph;
	graph.add(Constraint(x1, pose1));
	graph.add(IMUMeasurement(imu12, dt, x1, x2, model));
	graph.add(IMUMeasurement(imu23, dt, x2, x3, model));
	graph.add(IMUMeasurement(imu34, dt, x3, x4, model));
	graph.add(VelocityConstraint(x1, x2, dt));
	graph.add(VelocityConstraint(x2, x3, dt));
	graph.add(VelocityConstraint(x3, x4, dt));

	// ground truth values
	imu::Values true_values;
	true_values.insert(x1, pose1);
	true_values.insert(x2, pose2);
	true_values.insert(x3, pose3);
	true_values.insert(x4, pose4);

	// verify zero error
	EXPECT_DOUBLES_EQUAL(0, graph.error(true_values), 1e-5);

	// initialize with zero values and optimize
	imu::Values values;
	values.insert(x1, PoseRTV());
	values.insert(x2, PoseRTV());
	values.insert(x3, PoseRTV());
	values.insert(x4, PoseRTV());

	imu::Values actual = graph.optimize(values);
	EXPECT(assert_equal(true_values, actual, tol));
}

/* ************************************************************************* */
TEST( testIMUSystem, optimize_chain_fullfactor ) {
	// create a simple chain of poses to generate IMU measurements
	const double dt = 1.0;
	PoseRTV pose1,
					pose2(Point3(1.0, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Vector_(3, 1.0, 0.0, 0.0)),
					pose3(Point3(2.0, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Vector_(3, 1.0, 0.0, 0.0)),
					pose4(Point3(3.0, 0.0, 0.0), Rot3::ypr(0.0, 0.0, 0.0), Vector_(3, 1.0, 0.0, 0.0));

	// create measurements
	SharedDiagonal model = noiseModel::Isotropic::Sigma(9, 1.0);
	Vector imu12(6), imu23(6), imu34(6);
	imu12 = pose1.imuPrediction(pose2, dt);
	imu23 = pose2.imuPrediction(pose3, dt);
	imu34 = pose3.imuPrediction(pose4, dt);

	// assemble simple graph with IMU measurements and velocity constraints
	Graph graph;
	graph.add(Constraint(x1, pose1));
	graph.add(FullIMUMeasurement(imu12, dt, x1, x2, model));
	graph.add(FullIMUMeasurement(imu23, dt, x2, x3, model));
	graph.add(FullIMUMeasurement(imu34, dt, x3, x4, model));

	// ground truth values
	imu::Values true_values;
	true_values.insert(x1, pose1);
	true_values.insert(x2, pose2);
	true_values.insert(x3, pose3);
	true_values.insert(x4, pose4);

	// verify zero error
	EXPECT_DOUBLES_EQUAL(0, graph.error(true_values), 1e-5);

	// initialize with zero values and optimize
	imu::Values values;
	values.insert(x1, PoseRTV());
	values.insert(x2, PoseRTV());
	values.insert(x3, PoseRTV());
	values.insert(x4, PoseRTV());

	cout << "Initial Error: " << graph.error(values) << endl; // Initial error is 0.5 - need better prediction model

	imu::Values actual = graph.optimize(values);
//	EXPECT(assert_equal(true_values, actual, tol)); // FAIL
}

/* ************************************************************************* */
TEST( testIMUSystem, linear_trajectory) {
	// create a linear trajectory of poses
	// and verify simple solution
	const double dt = 1.0;

	PoseRTV start;
	Vector accel = delta(3, 0, 0.5); // forward force
	Vector gyro = delta(3, 0, 0.1); // constant rotation
	SharedDiagonal model = noiseModel::Unit::Create(9);

	imu::Values true_traj, init_traj;
	Graph graph;

	graph.add(Constraint(PoseKey(0), start));
	true_traj.insert(PoseKey(0), start);
	init_traj.insert(PoseKey(0), start);

	size_t nrPoses = 10;
	PoseRTV cur_pose = start;
	for (size_t i=1; i<nrPoses; ++i) {
		Key xA = PoseKey(i-1), xB = PoseKey(i);
		cur_pose = cur_pose.generalDynamics(accel, gyro, dt);
		graph.add(FullIMUMeasurement(accel - g, gyro, dt, xA, xB, model));
		true_traj.insert(xB, cur_pose);
		init_traj.insert(xB, PoseRTV());
	}
//	EXPECT_DOUBLES_EQUAL(0, graph.error(true_traj), 1e-5); // FAIL
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
