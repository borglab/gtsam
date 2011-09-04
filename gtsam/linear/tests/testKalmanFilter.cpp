/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testKalmanFilter.cpp
 *
 * Test simple linear Kalman filter on a moving 2D point
 *
 *  Created on: Sep 3, 2011
 *  @Author: Stephen Williams
 *  @Author: Frank Dellaert
 */

#include <gtsam/linear/KalmanFilter.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

/** Small 2D point class implemented as a Vector */
struct State: Vector {
	State(double x, double y) :
			Vector(Vector_(2, x, y)) {
	}
};

/* ************************************************************************* */
TEST( KalmanFilter, linear1 ) {

	// Create the controls and measurement properties for our example
	Matrix F = eye(2,2);
	Matrix B = eye(2,2);
	Vector u = Vector_(2, 1.0, 0.0);
	SharedDiagonal modelQ = noiseModel::Isotropic::Sigma(2, 0.1);
	Matrix Q = 0.01*eye(2,2);
	Matrix H = eye(2,2);
	State z1(1.0, 0.0);
	State z2(2.0, 0.0);
	State z3(3.0, 0.0);
	SharedDiagonal modelR = noiseModel::Isotropic::Sigma(2, 0.1);
	Matrix R = 0.01*eye(2,2);

	// Create the set of expected output TestValues
	State expected0(0.0, 0.0);
	Matrix P00 = 0.01*eye(2,2);

	State expected1(1.0, 0.0);
	Matrix P01 = P00 + Q;
	Matrix I11 = inverse(P01) + inverse(R);

	State expected2(2.0, 0.0);
	Matrix P12 = inverse(I11) + Q;
	Matrix I22 = inverse(P12) + inverse(R);

	State expected3(3.0, 0.0);
	Matrix P23 = inverse(I22) + Q;
	Matrix I33 = inverse(P23) + inverse(R);

	// Create the Kalman Filter initialization point
	State x_initial(0.0,0.0);
	SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2,0.1);

	// Create an KalmanFilter object
	KalmanFilter kalmanFilter(x_initial, P_initial);
	EXPECT(assert_equal(expected0,kalmanFilter.mean()));
	EXPECT(assert_equal(P00,kalmanFilter.covariance()));

	// Run iteration 1
	kalmanFilter.predict(F, B, u, modelQ);
	EXPECT(assert_equal(expected1,kalmanFilter.mean()));
	EXPECT(assert_equal(P01,kalmanFilter.covariance()));
	kalmanFilter.update(H,z1,modelR);
	EXPECT(assert_equal(expected1,kalmanFilter.mean()));
	EXPECT(assert_equal(I11,kalmanFilter.information()));

	// Run iteration 2
	kalmanFilter.predict(F, B, u, modelQ);
	EXPECT(assert_equal(expected2,kalmanFilter.mean()));
	kalmanFilter.update(H,z2,modelR);
	EXPECT(assert_equal(expected2,kalmanFilter.mean()));

	// Run iteration 3
	kalmanFilter.predict(F, B, u, modelQ);
	EXPECT(assert_equal(expected3,kalmanFilter.mean()));
	kalmanFilter.update(H,z3,modelR);
	EXPECT(assert_equal(expected3,kalmanFilter.mean()));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

