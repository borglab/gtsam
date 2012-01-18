/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testKalmanFilter.cpp
 * @brief Test simple linear Kalman filter on a moving 2D point
 * @date Sep 3, 2011
 * @author Stephen Williams
 * @author Frank Dellaert
 */

#include <gtsam/linear/KalmanFilter.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/linear/SharedGaussian.h>
#include <gtsam/linear/SharedNoiseModel.h>
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
TEST( KalmanFilter, constructor ) {
	// Create the Kalman Filter initialization point
	State x_initial(0.0,0.0);
	SharedDiagonal P1 = noiseModel::Isotropic::Sigma(2,0.1);

	// Create an KalmanFilter object
	KalmanFilter kf1(x_initial, P1);
	Matrix Sigma = Matrix_(2,2,0.01,0.0,0.0,0.01);
	EXPECT(assert_equal(Sigma,kf1.covariance()));

	// Create one with a sharedGaussian
	KalmanFilter kf2(x_initial, Sigma);
	EXPECT(assert_equal(Sigma,kf2.covariance()));

	// Now make sure both agree
	EXPECT(assert_equal(kf1.covariance(),kf2.covariance()));
}

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

	// Run iteration 2 (with full covariance)
	kalmanFilter.predictQ(F, B, u, Q);
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
TEST( KalmanFilter, predict ) {

	// Create dynamics model
	Matrix F = Matrix_(2,2, 1.0,0.1, 0.2,1.1);
	Matrix B = Matrix_(2,3, 1.0,0.1,0.2, 1.1,1.2,0.8);
	Vector u = Vector_(3, 1.0, 0.0, 2.0);
	Matrix R = Matrix_(2,2, 1.0,0.0, 0.0,3.0);
	Matrix M = trans(R)*R;
	Matrix Q = inverse(M);

	// Create the Kalman Filter initialization point
	State x_initial(0.0,0.0);
	SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2,1);

	// Create two KalmanFilter objects
	KalmanFilter kalmanFilter1(x_initial, P_initial);
	KalmanFilter kalmanFilter2(x_initial, P_initial);

	// Ensure predictQ and predict2 give same answer for non-trivial inputs
	kalmanFilter1.predictQ(F, B, u, Q);
	// We have A1 = -F,  A2 = I_, b = B*u:
	Matrix A1 = -R*F, A2 = R;
	Vector b = R*B*u;
	SharedDiagonal nop = noiseModel::Isotropic::Sigma(2, 1.0);
	kalmanFilter2.predict2(A1,A2,b,nop);
	EXPECT(assert_equal(kalmanFilter1.mean(),kalmanFilter2.mean()));
	EXPECT(assert_equal(kalmanFilter1.covariance(),kalmanFilter2.covariance()));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

