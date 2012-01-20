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

	// Create initial KalmanFilter object
	KalmanFilter KF0(x_initial, P_initial);
	EXPECT(assert_equal(expected0,KF0.mean()));
	EXPECT(assert_equal(P00,KF0.covariance()));

	// Run iteration 1
	KalmanFilter KF1p = KF0.predict(F, B, u, modelQ);
	EXPECT(assert_equal(expected1,KF1p.mean()));
	EXPECT(assert_equal(P01,KF1p.covariance()));

	KalmanFilter KF1 = KF1p.update(H,z1,modelR);
	EXPECT(assert_equal(expected1,KF1.mean()));
	EXPECT(assert_equal(I11,KF1.information()));

	// Run iteration 2 (with full covariance)
	KalmanFilter KF2p = KF1.predictQ(F, B, u, Q);
	EXPECT(assert_equal(expected2,KF2p.mean()));

	KalmanFilter KF2 = KF2p.update(H,z2,modelR);
	EXPECT(assert_equal(expected2,KF2.mean()));

	// Run iteration 3
	KalmanFilter KF3p = KF2.predict(F, B, u, modelQ);
	EXPECT(assert_equal(expected3,KF3p.mean()));

	KalmanFilter KF3 = KF3p.update(H,z3,modelR);
	EXPECT(assert_equal(expected3,KF3.mean()));
}

/* ************************************************************************* */
TEST( KalmanFilter, predict ) {

	// Create dynamics model
	Matrix F = Matrix_(2,2, 1.0,0.1, 0.2,1.1);
	Matrix B = Matrix_(2,3, 1.0,0.1,0.2, 1.1,1.2,0.8);
	Vector u = Vector_(3, 1.0, 0.0, 2.0);
	Matrix R = Matrix_(2,2, 1.0,0.5, 0.0,3.0);
	Matrix M = trans(R)*R;
	Matrix Q = inverse(M);

	// Create the Kalman Filter initialization point
	State x_initial(0.0,0.0);
	SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2,1);

	// Create two KalmanFilter objects
	KalmanFilter KF0(x_initial, P_initial);

	// Ensure predictQ and predict2 give same answer for non-trivial inputs
	KalmanFilter KFa = KF0.predictQ(F, B, u, Q);
	// We have A1 = -F,  A2 = I_, b = B*u, pre-multipled with R to match Q noise model
	Matrix A1 = -R*F, A2 = R;
	Vector b = R*B*u;
	SharedDiagonal nop = noiseModel::Isotropic::Sigma(2, 1.0);
	KalmanFilter KFb = KF0.predict2(A1,A2,b,nop);
	EXPECT(assert_equal(KFa.mean(),KFb.mean()));
	EXPECT(assert_equal(KFa.covariance(),KFb.covariance()));
}

/* ************************************************************************* */
// Test both QR and LDL versions in case of a realistic (AHRS) dynamics update
TEST( KalmanFilter, QRvsCholesky ) {

	Vector mean = zero(9);
	Matrix covariance = 1e-6*Matrix_(9,9,
			15.0, -6.2, 0.0, 0.0, 0.0, 0.0, 0.0, 63.8, -0.6,
			-6.2, 21.9, -0.0, 0.0, 0.0, 0.0, -63.8, -0.0, -0.1,
			0.0, -0.0, 10000000.0, 0.0, 0.0, 0.0, 0.0, 0.1, -0.0,
			0.0, 0.0, 0.0, 23.4, 24.5, -0.6, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 24.5, 87.9, 10.1, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, -0.6, 10.1, 61.1, 0.0, 0.0, 0.0,
			0.0, -63.8, 0.0, 0.0, 0.0, 0.0, 625.0, 0.0, 0.0,
			63.8, -0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 625.0, 0.0,
			-0.6, -0.1, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 625.0);

	// Set up dynamics update
	Matrix Psi_k = 1e-6*Matrix_(9,9,
			1000000.0, 0.0, 0.0, -19189.0, 277.7, -13.0, 0.0, 0.0, 0.0,
			0.0, 1000000.0, 0.0, 277.6, 19188.3, 164.1, 0.0, 0.0, 0.0,
			0.0, 0.0, 1000000.0, -15.4, -163.9, 19190.3, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 999973.7, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 999973.7, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 999973.7, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999973.7, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999973.7, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 999973.7);
	Matrix B = zeros(9,1);
	Vector u = zero(1);
	Matrix dt_Q_k = 1e-6*Matrix_(9,9,
			33.6, 1.3, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			1.3, 126.5, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			-0.0, -0.3, 88.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.2, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.2, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.2);

	// Create two KalmanFilter using different factorization method and compare
	KalmanFilter KFa = KalmanFilter(mean, covariance,KalmanFilter::QR).predictQ(Psi_k,B,u,dt_Q_k);
	KalmanFilter KFb = KalmanFilter(mean, covariance,KalmanFilter::LDL).predictQ(Psi_k,B,u,dt_Q_k);
	EXPECT(assert_equal(KFa.mean(),KFb.mean()));
//	EXPECT(assert_equal(KFa.information(),KFb.information()));
//	EXPECT(assert_equal(KFa.covariance(),KFb.covariance()));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

