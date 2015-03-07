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
#include <gtsam/base/Testable.h>
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

	// Create a Kalman filter of dimension 2
	KalmanFilter kf1(2);

	// Create inital mean/covariance
	State x_initial(0.0, 0.0);
	SharedDiagonal P1 = noiseModel::Isotropic::Sigma(2, 0.1);

	// Get initial state by passing initial mean/covariance to the p
	KalmanFilter::State p1 = kf1.init(x_initial, P1);

	// Assert it has the correct mean, covariance and information
	EXPECT(assert_equal(x_initial, p1->mean()));
	Matrix Sigma = Matrix_(2, 2, 0.01, 0.0, 0.0, 0.01);
	EXPECT(assert_equal(Sigma, p1->covariance()));
	EXPECT(assert_equal(inverse(Sigma), p1->information()));

	// Create one with a sharedGaussian
	KalmanFilter::State p2 = kf1.init(x_initial, Sigma);
	EXPECT(assert_equal(Sigma, p2->covariance()));

	// Now make sure both agree
	EXPECT(assert_equal(p1->covariance(), p2->covariance()));
}

/* ************************************************************************* */
TEST( KalmanFilter, linear1 ) {

	// Create the controls and measurement properties for our example
	Matrix F = eye(2, 2);
	Matrix B = eye(2, 2);
	Vector u = Vector_(2, 1.0, 0.0);
	SharedDiagonal modelQ = noiseModel::Isotropic::Sigma(2, 0.1);
	Matrix Q = 0.01*eye(2, 2);
	Matrix H = eye(2, 2);
	State z1(1.0, 0.0);
	State z2(2.0, 0.0);
	State z3(3.0, 0.0);
	SharedDiagonal modelR = noiseModel::Isotropic::Sigma(2, 0.1);
	Matrix R = 0.01*eye(2, 2);

	// Create the set of expected output TestValues
	State expected0(0.0, 0.0);
	Matrix P00 = 0.01*eye(2, 2);

	State expected1(1.0, 0.0);
	Matrix P01 = P00 + Q;
	Matrix I11 = inverse(P01) + inverse(R);

	State expected2(2.0, 0.0);
	Matrix P12 = inverse(I11) + Q;
	Matrix I22 = inverse(P12) + inverse(R);

	State expected3(3.0, 0.0);
	Matrix P23 = inverse(I22) + Q;
	Matrix I33 = inverse(P23) + inverse(R);

	// Create a Kalman filter of dimension 2
	KalmanFilter kf(2);

	// Create the Kalman Filter initialization point
	State x_initial(0.0, 0.0);
	SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2, 0.1);

	// Create initial KalmanFilter object
	KalmanFilter::State p0 = kf.init(x_initial, P_initial);
	EXPECT(assert_equal(expected0, p0->mean()));
	EXPECT(assert_equal(P00, p0->covariance()));

	// Run iteration 1
	KalmanFilter::State p1p = kf.predict(p0, F, B, u, modelQ);
	EXPECT(assert_equal(expected1, p1p->mean()));
	EXPECT(assert_equal(P01, p1p->covariance()));

	KalmanFilter::State p1 = kf.update(p1p, H, z1, modelR);
	EXPECT(assert_equal(expected1, p1->mean()));
	EXPECT(assert_equal(I11, p1->information()));

	// Run iteration 2 (with full covariance)
	KalmanFilter::State p2p = kf.predictQ(p1, F, B, u, Q);
	EXPECT(assert_equal(expected2, p2p->mean()));

	KalmanFilter::State p2 = kf.update(p2p, H, z2, modelR);
	EXPECT(assert_equal(expected2, p2->mean()));

	// Run iteration 3
	KalmanFilter::State p3p = kf.predict(p2, F, B, u, modelQ);
	EXPECT(assert_equal(expected3, p3p->mean()));
	LONGS_EQUAL(3, KalmanFilter::step(p3p));

	KalmanFilter::State p3 = kf.update(p3p, H, z3, modelR);
	EXPECT(assert_equal(expected3, p3->mean()));
	LONGS_EQUAL(3, KalmanFilter::step(p3));
}

/* ************************************************************************* */
TEST( KalmanFilter, predict ) {

	// Create dynamics model
	Matrix F = Matrix_(2, 2, 1.0, 0.1, 0.2, 1.1);
	Matrix B = Matrix_(2, 3, 1.0, 0.1, 0.2, 1.1, 1.2, 0.8);
	Vector u = Vector_(3, 1.0, 0.0, 2.0);
	Matrix R = Matrix_(2, 2, 1.0, 0.5, 0.0, 3.0);
	Matrix M = trans(R)*R;
	Matrix Q = inverse(M);

	// Create a Kalman filter of dimension 2
	KalmanFilter kf(2);

	// Create the Kalman Filter initialization point
	State x_initial(0.0, 0.0);
	SharedDiagonal P_initial = noiseModel::Isotropic::Sigma(2, 1);

	// Create initial KalmanFilter state
	KalmanFilter::State p0 = kf.init(x_initial, P_initial);

	// Ensure predictQ and predict2 give same answer for non-trivial inputs
	KalmanFilter::State pa = kf.predictQ(p0, F, B, u, Q);
	// We have A1 = -F, A2 = I_, b = B*u, pre-multipled with R to match Q noise model
	Matrix A1 = -R*F, A2 = R;
	Vector b = R*B*u;
	SharedDiagonal nop = noiseModel::Isotropic::Sigma(2, 1.0);
	KalmanFilter::State pb = kf.predict2(p0, A1, A2, b, nop);
	EXPECT(assert_equal(pa->mean(), pb->mean()));
	EXPECT(assert_equal(pa->covariance(), pb->covariance()));
}

/* ************************************************************************* */
// Test both QR and Cholesky versions in case of a realistic (AHRS) dynamics update
TEST( KalmanFilter, QRvsCholesky ) {

	Vector mean = ones(9);
	Matrix covariance = 1e-6*Matrix_(9, 9,
			15.0, -6.2, 0.0, 0.0, 0.0, 0.0, 0.0, 63.8, -0.6,
			-6.2, 21.9, -0.0, 0.0, 0.0, 0.0, -63.8, -0.0, -0.1,
			0.0, -0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.1, -0.0,
			0.0, 0.0, 0.0, 23.4, 24.5, -0.6, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 24.5, 87.9, 10.1, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, -0.6, 10.1, 61.1, 0.0, 0.0, 0.0,
			0.0, -63.8, 0.0, 0.0, 0.0, 0.0, 625.0, 0.0, 0.0,
			63.8, -0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 625.0, 0.0,
			-0.6, -0.1, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 625.0);

	// Create two Kalman filter of dimension 9, one using QR the other Cholesky
	KalmanFilter kfa(9, KalmanFilter::QR), kfb(9, KalmanFilter::CHOLESKY);

	// create corresponding initial states
	KalmanFilter::State p0a = kfa.init(mean, covariance);
	KalmanFilter::State p0b = kfb.init(mean, covariance);

	// Set up dynamics update
	Matrix Psi_k = 1e-6*Matrix_(9, 9,
			1000000.0, 0.0, 0.0, -19200.0, 600.0, -0.0, 0.0, 0.0, 0.0,
			0.0, 1000000.0, 0.0, 600.0, 19200.0, 200.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1000000.0, -0.0, -200.0, 19200.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000.0);
	Matrix B = zeros(9, 1);
	Vector u = zero(1);
	Matrix dt_Q_k = 1e-6*Matrix_(9, 9,
			33.7, 3.1, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			3.1, 126.4, -0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			-0.0, -0.3, 88.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.2, 0.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.2, 0.0,
			0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.2);

	// Do prediction step
	KalmanFilter::State pa = kfa.predictQ(p0a, Psi_k, B, u, dt_Q_k);
	KalmanFilter::State pb = kfb.predictQ(p0b, Psi_k, B, u, dt_Q_k);

	// Check that they yield the same mean and information matrix
	EXPECT(assert_equal(pa->mean(), pb->mean()));
	EXPECT(assert_equal(pa->information(), pb->information(), 1e-7));

	// and in addition attain the correct covariance
	Vector expectedMean = Vector_(9, 0.9814, 1.0200, 1.0190, 1., 1., 1., 1., 1., 1.);
	EXPECT(assert_equal(expectedMean, pa->mean(), 1e-7));
	EXPECT(assert_equal(expectedMean, pb->mean(), 1e-7));
	Matrix expected = 1e-6*Matrix_(9, 9,
			48.8, -3.1, -0.0, -0.4, -0.4, 0.0, 0.0, 63.8, -0.6,
			-3.1, 148.4, -0.3, 0.5, 1.7, 0.2, -63.8, 0.0, -0.1,
			-0.0, -0.3, 188.0, -0.0, 0.2, 1.2, 0.0, 0.1, 0.0,
			-0.4, 0.5, -0.0, 23.6, 24.5, -0.6, 0.0, 0.0, 0.0,
			-0.4, 1.7, 0.2, 24.5, 88.1, 10.1, 0.0, 0.0, 0.0,
			0.0, 0.2, 1.2, -0.6, 10.1, 61.3, 0.0, 0.0, 0.0,
			0.0, -63.8, 0.0, 0.0, 0.0, 0.0, 647.2, 0.0, 0.0,
			63.8, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 647.2, 0.0,
			-0.6, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 647.2);
	EXPECT(assert_equal(expected, pa->covariance(), 1e-7));
	EXPECT(assert_equal(expected, pb->covariance(), 1e-7));

	// prepare update
	Matrix H = 1e-3*Matrix_(3, 9,
			0.0, 9795.9, 83.6, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
			-9795.9, 0.0, -5.2, 0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
			-83.6, 5.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.);
	Vector z = Vector_(3, 0.2599 , 1.3327 , 0.2007);
	Vector sigmas = Vector_(3, 0.3323 , 0.2470 , 0.1904);
	SharedDiagonal modelR = noiseModel::Diagonal::Sigmas(sigmas);

	// do update
	KalmanFilter::State pa2 = kfa.update(pa, H, z, modelR);
	KalmanFilter::State pb2 = kfb.update(pb, H, z, modelR);

	// Check that they yield the same mean and information matrix
	EXPECT(assert_equal(pa2->mean(), pb2->mean()));
	EXPECT(assert_equal(pa2->information(), pb2->information(), 1e-7));

	// and in addition attain the correct mean and covariance
	Vector expectedMean2 = Vector_(9, 0.9207, 0.9030, 1.0178, 1.0002, 0.9992, 0.9998, 0.9981, 1.0035, 0.9882);
	EXPECT(assert_equal(expectedMean2, pa2->mean(), 1e-4));// not happy with tolerance here !
	EXPECT(assert_equal(expectedMean2, pb2->mean(), 1e-4));// is something still amiss?
	Matrix expected2 = 1e-6*Matrix_(9, 9,
			46.1, -2.6, -0.0, -0.4, -0.4, 0.0, 0.0, 63.9, -0.5,
			-2.6, 132.8, -0.5, 0.4, 1.5, 0.2, -64.0, -0.0, -0.1,
			-0.0, -0.5, 188.0, -0.0, 0.2, 1.2, -0.0, 0.1, 0.0,
			-0.4, 0.4, -0.0, 23.6, 24.5, -0.6, -0.0, -0.0, -0.0,
			-0.4, 1.5, 0.2, 24.5, 88.1, 10.1, -0.0, -0.0, -0.0,
			0.0, 0.2, 1.2, -0.6, 10.1, 61.3, -0.0, 0.0, 0.0,
			0.0, -64.0, -0.0, -0.0, -0.0, -0.0, 647.2, -0.0, 0.0,
			63.9, -0.0, 0.1, -0.0, -0.0, 0.0, -0.0, 647.2, 0.1,
			-0.5, -0.1, 0.0, -0.0, -0.0, 0.0, 0.0, 0.1, 635.8);
	EXPECT(assert_equal(expected2, pa2->covariance(), 1e-7));
	EXPECT(assert_equal(expected2, pb2->covariance(), 1e-7));

	// do the above update again, this time with a full Matrix Q
	Matrix modelQ = diag(emul(sigmas,sigmas));
  KalmanFilter::State pa3 = kfa.updateQ(pa, H, z, modelQ);
  KalmanFilter::State pb3 = kfb.updateQ(pb, H, z, modelQ);

  // Check that they yield the same mean and information matrix
  EXPECT(assert_equal(pa3->mean(), pb3->mean()));
  EXPECT(assert_equal(pa3->information(), pb3->information(), 1e-7));

  // and in addition attain the correct mean and covariance
  EXPECT(assert_equal(expectedMean2, pa3->mean(), 1e-4));
  EXPECT(assert_equal(expectedMean2, pb3->mean(), 1e-4));

  EXPECT(assert_equal(expected2, pa3->covariance(), 1e-7));
  EXPECT(assert_equal(expected2, pb3->covariance(), 1e-7));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

