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
 *  Created on: Aug 19, 2011
 *  @Author: Stephen Williams
 *  @Author: Frank Dellaert
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/JacobianFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

class KalmanFilter {
private:

	size_t n_; /** dimensionality of state */
	Matrix I_; /** identity matrix of size n*n */

	/** The Kalman filter posterior density is a Gaussian Conditional with no parents */
	GaussianConditional::shared_ptr density_;

	/**
	 * solve a factor graph fragment and store result as density_
	 */
	void solve(GaussianFactorGraph& factorGraph) {

		// Solve the factor graph
		GaussianSequentialSolver solver(factorGraph);
		GaussianBayesNet::shared_ptr bayesNet = solver.eliminate();

		// As this is a filter, all we need is the posterior P(x_t),
		// so we just keep the root of the Bayes net
		// We need to create a new density, because we always keep the index at 0
		const GaussianConditional::shared_ptr& root = bayesNet->back();
		density_.reset(
				new GaussianConditional(0, root->get_d(), root->get_R(),
						root->get_sigmas()));
	}

public:

	/**
	 * Constructor from prior density at time k=0
	 * In Kalman Filter notation, these are is x_{0|0} and P_{0|0}
	 * @param x estimate at time 0
	 * @param P covariance at time 0, restricted to diagonal Gaussian 'model' for now
	 *
	 */
	KalmanFilter(const Vector& x, const SharedDiagonal& model) :
			n_(x.size()), I_(eye(n_, n_)) {

		// Create a factor graph f(x0), eliminate it into P(x0)
		GaussianFactorGraph factorGraph;
		factorGraph.add(0, I_, x, model);
		solve(factorGraph);
	}

	/**
	 * Return mean of posterior P(x|Z) at given all measurements Z
	 */
	Vector mean() const {
		// Solve for mean
		Index nVars = 1;
		VectorValues x(nVars, n_);
		density_->rhs(x);
		density_->solveInPlace(x);
		return x[0];
	}

	/**
	 * Return information matrix of posterior P(x|Z) at given all measurements Z
	 */
	Matrix information() const {
		return density_->computeInformation();
	}

	/**
	 * Return covariance of posterior P(x|Z) at given all measurements Z
	 */
	Matrix covariance() const {
		return inverse(information());
	}

	/**
	 * Predict the state P(x_{t+1}|Z^t)
	 *   In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
	 *   After the call, that is the density that can be queried.
	 * Details and parameters:
	 *   In a linear Kalman Filter, the motion model is f(x_{t}) = F*x_{t} + B*u_{t} + w
	 *   where F is the state transition model/matrix, B is the control input model,
	 *   and w is zero-mean, Gaussian white noise with covariance Q.
	 *   Q is normally derived as G*w*G^T where w models uncertainty of some physical property,
	 *   such as velocity or acceleration, and G is derived from physics.
	 *   In the current implementation, the noise model for w is restricted to a diagonal.
	 *   TODO: allow for a G
	 */
	void predict(const Matrix& F, const Matrix& B, const Vector& u,
			const SharedDiagonal& model) {
		// We will create a small factor graph f1-(x0)-f2-(x1)
		// where factor f1 is just the prior from time t0, P(x0)
		// and   factor f2 is from the motion model
		GaussianFactorGraph factorGraph;

		// push back f1
		factorGraph.push_back(density_->toFactor());

		// The factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
		factorGraph.add(0, -F, 1, I_, B*u, model);

		// Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		solve(factorGraph);
	}

	/**
	 * Update Kalman filter with a measurement
	 * For the Kalman Filter, the measurement function, h(x_{t}) = z_{t}
	 * will be of the form h(x_{t}) = H*x_{t} + v
	 * where H is the observation model/matrix, and v is zero-mean,
	 * Gaussian white noise with covariance R.
	 * Currently, R is restricted to diagonal Gaussians (model parameter)
	 */
	void update(const Matrix& H, const Vector& z, const SharedDiagonal& model) {
		// We will create a small factor graph f1-(x0)-f2
		// where factor f1 is the predictive density
		// and   factor f2 is from the measurement model
		GaussianFactorGraph factorGraph;

		// push back f1
		factorGraph.push_back(density_->toFactor());

		// The factor related to the measurements would be defined as
		// f2 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
		//    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
		factorGraph.add(0, H, z, model);

		// Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		solve(factorGraph);
	}

};
// KalmanFilter

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

