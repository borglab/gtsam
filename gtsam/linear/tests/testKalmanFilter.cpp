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
 *  @Author: Frank Dellaert
 *  @Author: Stephen Williams
 */

#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/HessianFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

class KalmanFilter {
private:

	size_t n_; /** dimensionality of state */

	/**
	 * The Kalman filter posterior density is a Gaussian Conditional with no parents
	 */
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
		density_ = bayesNet->back();
	}

public:

	/**
	 * Constructor from prior density at time k=0
	 * In Kalman Filter notation, these are is x_{0|0} and P_{0|0}
	 * @param x estimate at time 0
	 * @param P covariance at time 0
	 *
	 */
	KalmanFilter(const Vector& x, const Matrix& P) :
			n_(x.size()) {

		// Create a Hessian Factor from (x,P)
		HessianFactor::shared_ptr factor(new HessianFactor(0, x, P));

#ifdef LOWLEVEL
		// Eliminate it directly using LDL
		size_t nrFrontals = 1;
		Eigen::LDLT<Matrix>::TranspositionType pi = factor->partialLDL(nrFrontals);
		vector<Index> keys;
		keys.push_back(0);
		density_ = factor->splitEliminatedFactor(nrFrontals, keys, pi);
#else
		// Create a factor graph f(x), eliminate it into P(x)
		GaussianFactorGraph factorGraph;
		factorGraph.push_back(factor);
		solve(factorGraph);
#endif
	}

	/**
	 * Return mean of posterior P(x|Z) at given all measurements Z
	 */
	Vector mean() const {
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
	 *   Note, in some models, Q is actually derived as G*w*G^T where w models uncertainty of some
	 *   physical property, such as velocity or acceleration, and G is derived from physics.
	 */
	void predict(const Matrix& F, const Matrix& B, const Vector& u,
			const Matrix& Q) {
	  // We will create a small factor graph f1-(x0)-f2-(x1)
	  // where factor f1 is just the prior from time t0, P(x0)
	  // and   factor f2 is from the motion model
		GaussianFactorGraph factorGraph;

		// push back f1
		factorGraph.push_back(density_->toFactor());

		// The factor related to the motion model is defined as
		// f2(x_{t},x_{t+1}) = (F*x_{t} - x_{t+1}) * Q^-1 * (F*x_{t} - x_{t+1})^T


	  // Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
		solve(factorGraph);
	}
};
// KalmanFilter

/* ************************************************************************* */
TEST(GaussianConditional, constructor) {

	// [code below basically does SRIF with LDL]

	// Ground truth example
	// Start at origin, move to the right (x-axis): 0,0  0,1  0,2
	// Motion model is just moving to the right (x'-x)^2
	// Measurements are GPS like, (x-z)^2, where z is a 2D measurement
	// i.e., we should get 0,0  0,1  0,2 if there is no noise

  // Initialize state x0 (2D point) at origin
  Vector x00 = Vector_(2,0.0,0.0);
  Matrix P00 = 0.1*eye(2,2);

  // Initialize a Kalman filter
  KalmanFilter kalmanFilter(x00,P00);
	EXPECT(assert_equal(x00, kalmanFilter.mean()));
	EXPECT(assert_equal(P00, kalmanFilter.covariance()));

  // Now predict the state at t=1, i.e. P(x_1)
	// For the purposes of this example, let us assume we are using a constant-position model and
	// the controls are driving the point to the right at 1 m/s.
	// Then, F = [1 0 ; 0 1], B = [1 0 ; 0 1] and u = [1 ; 0].
	// Let us also assume that the process noise Q = [0.1 0 ; 0 0.1];
	Matrix F = eye(2,2);
	Matrix B = eye(2,2);
	Vector u = Vector_(2,1.0,0.0);
	Matrix Q = 0.2*eye(2,2);
	kalmanFilter.predict(F, B, u, Q);

  Vector x10 = Vector_(2,0.0,0.0);
  Matrix P10 = 0.1*eye(2,2);
	EXPECT(assert_equal(x10, kalmanFilter.mean()));
	EXPECT(assert_equal(P10, kalmanFilter.covariance()));
/*
  //
  // Because of the way GTSAM works internally, we have used nonlinear class even though this example
  // system is linear. We first convert the nonlinear factor graph into a linear one, using the specified
  // ordering. Linear factors are simply numbered, and are not accessible via named key like the nonlinear
  // variables. Also, the nonlinear factors are linearized around an initial estimate. For a true linear
  // system, the initial estimate is not important.

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianSequentialSolver solver0(*linearFactorGraph);
  GaussianBayesNet::shared_ptr linearBayesNet = solver0.eliminate();

  // Extract the current estimate of x1,P1 from the Bayes Network
  VectorValues result = optimize(*linearBayesNet);
  Vector x1_predict = linearizationPoints[x1].expmap(result[ordering->at(x1)]);
  x1_predict.print("X1 Predict");

  // Update the new linearization point to the new estimate
  linearizationPoints.update(x1, x1_predict);



  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  // Some care must be done here, as the linearization point in future steps will be different
  // than what was used when the factor was created.
  // f = || F*dx1' - (F*x0 - x1) ||^2, originally linearized at x1 = x0
  // After this step, the factor needs to be linearized around x1 = x1_predict
  // This changes the factor to f = || F*dx1'' - b'' ||^2
  //                              = || F*(dx1' - (dx1' - dx1'')) - b'' ||^2
  //                              = || F*dx1' - F*(dx1' - dx1'') - b'' ||^2
  //                              = || F*dx1' - (b'' + F(dx1' - dx1'')) ||^2
  //                              -> b' = b'' + F(dx1' - dx1'')
  //                              -> b'' = b' - F(dx1' - dx1'')
  //                              = || F*dx1'' - (b'  - F(dx1' - dx1'')) ||^2
  //                              = || F*dx1'' - (b'  - F(x_predict - x_inital)) ||^2
  const GaussianConditional::shared_ptr& cg0 = linearBayesNet->back();
  assert(cg0->nrFrontals() == 1);
  assert(cg0->nrParents() == 0);
  linearFactorGraph->add(0, cg0->get_R(), cg0->get_d() - cg0->get_R()*result[ordering->at(x1)], noiseModel::Diagonal::Sigmas(cg0->get_sigmas(), true));

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->insert(x1, 0);

  // Now, a measurement, z1, has been received, and the Kalman Filter should be "Updated"/"Corrected"
  // This is equivalent to saying P(x1|z1) ~ P(z1|x1)*P(x1) ~ f3(x1)*f4(x1;z1)
  // where f3 is the prior from the previous step, and
  // where f4 is a measurement factor
  //
  // So, now we need to create the measurement factor, f4
  // For the Kalman Filter, this is the measurement function, h(x_{t}) = z_{t}
  // Assuming the system is linear, this will be of the form h(x_{t}) = H*x_{t} + v
  // where H is the observation model/matrix, and v is zero-mean, Gaussian white noise with covariance R
  //
  // For the purposes of this example, let us assume we have something like a GPS that returns
  // the current position of the robot. For this simple example, we can use a PriorFactor to model the
  // observation as it depends on only a single state variable, x1. To model real sensor observations
  // generally requires the creation of a new factor type. For example, factors for range sensors, bearing
  // sensors, and camera projections have already been added to GTSAM.
  //
  // In the case of factor graphs, the factor related to the measurements would be defined as
  // f4 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
  //    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
  // This can be modeled using the PriorFactor, where the mean is z_{t} and the covariance is R.
  Vector z1(1.0, 0.0);
  SharedDiagonal R1 = noiseModel::Diagonal::Sigmas(Vector_(2, 0.25, 0.25));
  PriorFactor<Values, Key> factor4(x1, z1, R1);
  // Linearize the factor and add it to the linear factor graph
  linearFactorGraph->push_back(factor4.linearize(linearizationPoints, *ordering));

  // We have now made the small factor graph f3-(x1)-f4
  // where factor f3 is the prior from previous time ( P(x1) )
  // and   factor f4 is from the measurement, z1 ( P(x1|z1) )
  // Eliminate this in order x1, to get Bayes net P(x1)
  // As this is a filter, all we need is the posterior P(x1), so we just keep the root of the Bayes net
  // We solve as before...

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianSequentialSolver solver1(*linearFactorGraph);
  linearBayesNet = solver1.eliminate();

  // Extract the current estimate of x1 from the Bayes Network
  result = optimize(*linearBayesNet);
  Vector x1_update = linearizationPoints[x1].expmap(result[ordering->at(x1)]);
  x1_update.print("X1 Update");

  // Update the linearization point to the new estimate
  linearizationPoints.update(x1, x1_update);






  // Wash, rinse, repeat for another time step
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  // The linearization point of this prior must be moved to the new estimate of x, and the key/index needs to be reset to 0,
  // the first key in the next iteration
  const GaussianConditional::shared_ptr& cg1 = linearBayesNet->back();
  assert(cg1->nrFrontals() == 1);
  assert(cg1->nrParents() == 0);
  JacobianFactor tmpPrior1 = JacobianFactor(*cg1);
  linearFactorGraph->add(0, tmpPrior1.getA(tmpPrior1.begin()), tmpPrior1.getb() - tmpPrior1.getA(tmpPrior1.begin()) * result[ordering->at(x1)], tmpPrior1.get_model());

  // Create a key for the new state
  Key x2(2);

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->insert(x1, 0);
  ordering->insert(x2, 1);

  // Create a nonlinear factor describing the motion model
  difference = Vector(1,0);
  Q = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1));
  BetweenFactor<Values, Key> factor6(x1, x2, difference, Q);

  // Linearize the factor and add it to the linear factor graph
  linearizationPoints.insert(x2, x1_update);
  linearFactorGraph->push_back(factor6.linearize(linearizationPoints, *ordering));

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x1,x2) = P(x1|x2)*P(x2) )
  GaussianSequentialSolver solver2(*linearFactorGraph);
  linearBayesNet = solver2.eliminate();

  // Extract the current estimate of x2 from the Bayes Network
  result = optimize(*linearBayesNet);
  Vector x2_predict = linearizationPoints[x2].expmap(result[ordering->at(x2)]);
  x2_predict.print("X2 Predict");

  // Update the linearization point to the new estimate
  linearizationPoints.update(x2, x2_predict);



  // Now add the next measurement
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  const GaussianConditional::shared_ptr& cg2 = linearBayesNet->back();
  assert(cg2->nrFrontals() == 1);
  assert(cg2->nrParents() == 0);
  JacobianFactor tmpPrior2 = JacobianFactor(*cg2);
  linearFactorGraph->add(0, tmpPrior2.getA(tmpPrior2.begin()), tmpPrior2.getb() - tmpPrior2.getA(tmpPrior2.begin()) * result[ordering->at(x2)], tmpPrior2.get_model());

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->insert(x2, 0);

  // And update using z2 ...
  Vector z2(2.0, 0.0);
  SharedDiagonal R2 = noiseModel::Diagonal::Sigmas(Vector_(2, 0.25, 0.25));
  PriorFactor<Values, Key> factor8(x2, z2, R2);

  // Linearize the factor and add it to the linear factor graph
  linearFactorGraph->push_back(factor8.linearize(linearizationPoints, *ordering));

  // We have now made the small factor graph f7-(x2)-f8
  // where factor f7 is the prior from previous time ( P(x2) )
  // and   factor f8 is from the measurement, z2 ( P(x2|z2) )
  // Eliminate this in order x2, to get Bayes net P(x2)
  // As this is a filter, all we need is the posterior P(x2), so we just keep the root of the Bayes net
  // We solve as before...

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianSequentialSolver solver3(*linearFactorGraph);
  linearBayesNet = solver3.eliminate();

  // Extract the current estimate of x2 from the Bayes Network
  result = optimize(*linearBayesNet);
  Vector x2_update = linearizationPoints[x2].expmap(result[ordering->at(x2)]);
  x2_update.print("X2 Update");

  // Update the linearization point to the new estimate
  linearizationPoints.update(x2, x2_update);






  // Wash, rinse, repeat for a third time step
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  const GaussianConditional::shared_ptr& cg3 = linearBayesNet->back();
  assert(cg3->nrFrontals() == 1);
  assert(cg3->nrParents() == 0);
  JacobianFactor tmpPrior3 = JacobianFactor(*cg3);
  linearFactorGraph->add(0, tmpPrior3.getA(tmpPrior3.begin()), tmpPrior3.getb() - tmpPrior3.getA(tmpPrior3.begin()) * result[ordering->at(x2)], tmpPrior3.get_model());

  // Create a key for the new state
  Key x3(3);

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->insert(x2, 0);
  ordering->insert(x3, 1);

  // Create a nonlinear factor describing the motion model
  difference = Vector(1,0);
  Q = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.1));
  BetweenFactor<Values, Key> factor10(x2, x3, difference, Q);

  // Linearize the factor and add it to the linear factor graph
  linearizationPoints.insert(x3, x2_update);
  linearFactorGraph->push_back(factor10.linearize(linearizationPoints, *ordering));

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x1,x2) = P(x1|x2)*P(x2) )
  GaussianSequentialSolver solver4(*linearFactorGraph);
  linearBayesNet = solver4.eliminate();

  // Extract the current estimate of x3 from the Bayes Network
  result = optimize(*linearBayesNet);
  Vector x3_predict = linearizationPoints[x3].expmap(result[ordering->at(x3)]);
  x3_predict.print("X3 Predict");

  // Update the linearization point to the new estimate
  linearizationPoints.update(x3, x3_predict);



  // Now add the next measurement
  // Create a new, empty graph and add the prior from the previous step
  linearFactorGraph = GaussianFactorGraph::shared_ptr(new GaussianFactorGraph);

  // Convert the root conditional, P(x1) in this case, into a Prior for the next step
  const GaussianConditional::shared_ptr& cg4 = linearBayesNet->back();
  assert(cg4->nrFrontals() == 1);
  assert(cg4->nrParents() == 0);
  JacobianFactor tmpPrior4 = JacobianFactor(*cg4);
  linearFactorGraph->add(0, tmpPrior4.getA(tmpPrior4.begin()), tmpPrior4.getb() - tmpPrior4.getA(tmpPrior4.begin()) * result[ordering->at(x3)], tmpPrior4.get_model());

  // Create the desired ordering
  ordering = Ordering::shared_ptr(new Ordering);
  ordering->insert(x3, 0);

  // And update using z3 ...
  Vector z3(3.0, 0.0);
  SharedDiagonal R3 = noiseModel::Diagonal::Sigmas(Vector_(2, 0.25, 0.25));
  PriorFactor<Values, Key> factor12(x3, z3, R3);

  // Linearize the factor and add it to the linear factor graph
  linearFactorGraph->push_back(factor12.linearize(linearizationPoints, *ordering));

  // We have now made the small factor graph f11-(x3)-f12
  // where factor f11 is the prior from previous time ( P(x3) )
  // and   factor f12 is from the measurement, z3 ( P(x3|z3) )
  // Eliminate this in order x3, to get Bayes net P(x3)
  // As this is a filter, all we need is the posterior P(x3), so we just keep the root of the Bayes net
  // We solve as before...

  // Solve the linear factor graph, converting it into a linear Bayes Network ( P(x0,x1) = P(x0|x1)*P(x1) )
  GaussianSequentialSolver solver5(*linearFactorGraph);
  linearBayesNet = solver5.eliminate();

  // Extract the current estimate of x2 from the Bayes Network
  result = optimize(*linearBayesNet);
  Vector x3_update = linearizationPoints[x3].expmap(result[ordering->at(x3)]);
  x3_update.print("X3 Update");

  // Update the linearization point to the new estimate
  linearizationPoints.update(x3, x3_update);
  */
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

