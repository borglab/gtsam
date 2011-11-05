/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testPose2SLAM.cpp
 *  @author Frank Dellaert, Viorela Ila
 **/

// Magically casts strings like "x3" to a Symbol('x',3) key, see Key.h
#define GTSAM_MAGIC_KEY

#include <gtsam/slam/pose2SLAM.h>
#include <gtsam/nonlinear/NonlinearOptimizer-inl.h>
#include <gtsam/inference/FactorGraph-inl.h>
#include <gtsam/base/numericalDerivative.h>
using namespace gtsam;

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>
using namespace boost;
using namespace boost::assign;

#include <iostream>
using namespace std;

// common measurement covariance
static double sx=0.5, sy=0.5,st=0.1;
static noiseModel::Gaussian::shared_ptr covariance(
		noiseModel::Gaussian::Covariance(Matrix_(3, 3,
	sx*sx, 0.0, 0.0,
	0.0, sy*sy, 0.0,
	0.0, 0.0, st*st
	)));
//static noiseModel::Gaussian::shared_ptr I3(noiseModel::Unit::Create(3));

/* ************************************************************************* */
Vector f(const Pose2& pose1, const Pose2& pose2) {
	Pose2 z(2,2,M_PI_2);
	Pose2Factor constraint(1, 2, z, covariance);
	return constraint.evaluateError(pose1, pose2);
}

TEST( Pose2SLAM, constraint )
{
	// create a factor between unknown poses p1 and p2
	Pose2 pose1, pose2(2,2,M_PI_2);
	Pose2Factor constraint(1, 2, pose2, covariance);
	Matrix H1, H2;
	Vector actual = constraint.evaluateError(pose1, pose2, H1, H2);

	Matrix numericalH1 = numericalDerivative21(&f , pose1, pose2);
	EXPECT(assert_equal(numericalH1,H1,5e-3));

	Matrix numericalH2 = numericalDerivative22(&f , pose1, pose2);
	EXPECT(assert_equal(numericalH2,H2));
}

/* ************************************************************************* */
TEST( Pose2SLAM, constructor )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Pose2Graph graph;
	graph.addConstraint(1,2,measured, covariance);
	// get the size of the graph
	size_t actual = graph.size();
	// verify
	size_t expected = 1;
	CHECK(actual == expected);
}

/* ************************************************************************* */
TEST( Pose2SLAM, linearization )
{
	// create a factor between unknown poses p1 and p2
	Pose2 measured(2,2,M_PI_2);
	Pose2Factor constraint(1,2,measured, covariance);
	Pose2Graph graph;
	graph.addConstraint(1,2,measured, covariance);

	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4) looking at negative (ground truth is at 4.1,2)
	Pose2Values config;
	config.insert(1,p1);
	config.insert(2,p2);
	// Linearize
	Ordering ordering(*config.orderingArbitrary());
	boost::shared_ptr<FactorGraph<GaussianFactor> > lfg_linearized = graph.linearize(config, ordering);
	//lfg_linearized->print("lfg_actual");

	// the expected linear factor
	FactorGraph<GaussianFactor> lfg_expected;
	Matrix A1 = Matrix_(3,3,
	    0.0,-2.0, -4.2,
	    2.0, 0.0, -4.2,
	    0.0, 0.0,-10.0);

	Matrix A2 = Matrix_(3,3,
	    2.0, 0.0,  0.0,
	    0.0, 2.0,  0.0,
	    0.0, 0.0, 10.0);

	Vector b = Vector_(3,-0.1/sx,0.1/sy,0.0);
	SharedDiagonal probModel1 = noiseModel::Unit::Create(3);
	lfg_expected.push_back(JacobianFactor::shared_ptr(new JacobianFactor(ordering["x1"], A1, ordering["x2"], A2, b, probModel1)));

	CHECK(assert_equal(lfg_expected, *lfg_linearized));
}

/* ************************************************************************* */
TEST(Pose2Graph, optimize) {

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, Pose2(0,0,0));
  fg->addConstraint(0, 1, Pose2(1,2,M_PI_2), covariance);

  // Create initial config
  boost::shared_ptr<Pose2Values> initial(new Pose2Values());
  initial->insert(0, Pose2(0,0,0));
  initial->insert(1, Pose2(0,0,0));

  // Choose an ordering and optimize
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1";
  typedef NonlinearOptimizer<Pose2Graph, Pose2Values> Optimizer;

  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
  Optimizer optimizer0(fg, initial, ordering, params);
  Optimizer optimizer = optimizer0.levenbergMarquardt();

  // Check with expected config
  Pose2Values expected;
  expected.insert(0, Pose2(0,0,0));
  expected.insert(1, Pose2(1,2,M_PI_2));
  CHECK(assert_equal(expected, *optimizer.values()));
}

/* ************************************************************************* */
// test optimization with 3 poses
TEST(Pose2Graph, optimizeThreePoses) {

	// Create a hexagon of poses
	Pose2Values hexagon = pose2SLAM::circle(3,1.0);
  Pose2 p0 = hexagon[0], p1 = hexagon[1];

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, p0);
  Pose2 delta = p0.between(p1);
  fg->addConstraint(0, 1, delta, covariance);
  fg->addConstraint(1, 2, delta, covariance);
  fg->addConstraint(2, 0, delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Values> initial(new Pose2Values());
  initial->insert(0, p0);
  initial->insert(1, hexagon[1].expmap(Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(2, hexagon[2].expmap(Vector_(3, 0.1,-0.1, 0.1)));

  // Choose an ordering
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1","x2";

  // optimize
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
  pose2SLAM::Optimizer optimizer0(fg, initial, ordering, params);
  pose2SLAM::Optimizer optimizer = optimizer0.levenbergMarquardt();

  Pose2Values actual = *optimizer.values();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));
}

/* ************************************************************************* */
// test optimization with 6 poses arranged in a hexagon and a loop closure
TEST_UNSAFE(Pose2Graph, optimizeCircle) {

	// Create a hexagon of poses
	Pose2Values hexagon = pose2SLAM::circle(6,1.0);
  Pose2 p0 = hexagon[0], p1 = hexagon[1];

	// create a Pose graph with one equality constraint and one measurement
  shared_ptr<Pose2Graph> fg(new Pose2Graph);
  fg->addHardConstraint(0, p0);
  Pose2 delta = p0.between(p1);
  fg->addConstraint(0, 1, delta, covariance);
  fg->addConstraint(1,2, delta, covariance);
  fg->addConstraint(2,3, delta, covariance);
  fg->addConstraint(3,4, delta, covariance);
  fg->addConstraint(4,5, delta, covariance);
  fg->addConstraint(5, 0, delta, covariance);

  // Create initial config
  boost::shared_ptr<Pose2Values> initial(new Pose2Values());
  initial->insert(0, p0);
  initial->insert(1, hexagon[1].expmap(Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(2, hexagon[2].expmap(Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert(3, hexagon[3].expmap(Vector_(3,-0.1, 0.1,-0.1)));
  initial->insert(4, hexagon[4].expmap(Vector_(3, 0.1,-0.1, 0.1)));
  initial->insert(5, hexagon[5].expmap(Vector_(3,-0.1, 0.1,-0.1)));

  // Choose an ordering
  shared_ptr<Ordering> ordering(new Ordering);
  *ordering += "x0","x1","x2","x3","x4","x5";

  // optimize
  NonlinearOptimizationParameters::sharedThis params = NonlinearOptimizationParameters::newDrecreaseThresholds(1e-15, 1e-15);
  pose2SLAM::Optimizer optimizer0(fg, initial, ordering, params);
  pose2SLAM::Optimizer optimizer = optimizer0.levenbergMarquardt();

  Pose2Values actual = *optimizer.values();

  // Check with ground truth
  CHECK(assert_equal(hexagon, actual));

  // Check loop closure
  CHECK(assert_equal(delta,actual[5].between(actual[0])));

//  Pose2SLAMOptimizer myOptimizer("3");

//  Matrix A1 = myOptimizer.a1();
//  LONGS_EQUAL(3,  A1.rows());
//  LONGS_EQUAL(17, A1.cols()); // 7 + 7 + 3
//
//  Matrix A2 = myOptimizer.a2();
//  LONGS_EQUAL(3, A1.rows());
//  LONGS_EQUAL(7, A2.cols()); // 7
//
//  Vector b1 = myOptimizer.b1();
//  LONGS_EQUAL(9, b1.size()); // 3 + 3 + 3
//
//  Vector b2 = myOptimizer.b2();
//  LONGS_EQUAL(3, b2.size()); // 3
//
//  // Here, call matlab to
//  // A=[A1;A2], b=[b1;b2]
//  // R=qr(A1)
//  // call pcg on A,b, with preconditioner R -> get x
//
//  Vector x = myOptimizer.optimize();
//  LONGS_EQUAL(9, x.size()); // 3 + 3 + 3
//
//  myOptimizer.update(x);
//
//  Pose2Values expected;
//  expected.insert(0, Pose2(0.,0.,0.));
//  expected.insert(1, Pose2(1.,0.,0.));
//  expected.insert(2, Pose2(2.,0.,0.));
//
//  // Check with ground truth
//  CHECK(assert_equal(expected, *myOptimizer.theta()));
}

/* ************************************************************************* */
TEST(Pose2Graph, optimize2) {
//  Pose2SLAMOptimizer myOptimizer("100");
//  Matrix A1 = myOptimizer.a1();
//  Matrix A2 = myOptimizer.a2();
//  cout << "A1: " << A1.rows() << " " << A1.cols() << endl;
//  cout << "A2: " << A2.rows() << " " << A2.cols() << endl;
//
//  //cout << "error: " << myOptimizer.error() << endl;
//  for(int i = 0; i<10; i++) {
//  	myOptimizer.linearize();
//  	Vector x = myOptimizer.optimize();
//  	myOptimizer.update(x);
//  }
//  //cout << "error: " << myOptimizer.error() << endl;
//  CHECK(myOptimizer.error() < 1.);
}

///* ************************************************************************* */
// SL-NEEDED? TEST(Pose2Graph, findMinimumSpanningTree) {
//	Pose2Graph G, T, C;
//	G.addConstraint(1, 2, Pose2(0.,0.,0.), I3);
//	G.addConstraint(1, 3, Pose2(0.,0.,0.), I3);
//	G.addConstraint(2, 3, Pose2(0.,0.,0.), I3);
//
//	PredecessorMap<pose2SLAM::Key> tree =
//			G.findMinimumSpanningTree<pose2SLAM::Key, Pose2Factor>();
//	CHECK(tree[1] == 1);
//	CHECK(tree[2] == 1);
//	CHECK(tree[3] == 1);
//}
//
///* ************************************************************************* */
// SL-NEEDED? TEST(Pose2Graph, split) {
//	Pose2Graph G, T, C;
//	G.addConstraint(1, 2, Pose2(0.,0.,0.), I3);
//	G.addConstraint(1, 3, Pose2(0.,0.,0.), I3);
//	G.addConstraint(2, 3, Pose2(0.,0.,0.), I3);
//
//	PredecessorMap<pose2SLAM::Key> tree;
//	tree.insert(1,2);
//	tree.insert(2,2);
//	tree.insert(3,2);
//
//	G.split<pose2SLAM::Key, Pose2Factor>(tree, T, C);
//	LONGS_EQUAL(2, T.size());
//	LONGS_EQUAL(1, C.size());
//}

using namespace pose2SLAM;

/* ************************************************************************* */
TEST( Pose2Values, pose2Circle )
{
	// expected is 4 poses tangent to circle with radius 1m
	Pose2Values expected;
	expected.insert(0, Pose2( 1,  0,   M_PI_2));
	expected.insert(1, Pose2( 0,  1, - M_PI  ));
	expected.insert(2, Pose2(-1,  0, - M_PI_2));
	expected.insert(3, Pose2( 0, -1,   0     ));

	Pose2Values actual = pose2SLAM::circle(4,1.0);
	CHECK(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( Pose2Values, expmap )
{
	// expected is circle shifted to right
	Pose2Values expected;
	expected.insert(0, Pose2( 1.1,  0,   M_PI_2));
	expected.insert(1, Pose2( 0.1,  1, - M_PI  ));
	expected.insert(2, Pose2(-0.9,  0, - M_PI_2));
	expected.insert(3, Pose2( 0.1, -1,   0     ));

	// Note expmap coordinates are in local coordinates, so shifting to right requires thought !!!
  Pose2Values circle(pose2SLAM::circle(4,1.0));
  Ordering ordering(*circle.orderingArbitrary());
	VectorValues delta(circle.dims(ordering));
	delta[ordering[Key(0)]] = Vector_(3, 0.0,-0.1,0.0);
	delta[ordering[Key(1)]] = Vector_(3, -0.1,0.0,0.0);
	delta[ordering[Key(2)]] = Vector_(3, 0.0,0.1,0.0);
	delta[ordering[Key(3)]] = Vector_(3, 0.1,0.0,0.0);
	Pose2Values actual = circle.expmap(delta, ordering);
	CHECK(assert_equal(expected,actual));
}

// Common measurement covariance
static SharedNoiseModel sigmas = sharedSigmas(Vector_(3,sx,sy,st));

/* ************************************************************************* */
// Very simple test establishing Ax-b \approx z-h(x)
TEST( Pose2Prior, error )
{
	// Choose a linearization point
	Pose2 p1(1, 0, 0); // robot at (1,0)
	Pose2Values x0;
	x0.insert(1, p1);

	// Create factor
	Pose2Prior factor(1, p1, sigmas);

	// Actual linearization
	Ordering ordering(*x0.orderingArbitrary());
	boost::shared_ptr<JacobianFactor> linear =
	    boost::dynamic_pointer_cast<JacobianFactor>(factor.linearize(x0, ordering));

	// Check error at x0, i.e. delta = zero !
	VectorValues delta(VectorValues::Zero(x0.dims(ordering)));
	delta[ordering["x1"]] = zero(3);
	Vector error_at_zero = Vector_(3,0.0,0.0,0.0);
	CHECK(assert_equal(error_at_zero,factor.whitenedError(x0)));
	CHECK(assert_equal(-error_at_zero,linear->error_vector(delta)));

	// Check error after increasing p2
	VectorValues addition(VectorValues::Zero(x0.dims(ordering)));
	addition[ordering["x1"]] = Vector_(3, 0.1, 0.0, 0.0);
	VectorValues plus = delta + addition;
	Pose2Values x1 = x0.expmap(plus, ordering);
	Vector error_at_plus = Vector_(3,0.1/sx,0.0,0.0); // h(x)-z = 0.1 !
	CHECK(assert_equal(error_at_plus,factor.whitenedError(x1)));
	CHECK(assert_equal(error_at_plus,linear->error_vector(plus)));
}

/* ************************************************************************* */
// common Pose2Prior for tests below
static gtsam::Pose2 priorVal(2,2,M_PI_2);
static Pose2Prior priorFactor(1,priorVal, sigmas);

/* ************************************************************************* */
// The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
// Hence i.e., b = approximates z-h(x0) = error_vector(x0)
LieVector hprior(const Pose2& p1) {
	return LieVector(sigmas->whiten(priorFactor.evaluateError(p1)));
}

/* ************************************************************************* */
TEST( Pose2Prior, linearize )
{
	// Choose a linearization point at ground truth
	Pose2Values x0;
	x0.insert(1,priorVal);

	// Actual linearization
	Ordering ordering(*x0.orderingArbitrary());
	boost::shared_ptr<JacobianFactor> actual =
	    boost::dynamic_pointer_cast<JacobianFactor>(priorFactor.linearize(x0, ordering));

	// Test with numerical derivative
	Matrix numericalH = numericalDerivative11(hprior, priorVal);
	CHECK(assert_equal(numericalH,actual->getA(actual->find(ordering["x1"]))));
}

/* ************************************************************************* */
// Very simple test establishing Ax-b \approx z-h(x)
TEST( Pose2Factor, error )
{
	// Choose a linearization point
	Pose2 p1; // robot at origin
	Pose2 p2(1, 0, 0); // robot at (1,0)
	Pose2Values x0;
	x0.insert(1, p1);
	x0.insert(2, p2);

	// Create factor
	Pose2 z = p1.between(p2);
	Pose2Factor factor(1, 2, z, covariance);

	// Actual linearization
	Ordering ordering(*x0.orderingArbitrary());
	boost::shared_ptr<JacobianFactor> linear =
	    boost::dynamic_pointer_cast<JacobianFactor>(factor.linearize(x0, ordering));

	// Check error at x0, i.e. delta = zero !
	VectorValues delta(x0.dims(ordering));
	delta[ordering["x1"]] = zero(3);
	delta[ordering["x2"]] = zero(3);
	Vector error_at_zero = Vector_(3,0.0,0.0,0.0);
	CHECK(assert_equal(error_at_zero,factor.unwhitenedError(x0)));
	CHECK(assert_equal(-error_at_zero, linear->error_vector(delta)));

	// Check error after increasing p2
	VectorValues plus = delta;
	plus[ordering["x2"]] = Vector_(3, 0.1, 0.0, 0.0);
	Pose2Values x1 = x0.expmap(plus, ordering);
	Vector error_at_plus = Vector_(3,0.1/sx,0.0,0.0); // h(x)-z = 0.1 !
	CHECK(assert_equal(error_at_plus,factor.whitenedError(x1)));
	CHECK(assert_equal(error_at_plus,linear->error_vector(plus)));
}

/* ************************************************************************* */
// common Pose2Factor for tests below
static Pose2 measured(2,2,M_PI_2);
static Pose2Factor factor(1,2,measured, covariance);

/* ************************************************************************* */
TEST( Pose2Factor, rhs )
{
	// Choose a linearization point
	Pose2 p1(1.1,2,M_PI_2); // robot at (1.1,2) looking towards y (ground truth is at 1,2, see testPose2)
	Pose2 p2(-1,4.1,M_PI);  // robot at (-1,4.1) looking at negative (ground truth is at -1,4)
	Pose2Values x0;
	x0.insert(1,p1);
	x0.insert(2,p2);

	// Actual linearization
	Ordering ordering(*x0.orderingArbitrary());
	boost::shared_ptr<JacobianFactor> linear =
	    boost::dynamic_pointer_cast<JacobianFactor>(factor.linearize(x0, ordering));

	// Check RHS
	Pose2 hx0 = p1.between(p2);
	CHECK(assert_equal(Pose2(2.1, 2.1, M_PI_2),hx0));
	Vector expected_b = Vector_(3, -0.1/sx, 0.1/sy, 0.0);
	CHECK(assert_equal(expected_b,-factor.whitenedError(x0)));
	CHECK(assert_equal(expected_b,linear->getb()));
}

/* ************************************************************************* */
// The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
// Hence i.e., b = approximates z-h(x0) = error_vector(x0)
LieVector h(const Pose2& p1,const Pose2& p2) {
	return LieVector(covariance->whiten(factor.evaluateError(p1,p2)));
}

/* ************************************************************************* */
TEST( Pose2Factor, linearize )
{
	// Choose a linearization point at ground truth
	Pose2 p1(1,2,M_PI_2);
	Pose2 p2(-1,4,M_PI);
	Pose2Values x0;
	x0.insert(1,p1);
	x0.insert(2,p2);

	// expected linearization
	Matrix expectedH1 = covariance->Whiten(Matrix_(3,3,
	    0.0,-1.0,-2.0,
	    1.0, 0.0,-2.0,
	    0.0, 0.0,-1.0
	));
	Matrix expectedH2 = covariance->Whiten(Matrix_(3,3,
	    1.0, 0.0, 0.0,
	    0.0, 1.0, 0.0,
	    0.0, 0.0, 1.0
	));
	Vector expected_b = Vector_(3, 0.0, 0.0, 0.0);

	// expected linear factor
	Ordering ordering(*x0.orderingArbitrary());
	SharedDiagonal probModel1 = noiseModel::Unit::Create(3);
	JacobianFactor expected(ordering["x1"], expectedH1, ordering["x2"], expectedH2, expected_b, probModel1);

	// Actual linearization
	boost::shared_ptr<JacobianFactor> actual =
	    boost::dynamic_pointer_cast<JacobianFactor>(factor.linearize(x0, ordering));
	CHECK(assert_equal(expected,*actual));

	// Numerical do not work out because BetweenFactor is approximate ?
	Matrix numericalH1 = numericalDerivative21(h, p1, p2);
	CHECK(assert_equal(expectedH1,numericalH1));
	Matrix numericalH2 = numericalDerivative22(h, p1, p2);
	CHECK(assert_equal(expectedH2,numericalH2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
