/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testIterative.cpp
 *  @brief  Unit tests for iterative methods
 *  @author Frank Dellaert
 **/

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
//#include <gtsam/linear/VectorValues.h>
//#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/linear/iterative-inl.h>
//#include <gtsam/inference/FactorGraph-inl.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace example;
using symbol_shorthand::X; // to create pose keys
using symbol_shorthand::L; // to create landmark keys

static bool verbose = false;

/* ************************************************************************* */
TEST( Iterative, steepestDescent )
{
	// Create factor graph
	Ordering ord;
	ord += L(1), X(1), X(2);
	FactorGraph<JacobianFactor> fg = createGaussianFactorGraph(ord);

  // eliminate and solve
  VectorValues expected = *GaussianSequentialSolver(fg).optimize();

	// Do gradient descent
	VectorValues zero = VectorValues::Zero(expected); // TODO, how do we do this normally?
  ConjugateGradientParameters parameters;
//  parameters.verbosity_ = ConjugateGradientParameters::COMPLEXITY;
	VectorValues actual = steepestDescent(fg, zero, parameters);
	CHECK(assert_equal(expected,actual,1e-2));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent )
{
//	// Expected solution
//	Ordering ord;
//	ord += L(1), X(1), X(2);
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//	VectorValues expected = fg.optimize(ord); // destructive
//
//	// create graph and get matrices
//	GaussianFactorGraph fg2 = createGaussianFactorGraph();
//	Matrix A;
//	Vector b;
//	Vector x0 = gtsam::zero(6);
//	boost::tie(A, b) = fg2.matrix(ord);
//	Vector expectedX = Vector_(6, -0.1, 0.1, -0.1, -0.1, 0.1, -0.2);
//
//	// Do conjugate gradient descent, System version
//	System Ab(A, b);
//	Vector actualX = conjugateGradientDescent(Ab, x0, verbose);
//	CHECK(assert_equal(expectedX,actualX,1e-9));
//
//	// Do conjugate gradient descent, Matrix version
//	Vector actualX2 = conjugateGradientDescent(A, b, x0, verbose);
//	CHECK(assert_equal(expectedX,actualX2,1e-9));
//
//	// Do conjugate gradient descent on factor graph
//	VectorValues zero = createZeroDelta();
//	VectorValues actual = conjugateGradientDescent(fg2, zero, verbose);
//	CHECK(assert_equal(expected,actual,1e-2));
//
//	// Test method
//	VectorValues actual2 = fg2.conjugateGradientDescent(zero, verbose);
//	CHECK(assert_equal(expected,actual2,1e-2));
}

/* ************************************************************************* */
/*TEST( Iterative, conjugateGradientDescent_hard_constraint )
{
	typedef Pose2Values::Key Key;

	Pose2Values config;
	config.insert(1, Pose2(0.,0.,0.));
	config.insert(2, Pose2(1.5,0.,0.));

	Pose2Graph graph;
	Matrix cov = eye(3);
	graph.push_back(Pose2Graph::sharedFactor(new Pose2Factor(Key(1), Key(2), Pose2(1.,0.,0.), cov)));
	graph.addHardConstraint(1, config[1]);

	VectorValues zeros;
	zeros.insert(X(1),zero(3));
	zeros.insert(X(2),zero(3));

	GaussianFactorGraph fg = graph.linearize(config);
	VectorValues actual = conjugateGradientDescent(fg, zeros, true, 1e-3, 1e-5, 10);

	VectorValues expected;
	expected.insert(X(1), zero(3));
	expected.insert(X(2), Vector_(-0.5,0.,0.));
	CHECK(assert_equal(expected, actual));
}*/

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent_soft_constraint )
{
//	Pose2Values config;
//	config.insert(1, Pose2(0.,0.,0.));
//	config.insert(2, Pose2(1.5,0.,0.));
//
//	Pose2Graph graph;
//	graph.addPrior(1, Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3, 1e-10));
//	graph.addConstraint(1,2, Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1));
//
//	VectorValues zeros;
//	zeros.insert(X(1),zero(3));
//	zeros.insert(X(2),zero(3));
//
//	boost::shared_ptr<GaussianFactorGraph> fg = graph.linearize(config);
//	VectorValues actual = conjugateGradientDescent(*fg, zeros, verbose, 1e-3, 1e-5, 100);
//
//	VectorValues expected;
//	expected.insert(X(1), zero(3));
//	expected.insert(X(2), Vector_(3,-0.5,0.,0.));
//	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Iterative, subgraphPCG )
{
//	typedef Pose2Values::Key Key;
//
//	Pose2Values theta_bar;
//	theta_bar.insert(1, Pose2(0.,0.,0.));
//	theta_bar.insert(2, Pose2(1.5,0.,0.));
//
//	Pose2Graph graph;
//	graph.addPrior(1, Pose2(0.,0.,0.), noiseModel::Isotropic::Sigma(3, 1e-10));
//	graph.addConstraint(1,2, Pose2(1.,0.,0.), noiseModel::Isotropic::Sigma(3, 1));
//
//	// generate spanning tree and create ordering
//	PredecessorMap<Key> tree = graph.findMinimumSpanningTree<Key, Pose2Factor>();
//	list<Key> keys = predecessorMap2Keys(tree);
//	list<Symbol> symbols;
//	symbols.resize(keys.size());
//	std::transform(keys.begin(), keys.end(), symbols.begin(), key2symbol<Key>);
//	Ordering ordering(symbols);
//
//	Key root = keys.back();
//	Pose2Graph T, C;
//	graph.split<Key, Pose2Factor>(tree, T, C);
//
//	// build the subgraph PCG system
//	boost::shared_ptr<GaussianFactorGraph> Ab1_ = T.linearize(theta_bar);
//	SubgraphPreconditioner::sharedFG Ab1 = T.linearize(theta_bar);
//	SubgraphPreconditioner::sharedFG Ab2 = C.linearize(theta_bar);
//	SubgraphPreconditioner::sharedBayesNet Rc1 = Ab1_->eliminate_(ordering);
//	SubgraphPreconditioner::sharedValues xbar = optimize_(*Rc1);
//	SubgraphPreconditioner system(Ab1, Ab2, Rc1, xbar);
//
//	VectorValues zeros = VectorValues::zero(*xbar);
//
//	// Solve the subgraph PCG
//	VectorValues ybar = conjugateGradients<SubgraphPreconditioner, VectorValues,
//			Errors> (system, zeros, verbose, 1e-5, 1e-5, 100);
//	VectorValues actual = system.x(ybar);
//
//	VectorValues expected;
//	expected.insert(X(1), zero(3));
//	expected.insert(X(2), Vector_(3, -0.5, 0., 0.));
//	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
