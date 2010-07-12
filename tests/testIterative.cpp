/**
 *  @file   testIterative.cpp
 *  @brief  Unit tests for iterative methods
 *  @author Frank Dellaert
 **/

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 3
#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "VectorConfig.h"
#include "smallExample.h"
#include "pose2SLAM.h"
#include "SubgraphPreconditioner.h"
#include "FactorGraph-inl.h"
#include "NonlinearFactorGraph-inl.h"
#include "iterative-inl.h"

using namespace std;
using namespace gtsam;
using namespace example;

static bool verbose = false;

/* ************************************************************************* */
TEST( Iterative, steepestDescent )
{
	// Expected solution
	Ordering ord;
	ord += "l1", "x1", "x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
	VectorConfig expected = fg.optimize(ord); // destructive

	// Do gradient descent
	GaussianFactorGraph fg2 = createGaussianFactorGraph();
	VectorConfig zero = createZeroDelta();
	VectorConfig actual = steepestDescent(fg2, zero, verbose);
	CHECK(assert_equal(expected,actual,1e-2));
}

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent )
{
	// Expected solution
	Ordering ord;
	ord += "l1", "x1", "x2";
	GaussianFactorGraph fg = createGaussianFactorGraph();
	VectorConfig expected = fg.optimize(ord); // destructive

	// create graph and get matrices
	GaussianFactorGraph fg2 = createGaussianFactorGraph();
	Matrix A;
	Vector b;
	Vector x0 = gtsam::zero(6);
	boost::tie(A, b) = fg2.matrix(ord);
	Vector expectedX = Vector_(6, -0.1, 0.1, -0.1, -0.1, 0.1, -0.2);

	// Do conjugate gradient descent, System version
	System Ab(A, b);
	Vector actualX = conjugateGradientDescent(Ab, x0, verbose);
	CHECK(assert_equal(expectedX,actualX,1e-9));

	// Do conjugate gradient descent, Matrix version
	Vector actualX2 = conjugateGradientDescent(A, b, x0, verbose);
	CHECK(assert_equal(expectedX,actualX2,1e-9));

	// Do conjugate gradient descent on factor graph
	VectorConfig zero = createZeroDelta();
	VectorConfig actual = conjugateGradientDescent(fg2, zero, verbose);
	CHECK(assert_equal(expected,actual,1e-2));

	// Test method
	VectorConfig actual2 = fg2.conjugateGradientDescent(zero, verbose);
	CHECK(assert_equal(expected,actual2,1e-2));
}

/* ************************************************************************* */
/*TEST( Iterative, conjugateGradientDescent_hard_constraint )
{
	typedef Pose2Config::Key Key;

	Pose2Config config;
	config.insert(1, Pose2(0.,0.,0.));
	config.insert(2, Pose2(1.5,0.,0.));

	Pose2Graph graph;
	Matrix cov = eye(3);
	graph.push_back(Pose2Graph::sharedFactor(new Pose2Factor(Key(1), Key(2), Pose2(1.,0.,0.), cov)));
	graph.addHardConstraint(1, config[1]);

	VectorConfig zeros;
	zeros.insert("x1",zero(3));
	zeros.insert("x2",zero(3));

	GaussianFactorGraph fg = graph.linearize(config);
	VectorConfig actual = conjugateGradientDescent(fg, zeros, true, 1e-3, 1e-5, 10);

	VectorConfig expected;
	expected.insert("x1", zero(3));
	expected.insert("x2", Vector_(-0.5,0.,0.));
	CHECK(assert_equal(expected, actual));
}*/

/* ************************************************************************* */
TEST( Iterative, conjugateGradientDescent_soft_constraint )
{
	Pose2Config config;
	config.insert(1, Pose2(0.,0.,0.));
	config.insert(2, Pose2(1.5,0.,0.));

	Pose2Graph graph;
	graph.addPrior(1, Pose2(0.,0.,0.), Isotropic::Sigma(3, 1e-10));
	graph.addConstraint(1,2, Pose2(1.,0.,0.), Isotropic::Sigma(3, 1));

	VectorConfig zeros;
	zeros.insert("x1",zero(3));
	zeros.insert("x2",zero(3));

	boost::shared_ptr<GaussianFactorGraph> fg = graph.linearize(config);
	VectorConfig actual = conjugateGradientDescent(*fg, zeros, verbose, 1e-3, 1e-5, 100);

	VectorConfig expected;
	expected.insert("x1", zero(3));
	expected.insert("x2", Vector_(3,-0.5,0.,0.));
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( Iterative, subgraphPCG )
{
	typedef Pose2Config::Key Key;

	Pose2Config theta_bar;
	theta_bar.insert(1, Pose2(0.,0.,0.));
	theta_bar.insert(2, Pose2(1.5,0.,0.));

	Pose2Graph graph;
	graph.addPrior(1, Pose2(0.,0.,0.), Isotropic::Sigma(3, 1e-10));
	graph.addConstraint(1,2, Pose2(1.,0.,0.), Isotropic::Sigma(3, 1));

	// generate spanning tree and create ordering
	PredecessorMap<Key> tree = graph.findMinimumSpanningTree<Key, Pose2Factor>();
	list<Key> keys = predecessorMap2Keys(tree);
	list<Symbol> symbols;
	symbols.resize(keys.size());
	std::transform(keys.begin(), keys.end(), symbols.begin(), key2symbol<Key>);
	Ordering ordering(symbols);

	Key root = keys.back();
	Pose2Graph T, C;
	graph.split<Key, Pose2Factor>(tree, T, C);

	// build the subgraph PCG system
	boost::shared_ptr<GaussianFactorGraph> Ab1_ = T.linearize(theta_bar);
	SubgraphPreconditioner::sharedFG Ab1 = T.linearize(theta_bar);
	SubgraphPreconditioner::sharedFG Ab2 = C.linearize(theta_bar);
	SubgraphPreconditioner::sharedBayesNet Rc1 = Ab1_->eliminate_(ordering);
	SubgraphPreconditioner::sharedConfig xbar = optimize_(*Rc1);
	SubgraphPreconditioner system(Ab1, Ab2, Rc1, xbar);

	VectorConfig zeros = VectorConfig::zero(*xbar);

	// Solve the subgraph PCG
	VectorConfig ybar = conjugateGradients<SubgraphPreconditioner, VectorConfig,
			Errors> (system, zeros, verbose, 1e-5, 1e-5, 100);
	VectorConfig actual = system.x(ybar);

	VectorConfig expected;
	expected.insert("x1", zero(3));
	expected.insert("x2", Vector_(3, -0.5, 0., 0.));
	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
