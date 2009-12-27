/**
 * @file    testGaussianISAM.cpp
 * @brief   Unit tests for GaussianISAM
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include "Ordering.h"
#include "GaussianBayesNet.h"
#include "ISAM-inl.h"
#include "GaussianISAM.h"
#include "smallExample.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
// Some numbers that should be consistent among all smoother tests

double sigmax1 = 0.786153, sigmax2 = 0.687131, sigmax3 = 0.671512, sigmax4 =
		0.669534, sigmax5 = sigmax3, sigmax6 = sigmax2, sigmax7 = sigmax1;

/* ************************************************************************* */
TEST( ISAM, iSAM_smoother )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);

	// run iSAM for every factor
	GaussianISAM actual;
	BOOST_FOREACH(boost::shared_ptr<GaussianFactor> factor, smoother) {
		GaussianFactorGraph factorGraph;
		factorGraph.push_back(factor);
		actual.update(factorGraph);
	}

	// Create expected Bayes Tree by solving smoother with "natural" ordering
	Ordering ordering;
	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
	GaussianISAM expected(smoother.eliminate(ordering));

	// Check whether BayesTree is correct
	CHECK(assert_equal(expected, actual));

	// obtain solution
	VectorConfig e; // expected solution
	Vector v = Vector_(2, 0., 0.);
	for (int i=1; i<=7; i++)
		e.insert(symbol('x', i), v);
	VectorConfig optimized = optimize(actual); // actual solution
	CHECK(assert_equal(e, optimized));
}

/* ************************************************************************* */
TEST( ISAM, iSAM_smoother2 )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);

	// Create initial tree from first 4 timestamps in reverse order !
	Ordering ord; ord += "x4","x3","x2","x1";
	GaussianFactorGraph factors1;
	for (int i=0;i<7;i++) factors1.push_back(smoother[i]);
	GaussianISAM actual(factors1.eliminate(ord));

	// run iSAM with remaining factors
	GaussianFactorGraph factors2;
	for (int i=7;i<13;i++) factors2.push_back(smoother[i]);
	actual.update(factors2);

	// Create expected Bayes Tree by solving smoother with "natural" ordering
	Ordering ordering;
	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
	GaussianISAM expected(smoother.eliminate(ordering));

	CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
C1 x6 x7
C2   x5 : x6
C3     x4 : x5
C4       x3 : x4
C5         x2 : x3
C6           x1 : x2
/* ************************************************************************* */
TEST( BayesTree, linear_smoother_shortcuts )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	for (int t = 1; t <= 7; t++)
		ordering.push_back(symbol('x', t));

	// eliminate using the "natural" ordering
	GaussianBayesNet chordalBayesNet = smoother.eliminate(ordering);

	// Create the Bayes tree
	GaussianISAM bayesTree(chordalBayesNet);
	LONGS_EQUAL(6,bayesTree.size());

	// Check the conditional P(Root|Root)
	GaussianBayesNet empty;
	GaussianISAM::sharedClique R = bayesTree.root();
	GaussianBayesNet actual1 = R->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(empty,actual1,1e-4));

	// Check the conditional P(C2|Root)
	GaussianISAM::sharedClique C2 = bayesTree["x5"];
	GaussianBayesNet actual2 = C2->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(empty,actual2,1e-4));

	// Check the conditional P(C3|Root)
  Vector sigma3 = repeat(2, 0.61808);
  Matrix A56 = Matrix_(2,2,-0.382022,0.,0.,-0.382022);
	GaussianBayesNet expected3;
	push_front(expected3,"x5", zero(2), eye(2), "x6", A56, sigma3);
	GaussianISAM::sharedClique C3 = bayesTree["x4"];
	GaussianBayesNet actual3 = C3->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(expected3,actual3,1e-4));

	// Check the conditional P(C4|Root)
  Vector sigma4 = repeat(2, 0.661968);
  Matrix A46 = Matrix_(2,2,-0.146067,0.,0.,-0.146067);
  GaussianBayesNet expected4;
  push_front(expected4,"x4", zero(2), eye(2), "x6", A46, sigma4);
	GaussianISAM::sharedClique C4 = bayesTree["x3"];
	GaussianBayesNet actual4 = C4->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(expected4,actual4,1e-4));
}

/* ************************************************************************* *
 Bayes tree for smoother with "nested dissection" ordering:

	 Node[x1] P(x1 | x2)
	 Node[x3] P(x3 | x2 x4)
	 Node[x5] P(x5 | x4 x6)
	 Node[x7] P(x7 | x6)
	 Node[x2] P(x2 | x4)
	 Node[x6] P(x6 | x4)
	 Node[x4] P(x4)

 becomes

	 C1		 x5 x6 x4
	 C2		  x3 x2 : x4
	 C3		    x1 : x2
	 C4		  x7 : x6

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_marginals )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// eliminate using a "nested dissection" ordering
	GaussianBayesNet chordalBayesNet = smoother.eliminate(ordering);

  VectorConfig expectedSolution;
  BOOST_FOREACH(string key, ordering)
		expectedSolution.insert(key,zero(2));
  VectorConfig actualSolution = optimize(chordalBayesNet);
	CHECK(assert_equal(expectedSolution,actualSolution,1e-4));

	// Create the Bayes tree
	GaussianISAM bayesTree(chordalBayesNet);
	LONGS_EQUAL(4,bayesTree.size());

	// Check marginal on x1
	GaussianBayesNet expected1 = simpleGaussian("x1", zero(2), sigmax1);
	GaussianBayesNet actual1 = bayesTree.marginalBayesNet<GaussianFactor>("x1");
	CHECK(assert_equal(expected1,actual1,1e-4));

	// Check marginal on x2
  GaussianBayesNet expected2 = simpleGaussian("x2", zero(2), sigmax2);
	GaussianBayesNet actual2 = bayesTree.marginalBayesNet<GaussianFactor>("x2");
	CHECK(assert_equal(expected2,actual2,1e-4));

	// Check marginal on x3
  GaussianBayesNet expected3 = simpleGaussian("x3", zero(2), sigmax3);
	GaussianBayesNet actual3 = bayesTree.marginalBayesNet<GaussianFactor>("x3");
	CHECK(assert_equal(expected3,actual3,1e-4));

	// Check marginal on x4
  GaussianBayesNet expected4 = simpleGaussian("x4", zero(2), sigmax4);
	GaussianBayesNet actual4 = bayesTree.marginalBayesNet<GaussianFactor>("x4");
	CHECK(assert_equal(expected4,actual4,1e-4));

	// Check marginal on x7 (should be equal to x1)
  GaussianBayesNet expected7 = simpleGaussian("x7", zero(2), sigmax7);
	GaussianBayesNet actual7 = bayesTree.marginalBayesNet<GaussianFactor>("x7");
	CHECK(assert_equal(expected7,actual7,1e-4));
}

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_shortcuts )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// Create the Bayes tree
	GaussianBayesNet chordalBayesNet = smoother.eliminate(ordering);
	GaussianISAM bayesTree(chordalBayesNet);

	// Check the conditional P(Root|Root)
	GaussianBayesNet empty;
	GaussianISAM::sharedClique R = bayesTree.root();
	GaussianBayesNet actual1 = R->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(empty,actual1,1e-4));

	// Check the conditional P(C2|Root)
	GaussianISAM::sharedClique C2 = bayesTree["x3"];
	GaussianBayesNet actual2 = C2->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(empty,actual2,1e-4));

	// Check the conditional P(C3|Root), which should be equal to P(x2|x4)
	GaussianConditional::shared_ptr p_x2_x4 = chordalBayesNet["x2"];
	GaussianBayesNet expected3; expected3.push_back(p_x2_x4);
	GaussianISAM::sharedClique C3 = bayesTree["x1"];
	GaussianBayesNet actual3 = C3->shortcut<GaussianFactor>(R);
	CHECK(assert_equal(expected3,actual3,1e-4));
}

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_clique_marginals )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// Create the Bayes tree
	GaussianBayesNet chordalBayesNet = smoother.eliminate(ordering);
	GaussianISAM bayesTree(chordalBayesNet);

	// Check the clique marginal P(C3)
	GaussianBayesNet expected = simpleGaussian("x2",zero(2),sigmax2);
  Vector sigma = repeat(2, 0.707107);
  Matrix A12 = (-0.5)*eye(2);
  push_front(expected,"x1", zero(2), eye(2), "x2", A12, sigma);
	GaussianISAM::sharedClique R = bayesTree.root(), C3 = bayesTree["x1"];
	FactorGraph<GaussianFactor> marginal = C3->marginal<GaussianFactor>(R);
	GaussianBayesNet actual = eliminate<GaussianFactor,GaussianConditional>(marginal,C3->keys());
	CHECK(assert_equal(expected,actual,1e-4));
}

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_joint )
{
	// Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7);
	Ordering ordering;
	ordering += "x1","x3","x5","x7","x2","x6","x4";

	// Create the Bayes tree
	GaussianBayesNet chordalBayesNet = smoother.eliminate(ordering);
	GaussianISAM bayesTree(chordalBayesNet);

  // Conditional density elements reused by both tests
	Vector sigma = repeat(2, 0.786146);
  Matrix I = eye(2), A = -0.00429185*I;

  // Check the joint density P(x1,x7) factored as P(x1|x7)P(x7)
  GaussianBayesNet expected1 = simpleGaussian("x7", zero(2), sigmax7);
  push_front(expected1,"x1", zero(2), I, "x7", A, sigma);
	GaussianBayesNet actual1 = bayesTree.jointBayesNet<GaussianFactor>("x1","x7");
	CHECK(assert_equal(expected1,actual1,1e-4));

	// Check the joint density P(x7,x1) factored as P(x7|x1)P(x1)
  GaussianBayesNet expected2 = simpleGaussian("x1", zero(2), sigmax1);
  push_front(expected2,"x7", zero(2), I, "x1", A, sigma);
	GaussianBayesNet actual2 = bayesTree.jointBayesNet<GaussianFactor>("x7","x1");
	CHECK(assert_equal(expected2,actual2,1e-4));

	// Check the joint density P(x1,x4), i.e. with a root variable
  GaussianBayesNet expected3 = simpleGaussian("x4", zero(2), sigmax4);
	Vector sigma14 = repeat(2, 0.784465);
  Matrix A14 = -0.0769231*I;
  push_front(expected3,"x1", zero(2), I, "x4", A14, sigma14);
	GaussianBayesNet actual3 = bayesTree.jointBayesNet<GaussianFactor>("x1","x4");
	CHECK(assert_equal(expected3,actual3,1e-4));

	// Check the joint density P(x4,x1), i.e. with a root variable, factored the other way
  GaussianBayesNet expected4 = simpleGaussian("x1", zero(2), sigmax1);
	Vector sigma41 = repeat(2, 0.668096);
  Matrix A41 = -0.055794*I;
  push_front(expected4,"x4", zero(2), I, "x1", A41, sigma41);
	GaussianBayesNet actual4 = bayesTree.jointBayesNet<GaussianFactor>("x4","x1");
	CHECK(assert_equal(expected4,actual4,1e-4));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
