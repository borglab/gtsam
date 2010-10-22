/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianISAM.cpp
 * @brief   Unit tests for GaussianISAM
 * @author  Michael Kaess
 */

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/inference/ISAM-inl.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/slam/smallExample.h>

using namespace std;
using namespace gtsam;
using namespace example;

/* ************************************************************************* */
// Some numbers that should be consistent among all smoother tests

double sigmax1 = 0.786153, sigmax2 = 1.0/1.47292, sigmax3 = 0.671512, sigmax4 =
		0.669534, sigmax5 = sigmax3, sigmax6 = sigmax2, sigmax7 = sigmax1;

const double tol = 1e-4;

/* ************************************************************************* */
TEST( ISAM, iSAM_smoother )
{
  Ordering ordering;
  for (int t = 1; t <= 7; t++) ordering += Symbol('x', t);

  // Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7, ordering).first;

	// run iSAM for every factor
	GaussianISAM actual;
	BOOST_FOREACH(boost::shared_ptr<GaussianFactor> factor, smoother) {
		GaussianFactorGraph factorGraph;
		factorGraph.push_back(factor);
		actual.update(factorGraph);
	}

	// Create expected Bayes Tree by solving smoother with "natural" ordering
	GaussianISAM expected(*GaussianSequentialSolver(smoother).eliminate());

	// Check whether BayesTree is correct
	CHECK(assert_equal(expected, actual));

	// obtain solution
	VectorValues e(vector<size_t>(7,2)); // expected solution
	e.makeZero();
	VectorValues optimized = optimize(actual); // actual solution
	CHECK(assert_equal(e, optimized));
}

/* ************************************************************************* */
// SL-FIX TEST( ISAM, iSAM_smoother2 )
//{
//	// Create smoother with 7 nodes
//	GaussianFactorGraph smoother = createSmoother(7);
//
//	// Create initial tree from first 4 timestamps in reverse order !
//	Ordering ord; ord += "x4","x3","x2","x1";
//	GaussianFactorGraph factors1;
//	for (int i=0;i<7;i++) factors1.push_back(smoother[i]);
//	GaussianISAM actual(*Inference::Eliminate(factors1));
//
//	// run iSAM with remaining factors
//	GaussianFactorGraph factors2;
//	for (int i=7;i<13;i++) factors2.push_back(smoother[i]);
//	actual.update(factors2);
//
//	// Create expected Bayes Tree by solving smoother with "natural" ordering
//	Ordering ordering;
//	for (int t = 1; t <= 7; t++) ordering += symbol('x', t);
//	GaussianISAM expected(smoother.eliminate(ordering));
//
//	CHECK(assert_equal(expected, actual));
//}

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
C1 x6 x7
C2   x5 : x6
C3     x4 : x5
C4       x3 : x4
C5         x2 : x3
C6           x1 : x2
**************************************************************************** */
TEST( BayesTree, linear_smoother_shortcuts )
{
	// Create smoother with 7 nodes
  Ordering ordering;
	GaussianFactorGraph smoother;
	boost::tie(smoother, ordering) = createSmoother(7);

	// eliminate using the "natural" ordering
	GaussianBayesNet chordalBayesNet = *GaussianSequentialSolver(smoother).eliminate();

	// Create the Bayes tree
	GaussianISAM bayesTree(chordalBayesNet);
	LONGS_EQUAL(6,bayesTree.size());

	// Check the conditional P(Root|Root)
	GaussianBayesNet empty;
	GaussianISAM::sharedClique R = bayesTree.root();
	GaussianBayesNet actual1 = R->shortcut(R);
	CHECK(assert_equal(empty,actual1,tol));

	// Check the conditional P(C2|Root)
	GaussianISAM::sharedClique C2 = bayesTree[ordering["x5"]];
	GaussianBayesNet actual2 = C2->shortcut(R);
	CHECK(assert_equal(empty,actual2,tol));

	// Check the conditional P(C3|Root)
	double sigma3 = 0.61808;
	Matrix A56 = Matrix_(2,2,-0.382022,0.,0.,-0.382022);
	GaussianBayesNet expected3;
	push_front(expected3,ordering["x5"], zero(2), eye(2)/sigma3, ordering["x6"], A56/sigma3, ones(2));
	GaussianISAM::sharedClique C3 = bayesTree[ordering["x4"]];
	GaussianBayesNet actual3 = C3->shortcut(R);
	CHECK(assert_equal(expected3,actual3,tol));

	// Check the conditional P(C4|Root)
	double sigma4 = 0.661968;
	Matrix A46 = Matrix_(2,2,-0.146067,0.,0.,-0.146067);
	GaussianBayesNet expected4;
	push_front(expected4, ordering["x4"], zero(2), eye(2)/sigma4, ordering["x6"], A46/sigma4, ones(2));
	GaussianISAM::sharedClique C4 = bayesTree[ordering["x3"]];
	GaussianBayesNet actual4 = C4->shortcut(R);
	CHECK(assert_equal(expected4,actual4,tol));
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

************************************************************************* */
TEST( BayesTree, balanced_smoother_marginals )
{
  // Create smoother with 7 nodes
  Ordering ordering;
  ordering += "x1","x3","x5","x7","x2","x6","x4";
  GaussianFactorGraph smoother = createSmoother(7, ordering).first;

  // Create the Bayes tree
  GaussianBayesNet chordalBayesNet = *GaussianSequentialSolver(smoother).eliminate();

	VectorValues expectedSolution(7, 2);
	expectedSolution.makeZero();
	VectorValues actualSolution = optimize(chordalBayesNet);
	CHECK(assert_equal(expectedSolution,actualSolution,tol));

	// Create the Bayes tree
	GaussianISAM bayesTree(chordalBayesNet);
	LONGS_EQUAL(4,bayesTree.size());

	double tol=1e-5;

	// Check marginal on x1
	GaussianBayesNet expected1 = simpleGaussian(ordering["x1"], zero(2), sigmax1);
	GaussianBayesNet actual1 = *bayesTree.marginalBayesNet(ordering["x1"]);
	CHECK(assert_equal(expected1,actual1,tol));

	// Check marginal on x2
	double sigx2 = 0.68712938; // FIXME: this should be corrected analytically
	GaussianBayesNet expected2 = simpleGaussian(ordering["x2"], zero(2), sigx2);
	GaussianBayesNet actual2 = *bayesTree.marginalBayesNet(ordering["x2"]);
	CHECK(assert_equal(expected2,actual2,tol)); // FAILS

	// Check marginal on x3
	GaussianBayesNet expected3 = simpleGaussian(ordering["x3"], zero(2), sigmax3);
	GaussianBayesNet actual3 = *bayesTree.marginalBayesNet(ordering["x3"]);
	CHECK(assert_equal(expected3,actual3,tol));

	// Check marginal on x4
	GaussianBayesNet expected4 = simpleGaussian(ordering["x4"], zero(2), sigmax4);
	GaussianBayesNet actual4 = *bayesTree.marginalBayesNet(ordering["x4"]);
	CHECK(assert_equal(expected4,actual4,tol));

	// Check marginal on x7 (should be equal to x1)
	GaussianBayesNet expected7 = simpleGaussian(ordering["x7"], zero(2), sigmax7);
	GaussianBayesNet actual7 = *bayesTree.marginalBayesNet(ordering["x7"]);
	CHECK(assert_equal(expected7,actual7,tol));
}

/* ************************************************************************* */
TEST( BayesTree, balanced_smoother_shortcuts )
{
	// Create smoother with 7 nodes
  Ordering ordering;
  ordering += "x1","x3","x5","x7","x2","x6","x4";
	GaussianFactorGraph smoother = createSmoother(7, ordering).first;

	// Create the Bayes tree
	GaussianBayesNet chordalBayesNet = *GaussianSequentialSolver(smoother).eliminate();
	GaussianISAM bayesTree(chordalBayesNet);

	// Check the conditional P(Root|Root)
	GaussianBayesNet empty;
	GaussianISAM::sharedClique R = bayesTree.root();
	GaussianBayesNet actual1 = R->shortcut(R);
	CHECK(assert_equal(empty,actual1,tol));

	// Check the conditional P(C2|Root)
	GaussianISAM::sharedClique C2 = bayesTree[ordering["x3"]];
	GaussianBayesNet actual2 = C2->shortcut(R);
	CHECK(assert_equal(empty,actual2,tol));

	// Check the conditional P(C3|Root), which should be equal to P(x2|x4)
	GaussianConditional::shared_ptr p_x2_x4 = chordalBayesNet[ordering["x2"]];
	GaussianBayesNet expected3; expected3.push_back(p_x2_x4);
	GaussianISAM::sharedClique C3 = bayesTree[ordering["x1"]];
	GaussianBayesNet actual3 = C3->shortcut(R);
	CHECK(assert_equal(expected3,actual3,tol));
}

///* ************************************************************************* */
//TEST( BayesTree, balanced_smoother_clique_marginals )
//{
//  // Create smoother with 7 nodes
//  Ordering ordering;
//  ordering += "x1","x3","x5","x7","x2","x6","x4";
//  GaussianFactorGraph smoother = createSmoother(7, ordering).first;
//
//  // Create the Bayes tree
//  GaussianBayesNet chordalBayesNet = *GaussianSequentialSolver(smoother).eliminate();
//  GaussianISAM bayesTree(chordalBayesNet);
//
//	// Check the clique marginal P(C3)
//	double sigmax2_alt = 1/1.45533; // THIS NEEDS TO BE CHECKED!
//	GaussianBayesNet expected = simpleGaussian(ordering["x2"],zero(2),sigmax2_alt);
//	push_front(expected,ordering["x1"], zero(2), eye(2)*sqrt(2), ordering["x2"], -eye(2)*sqrt(2)/2, ones(2));
//	GaussianISAM::sharedClique R = bayesTree.root(), C3 = bayesTree[ordering["x1"]];
//	GaussianFactorGraph marginal = C3->marginal(R);
//	GaussianVariableIndex varIndex(marginal);
//	Permutation toFront(Permutation::PullToFront(C3->keys(), varIndex.size()));
//	Permutation toFrontInverse(*toFront.inverse());
//	varIndex.permute(toFront);
//	BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, marginal) {
//	  factor->permuteWithInverse(toFrontInverse); }
//	GaussianBayesNet actual = *Inference::EliminateUntil(marginal, C3->keys().size(), varIndex);
//	actual.permuteWithInverse(toFront);
//	CHECK(assert_equal(expected,actual,tol));
//}

/* ************************************************************************* */
// SL-FIX TEST( BayesTree, balanced_smoother_joint )
//{
//	// Create smoother with 7 nodes
//	GaussianFactorGraph smoother = createSmoother(7);
//	Ordering ordering;
//	ordering += "x1","x3","x5","x7","x2","x6","x4";
//
//	// Create the Bayes tree, expected to look like:
//	//	 x5 x6 x4
//	//	   x3 x2 : x4
//	//	     x1 : x2
//	//	   x7 : x6
//	GaussianBayesNet chordalBayesNet = smoother.eliminate(ordering);
//	GaussianISAM bayesTree(chordalBayesNet);
//
//	// Conditional density elements reused by both tests
//	const Vector sigma = ones(2);
//	const Matrix I = eye(2), A = -0.00429185*I;
//
//	// Check the joint density P(x1,x7) factored as P(x1|x7)P(x7)
//	GaussianBayesNet expected1;
//	// Why does the sign get flipped on the prior?
//	GaussianConditional::shared_ptr
//		parent1(new GaussianConditional("x7", zero(2), -1*I/sigmax7, ones(2)));
//	expected1.push_front(parent1);
//	push_front(expected1,"x1", zero(2), I/sigmax7, "x7", A/sigmax7, sigma);
//	GaussianBayesNet actual1 = bayesTree.jointBayesNet<GaussianFactor>("x1","x7");
//	CHECK(assert_equal(expected1,actual1,tol));
//
//	// Check the joint density P(x7,x1) factored as P(x7|x1)P(x1)
//	GaussianBayesNet expected2;
//	GaussianConditional::shared_ptr
//			parent2(new GaussianConditional("x1", zero(2), -1*I/sigmax1, ones(2)));
//		expected2.push_front(parent2);
//	push_front(expected2,"x7", zero(2), I/sigmax1, "x1", A/sigmax1, sigma);
//	GaussianBayesNet actual2 = bayesTree.jointBayesNet<GaussianFactor>("x7","x1");
//	CHECK(assert_equal(expected2,actual2,tol));
//
//	// Check the joint density P(x1,x4), i.e. with a root variable
//	GaussianBayesNet expected3;
//	GaussianConditional::shared_ptr
//			parent3(new GaussianConditional("x4", zero(2), I/sigmax4, ones(2)));
//		expected3.push_front(parent3);
//	double sig14 = 0.784465;
//	Matrix A14 = -0.0769231*I;
//	push_front(expected3,"x1", zero(2), I/sig14, "x4", A14/sig14, sigma);
//	GaussianBayesNet actual3 = bayesTree.jointBayesNet<GaussianFactor>("x1","x4");
//	CHECK(assert_equal(expected3,actual3,tol));
//
//	// Check the joint density P(x4,x1), i.e. with a root variable, factored the other way
//	GaussianBayesNet expected4;
//	GaussianConditional::shared_ptr
//			parent4(new GaussianConditional("x1", zero(2), -1.0*I/sigmax1, ones(2)));
//		expected4.push_front(parent4);
//	double sig41 = 0.668096;
//	Matrix A41 = -0.055794*I;
//	push_front(expected4,"x4", zero(2), I/sig41, "x1", A41/sig41, sigma);
//	GaussianBayesNet actual4 = bayesTree.jointBayesNet<GaussianFactor>("x4","x1");
//	CHECK(assert_equal(expected4,actual4,tol));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
