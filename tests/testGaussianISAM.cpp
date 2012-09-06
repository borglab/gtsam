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

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Ordering.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/inference/ISAM.h>
#include <gtsam/geometry/Rot2.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
// Some numbers that should be consistent among all smoother tests

static double sigmax1 = 0.786153, sigmax2 = 1.0/1.47292, sigmax3 = 0.671512, sigmax4 =
		0.669534 /*, sigmax5 = sigmax3, sigmax6 = sigmax2*/, sigmax7 = sigmax1;

static const double tol = 1e-4;

/* ************************************************************************* */
TEST( ISAM, iSAM_smoother )
{
  Ordering ordering;
  for (int t = 1; t <= 7; t++) ordering += X(t);

  // Create smoother with 7 nodes
	GaussianFactorGraph smoother = createSmoother(7, ordering).first;

	// run iSAM for every factor
	GaussianISAM actual;
	BOOST_FOREACH(boost::shared_ptr<GaussianFactor> factor, smoother) {
		GaussianFactorGraph factorGraph;
		factorGraph.push_back(factor);
		actual.update(factorGraph);
	}

	BayesTree<GaussianConditional>::shared_ptr bayesTree = GaussianMultifrontalSolver(smoother).eliminate();
	// Create expected Bayes Tree by solving smoother with "natural" ordering
	GaussianISAM expected(*bayesTree);

	// Check whether BayesTree is correct
	EXPECT(assert_equal(expected, actual));

	// obtain solution
	VectorValues e(VectorValues::Zero(7,2)); // expected solution
	VectorValues optimized = optimize(actual); // actual solution
	EXPECT(assert_equal(e, optimized));
}

/* ************************************************************************* *
 Bayes tree for smoother with "natural" ordering:
C1 x6 x7
C2   x5 : x6
C3     x4 : x5
C4       x3 : x4
C5         x2 : x3
C6           x1 : x2
**************************************************************************** */
TEST_UNSAFE( BayesTree, linear_smoother_shortcuts )
{
	// Create smoother with 7 nodes
  Ordering ordering;
	GaussianFactorGraph smoother;
	boost::tie(smoother, ordering) = createSmoother(7);

	BayesTree<GaussianConditional> bayesTree = *GaussianMultifrontalSolver(smoother).eliminate();

	// Create the Bayes tree
	GaussianISAM isamTree(bayesTree);
	LONGS_EQUAL(6,isamTree.size());

	// Check the conditional P(Root|Root)
	GaussianBayesNet empty;
	GaussianISAM::sharedClique R = isamTree.root();
	GaussianBayesNet actual1 = GaussianISAM::shortcut(R,R);
	EXPECT(assert_equal(empty,actual1,tol));

	// Check the conditional P(C2|Root)
	GaussianISAM::sharedClique C2 = isamTree[ordering[X(5)]];
	GaussianBayesNet actual2 = GaussianISAM::shortcut(C2,R);
	EXPECT(assert_equal(empty,actual2,tol));

	// Check the conditional P(C3|Root)
	double sigma3 = 0.61808;
	Matrix A56 = Matrix_(2,2,-0.382022,0.,0.,-0.382022);
	GaussianBayesNet expected3;
	push_front(expected3,ordering[X(5)], zero(2), eye(2)/sigma3, ordering[X(6)], A56/sigma3, ones(2));
	GaussianISAM::sharedClique C3 = isamTree[ordering[X(4)]];
	GaussianBayesNet actual3 = GaussianISAM::shortcut(C3,R);
	EXPECT(assert_equal(expected3,actual3,tol));

	// Check the conditional P(C4|Root)
	double sigma4 = 0.661968;
	Matrix A46 = Matrix_(2,2,-0.146067,0.,0.,-0.146067);
	GaussianBayesNet expected4;
	push_front(expected4, ordering[X(4)], zero(2), eye(2)/sigma4, ordering[X(6)], A46/sigma4, ones(2));
	GaussianISAM::sharedClique C4 = isamTree[ordering[X(3)]];
	GaussianBayesNet actual4 = GaussianISAM::shortcut(C4,R);
	EXPECT(assert_equal(expected4,actual4,tol));
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
TEST_UNSAFE( BayesTree, balanced_smoother_marginals )
{
  // Create smoother with 7 nodes
  Ordering ordering;
  ordering += X(1),X(3),X(5),X(7),X(2),X(6),X(4);
  GaussianFactorGraph smoother = createSmoother(7, ordering).first;

  // Create the Bayes tree
  BayesTree<GaussianConditional> chordalBayesNet = *GaussianMultifrontalSolver(smoother).eliminate();

	VectorValues expectedSolution(VectorValues::Zero(7,2));
	VectorValues actualSolution = optimize(chordalBayesNet);
	EXPECT(assert_equal(expectedSolution,actualSolution,tol));

	// Create the Bayes tree
	GaussianISAM bayesTree(chordalBayesNet);
	LONGS_EQUAL(4,bayesTree.size());

	double tol=1e-5;

	// Check marginal on x1
	GaussianBayesNet expected1 = simpleGaussian(ordering[X(1)], zero(2), sigmax1);
	GaussianBayesNet actual1 = *bayesTree.marginalBayesNet(ordering[X(1)]);
	Matrix expectedCovarianceX1 = eye(2,2) * (sigmax1 * sigmax1);
	Matrix actualCovarianceX1;
	actualCovarianceX1 = bayesTree.marginalCovariance(ordering[X(1)]);
	EXPECT(assert_equal(expectedCovarianceX1, actualCovarianceX1, tol));
	EXPECT(assert_equal(expected1,actual1,tol));

	// Check marginal on x2
	double sigx2 = 0.68712938; // FIXME: this should be corrected analytically
	GaussianBayesNet expected2 = simpleGaussian(ordering[X(2)], zero(2), sigx2);
	GaussianBayesNet actual2 = *bayesTree.marginalBayesNet(ordering[X(2)]);
	Matrix expectedCovarianceX2 = eye(2,2) * (sigx2 * sigx2);
	Matrix actualCovarianceX2;
	actualCovarianceX2 = bayesTree.marginalCovariance(ordering[X(2)]);
	EXPECT(assert_equal(expectedCovarianceX2, actualCovarianceX2, tol));
	EXPECT(assert_equal(expected2,actual2,tol));

	// Check marginal on x3
	GaussianBayesNet expected3 = simpleGaussian(ordering[X(3)], zero(2), sigmax3);
	GaussianBayesNet actual3 = *bayesTree.marginalBayesNet(ordering[X(3)]);
	Matrix expectedCovarianceX3 = eye(2,2) * (sigmax3 * sigmax3);
	Matrix actualCovarianceX3;
	actualCovarianceX3 = bayesTree.marginalCovariance(ordering[X(3)]);
	EXPECT(assert_equal(expectedCovarianceX3, actualCovarianceX3, tol));
	EXPECT(assert_equal(expected3,actual3,tol));

	// Check marginal on x4
	GaussianBayesNet expected4 = simpleGaussian(ordering[X(4)], zero(2), sigmax4);
	GaussianBayesNet actual4 = *bayesTree.marginalBayesNet(ordering[X(4)]);
	Matrix expectedCovarianceX4 = eye(2,2) * (sigmax4 * sigmax4);
	Matrix actualCovarianceX4;
	actualCovarianceX4 = bayesTree.marginalCovariance(ordering[X(4)]);
	EXPECT(assert_equal(expectedCovarianceX4, actualCovarianceX4, tol));
	EXPECT(assert_equal(expected4,actual4,tol));

	// Check marginal on x7 (should be equal to x1)
	GaussianBayesNet expected7 = simpleGaussian(ordering[X(7)], zero(2), sigmax7);
	GaussianBayesNet actual7 = *bayesTree.marginalBayesNet(ordering[X(7)]);
	Matrix expectedCovarianceX7 = eye(2,2) * (sigmax7 * sigmax7);
	Matrix actualCovarianceX7;
	actualCovarianceX7 = bayesTree.marginalCovariance(ordering[X(7)]);
	EXPECT(assert_equal(expectedCovarianceX7, actualCovarianceX7, tol));
	EXPECT(assert_equal(expected7,actual7,tol));
}

/* ************************************************************************* */
TEST_UNSAFE( BayesTree, balanced_smoother_shortcuts )
{
	// Create smoother with 7 nodes
  Ordering ordering;
  ordering += X(1),X(3),X(5),X(7),X(2),X(6),X(4);
	GaussianFactorGraph smoother = createSmoother(7, ordering).first;

	// Create the Bayes tree
	BayesTree<GaussianConditional> bayesTree = *GaussianMultifrontalSolver(smoother).eliminate();
	GaussianISAM isamTree(bayesTree);

	// Check the conditional P(Root|Root)
	GaussianBayesNet empty;
	GaussianISAM::sharedClique R = isamTree.root();
	GaussianBayesNet actual1 = GaussianISAM::shortcut(R,R);
	EXPECT(assert_equal(empty,actual1,tol));

	// Check the conditional P(C2|Root)
	GaussianISAM::sharedClique C2 = isamTree[ordering[X(3)]];
	GaussianBayesNet actual2 = GaussianISAM::shortcut(C2,R);
	EXPECT(assert_equal(empty,actual2,tol));

	// Check the conditional P(C3|Root), which should be equal to P(x2|x4)
	/** TODO: Note for multifrontal conditional:
	 * p_x2_x4 is now an element conditional of the multifrontal conditional bayesTree[ordering[X(2)]]->conditional()
	 * We don't know yet how to take it out.
	 */
//	GaussianConditional::shared_ptr p_x2_x4 = bayesTree[ordering[X(2)]]->conditional();
//	p_x2_x4->print("Conditional p_x2_x4: ");
//	GaussianBayesNet expected3(p_x2_x4);
//	GaussianISAM::sharedClique C3 = isamTree[ordering[X(1)]];
//	GaussianBayesNet actual3 = GaussianISAM::shortcut(C3,R);
//	EXPECT(assert_equal(expected3,actual3,tol));
}

///* ************************************************************************* */
//TEST( BayesTree, balanced_smoother_clique_marginals )
//{
//  // Create smoother with 7 nodes
//  Ordering ordering;
//  ordering += X(1),X(3),X(5),X(7),X(2),X(6),X(4);
//  GaussianFactorGraph smoother = createSmoother(7, ordering).first;
//
//  // Create the Bayes tree
//  GaussianBayesNet chordalBayesNet = *GaussianSequentialSolver(smoother).eliminate();
//  GaussianISAM bayesTree(chordalBayesNet);
//
//	// Check the clique marginal P(C3)
//	double sigmax2_alt = 1/1.45533; // THIS NEEDS TO BE CHECKED!
//	GaussianBayesNet expected = simpleGaussian(ordering[X(2)],zero(2),sigmax2_alt);
//	push_front(expected,ordering[X(1)], zero(2), eye(2)*sqrt(2), ordering[X(2)], -eye(2)*sqrt(2)/2, ones(2));
//	GaussianISAM::sharedClique R = bayesTree.root(), C3 = bayesTree[ordering[X(1)]];
//	GaussianFactorGraph marginal = C3->marginal(R);
//	GaussianVariableIndex varIndex(marginal);
//	Permutation toFront(Permutation::PullToFront(C3->keys(), varIndex.size()));
//	Permutation toFrontInverse(*toFront.inverse());
//	varIndex.permute(toFront);
//	BOOST_FOREACH(const GaussianFactor::shared_ptr& factor, marginal) {
//	  factor->permuteWithInverse(toFrontInverse); }
//	GaussianBayesNet actual = *inference::EliminateUntil(marginal, C3->keys().size(), varIndex);
//	actual.permuteWithInverse(toFront);
//	EXPECT(assert_equal(expected,actual,tol));
//}

/* ************************************************************************* */
TEST_UNSAFE( BayesTree, balanced_smoother_joint )
{
	// Create smoother with 7 nodes
	Ordering ordering;
	ordering += X(1),X(3),X(5),X(7),X(2),X(6),X(4);
	GaussianFactorGraph smoother = createSmoother(7, ordering).first;

	// Create the Bayes tree, expected to look like:
	//	 x5 x6 x4
	//	   x3 x2 : x4
	//	     x1 : x2
	//	   x7 : x6
	BayesTree<GaussianConditional> chordalBayesNet = *GaussianMultifrontalSolver(smoother).eliminate();
	GaussianISAM bayesTree(chordalBayesNet);

	// Conditional density elements reused by both tests
	const Vector sigma = ones(2);
	const Matrix I = eye(2), A = -0.00429185*I;

	// Check the joint density P(x1,x7) factored as P(x1|x7)P(x7)
	GaussianBayesNet expected1;
	// Why does the sign get flipped on the prior?
	GaussianConditional::shared_ptr
		parent1(new GaussianConditional(ordering[X(7)], zero(2), -1*I/sigmax7, ones(2)));
	expected1.push_front(parent1);
	push_front(expected1,ordering[X(1)], zero(2), I/sigmax7, ordering[X(7)], A/sigmax7, sigma);
	GaussianBayesNet actual1 = *bayesTree.jointBayesNet(ordering[X(1)],ordering[X(7)]);
	EXPECT(assert_equal(expected1,actual1,tol));

//	// Check the joint density P(x7,x1) factored as P(x7|x1)P(x1)
//	GaussianBayesNet expected2;
//	GaussianConditional::shared_ptr
//			parent2(new GaussianConditional(ordering[X(1)], zero(2), -1*I/sigmax1, ones(2)));
//		expected2.push_front(parent2);
//	push_front(expected2,ordering[X(7)], zero(2), I/sigmax1, ordering[X(1)], A/sigmax1, sigma);
//	GaussianBayesNet actual2 = *bayesTree.jointBayesNet(ordering[X(7)],ordering[X(1)]);
//	EXPECT(assert_equal(expected2,actual2,tol));

	// Check the joint density P(x1,x4), i.e. with a root variable
	GaussianBayesNet expected3;
	GaussianConditional::shared_ptr
			parent3(new GaussianConditional(ordering[X(4)], zero(2), I/sigmax4, ones(2)));
		expected3.push_front(parent3);
	double sig14 = 0.784465;
	Matrix A14 = -0.0769231*I;
	push_front(expected3,ordering[X(1)], zero(2), I/sig14, ordering[X(4)], A14/sig14, sigma);
	GaussianBayesNet actual3 = *bayesTree.jointBayesNet(ordering[X(1)],ordering[X(4)]);
	EXPECT(assert_equal(expected3,actual3,tol));

//	// Check the joint density P(x4,x1), i.e. with a root variable, factored the other way
//	GaussianBayesNet expected4;
//	GaussianConditional::shared_ptr
//			parent4(new GaussianConditional(ordering[X(1)], zero(2), -1.0*I/sigmax1, ones(2)));
//		expected4.push_front(parent4);
//	double sig41 = 0.668096;
//	Matrix A41 = -0.055794*I;
//	push_front(expected4,ordering[X(4)], zero(2), I/sig41, ordering[X(1)], A41/sig41, sigma);
//	GaussianBayesNet actual4 = *bayesTree.jointBayesNet(ordering[X(4)],ordering[X(1)]);
//	EXPECT(assert_equal(expected4,actual4,tol));
}

/* ************************************************************************* */
TEST_UNSAFE(BayesTree, simpleMarginal)
{
  GaussianFactorGraph gfg;

  Matrix A12 = Rot2::fromDegrees(45.0).matrix();

  gfg.add(0, eye(2), zero(2), noiseModel::Isotropic::Sigma(2, 1.0));
  gfg.add(0, -eye(2), 1, eye(2), ones(2), noiseModel::Isotropic::Sigma(2, 1.0));
  gfg.add(1, -eye(2), 2, A12, ones(2), noiseModel::Isotropic::Sigma(2, 1.0));

  Matrix expected(GaussianSequentialSolver(gfg).marginalCovariance(2));
  Matrix actual(GaussianMultifrontalSolver(gfg).marginalCovariance(2));

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
