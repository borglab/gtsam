/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactorGraphB.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 **/

#include <tests/smallExample.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <string.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

double tol=1e-5;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( GaussianFactorGraph, equals ) {

  Ordering ordering; ordering += X(1),X(2),L(1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianFactorGraph fg2 = createGaussianFactorGraph(ordering);
  EXPECT(fg.equals(fg2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, error ) {
  Ordering ordering; ordering += X(1),X(2),L(1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  VectorValues cfg = createZeroDelta(ordering);

  // note the error is the same as in testNonlinearFactorGraph as a
  // zero delta config in the linear graph is equivalent to noisy in
  // non-linear, which is really linear under the hood
  double actual = fg.error(cfg);
  DOUBLES_EQUAL( 5.625, actual, 1e-9 );
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_x1 )
{
  Ordering ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);

  GaussianConditional::shared_ptr conditional;
  GaussianFactorGraph remaining;
  boost::tie(conditional,remaining) = inference::eliminateOne(fg, 0, EliminateQR);

  // create expected Conditional Gaussian
  Matrix I = 15*eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = ones(2);
  GaussianConditional expected(ordering[X(1)],15*d,R11,ordering[L(1)],S12,ordering[X(2)],S13,sigma);

  EXPECT(assert_equal(expected,*conditional,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_x2 )
{
  Ordering ordering; ordering += X(2),L(1),X(1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianConditional::shared_ptr actual = inference::eliminateOne(fg, 0, EliminateQR).first;

  // create expected Conditional Gaussian
  double sig = 0.0894427;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.2*I, S13 = -0.8*I;
  Vector d = Vector_(2, 0.2, -0.14)/sig, sigma = ones(2);
  GaussianConditional expected(ordering[X(2)],d,R11,ordering[L(1)],S12,ordering[X(1)],S13,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_l1 )
{
  Ordering ordering; ordering += L(1),X(1),X(2);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianConditional::shared_ptr actual = inference::eliminateOne(fg, 0, EliminateQR).first;

  // create expected Conditional Gaussian
  double sig = sqrt(2.0)/10.;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.5*I, S13 = -0.5*I;
  Vector d = Vector_(2, -0.1, 0.25)/sig, sigma = ones(2);
  GaussianConditional expected(ordering[L(1)],d,R11,ordering[X(1)],S12,ordering[X(2)],S13,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_x1_fast )
{
  Ordering ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianConditional::shared_ptr conditional;
  GaussianFactorGraph remaining;
  boost::tie(conditional,remaining) = inference::eliminateOne(fg, ordering[X(1)], EliminateQR);

  // create expected Conditional Gaussian
  Matrix I = 15*eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = ones(2);
  GaussianConditional expected(ordering[X(1)],15*d,R11,ordering[L(1)],S12,ordering[X(2)],S13,sigma);

  // Create expected remaining new factor
  JacobianFactor expectedFactor(1, Matrix_(4,2,
             4.714045207910318,                   0.,
                             0.,   4.714045207910318,
                             0.,                   0.,
                             0.,                   0.),
     2, Matrix_(4,2,
           -2.357022603955159,                   0.,
                            0.,  -2.357022603955159,
            7.071067811865475,                   0.,
                            0.,   7.071067811865475),
     Vector_(4, -0.707106781186547, 0.942809041582063, 0.707106781186547, -1.414213562373094), noiseModel::Unit::Create(4));

  EXPECT(assert_equal(expected,*conditional,tol));
  EXPECT(assert_equal((const GaussianFactor&)expectedFactor,*remaining.back(),tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_x2_fast )
{
  Ordering ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianConditional::shared_ptr actual = inference::eliminateOne(fg, ordering[X(2)], EliminateQR).first;

  // create expected Conditional Gaussian
  double sig = 0.0894427;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.2*I, S13 = -0.8*I;
  Vector d = Vector_(2, 0.2, -0.14)/sig, sigma = ones(2);
  GaussianConditional expected(ordering[X(2)],d,R11,ordering[X(1)],S13,ordering[L(1)],S12,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateOne_l1_fast )
{
  Ordering ordering; ordering += X(1),L(1),X(2);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianConditional::shared_ptr actual = inference::eliminateOne(fg, ordering[L(1)], EliminateQR).first;

  // create expected Conditional Gaussian
  double sig = sqrt(2.0)/10.;
  Matrix I = eye(2)/sig, R11 = I, S12 = -0.5*I, S13 = -0.5*I;
  Vector d = Vector_(2, -0.1, 0.25)/sig, sigma = ones(2);
  GaussianConditional expected(ordering[L(1)],d,R11,ordering[X(1)],S12,ordering[X(2)],S13,sigma);

  EXPECT(assert_equal(expected,*actual,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateAll )
{
	// create expected Chordal bayes Net
	Matrix I = eye(2);

  Ordering ordering;
  ordering += X(2),L(1),X(1);

	Vector d1 = Vector_(2, -0.1,-0.1);
	GaussianBayesNet expected = simpleGaussian(ordering[X(1)],d1,0.1);

	double sig1 = 0.149071;
	Vector d2 = Vector_(2, 0.0, 0.2)/sig1, sigma2 = ones(2);
	push_front(expected,ordering[L(1)],d2, I/sig1,ordering[X(1)], (-1)*I/sig1,sigma2);

	double sig2 = 0.0894427;
	Vector d3 = Vector_(2, 0.2, -0.14)/sig2, sigma3 = ones(2);
	push_front(expected,ordering[X(2)],d3, I/sig2,ordering[L(1)], (-0.2)*I/sig2, ordering[X(1)], (-0.8)*I/sig2, sigma3);

	// Check one ordering
	GaussianFactorGraph fg1 = createGaussianFactorGraph(ordering);
	GaussianBayesNet actual = *GaussianSequentialSolver(fg1).eliminate();
	EXPECT(assert_equal(expected,actual,tol));

  GaussianBayesNet actualQR = *GaussianSequentialSolver(fg1, true).eliminate();
  EXPECT(assert_equal(expected,actualQR,tol));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, copying )
{
  // Create a graph
  Ordering ordering; ordering += X(2),L(1),X(1);
  GaussianFactorGraph actual = createGaussianFactorGraph(ordering);

  // Copy the graph !
  GaussianFactorGraph copy = actual;

  // now eliminate the copy
  GaussianBayesNet actual1 = *GaussianSequentialSolver(copy).eliminate();

  // Create the same graph, but not by copying
  GaussianFactorGraph expected = createGaussianFactorGraph(ordering);

  // and check that original is still the same graph
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, CONSTRUCTOR_GaussianBayesNet )
{
  Ordering ord;
  ord += X(2),L(1),X(1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ord);

  // render with a given ordering
  GaussianBayesNet CBN = *GaussianSequentialSolver(fg).eliminate();

  // True GaussianFactorGraph
  GaussianFactorGraph fg2(CBN);
  GaussianBayesNet CBN2 = *GaussianSequentialSolver(fg2).eliminate();
  EXPECT(assert_equal(CBN,CBN2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, getOrdering)
{
  Ordering original; original += L(1),X(1),X(2);
  FactorGraph<IndexFactor> symbolic(createGaussianFactorGraph(original));
  Permutation perm(*inference::PermutationCOLAMD(VariableIndex(symbolic)));
  Ordering actual = original; actual.permuteWithInverse((*perm.inverse()));
  Ordering expected; expected += L(1),X(2),X(1);
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, optimize_Cholesky )
{
  // create an ordering
  Ordering ord; ord += X(2),L(1),X(1);

  // create a graph
	GaussianFactorGraph fg = createGaussianFactorGraph(ord);

	// optimize the graph
	VectorValues actual = *GaussianSequentialSolver(fg, false).optimize();

	// verify
	VectorValues expected = createCorrectDelta(ord);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, optimize_QR )
{
  // create an ordering
  Ordering ord; ord += X(2),L(1),X(1);

  // create a graph
	GaussianFactorGraph fg = createGaussianFactorGraph(ord);

	// optimize the graph
	VectorValues actual = *GaussianSequentialSolver(fg, true).optimize();

	// verify
	VectorValues expected = createCorrectDelta(ord);

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, combine)
{
  // create an ordering
  Ordering ord; ord += X(2),L(1),X(1);

  // create a test graph
	GaussianFactorGraph fg1 = createGaussianFactorGraph(ord);

	// create another factor graph
	GaussianFactorGraph fg2 = createGaussianFactorGraph(ord);

	// get sizes
	size_t size1 = fg1.size();
	size_t size2 = fg2.size();

	// combine them
	fg1.combine(fg2);

	EXPECT(size1+size2 == fg1.size());
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, combine2)
{
  // create an ordering
  Ordering ord; ord += X(2),L(1),X(1);

	// create a test graph
	GaussianFactorGraph fg1 = createGaussianFactorGraph(ord);

	// create another factor graph
	GaussianFactorGraph fg2 = createGaussianFactorGraph(ord);

	// get sizes
	size_t size1 = fg1.size();
	size_t size2 = fg2.size();

	// combine them
	GaussianFactorGraph fg3 = GaussianFactorGraph::combine2(fg1, fg2);

	EXPECT(size1+size2 == fg3.size());
}

/* ************************************************************************* */
// print a vector of ints if needed for debugging
void print(vector<int> v) {
	for (size_t k = 0; k < v.size(); k++)
		cout << v[k] << " ";
	cout << endl;
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother)
{
	GaussianFactorGraph fg1 = createSmoother(2).first;
	LONGS_EQUAL(3,fg1.size());
	GaussianFactorGraph fg2 = createSmoother(3).first;
	LONGS_EQUAL(5,fg2.size());
}

/* ************************************************************************* */
double error(const VectorValues& x) {
  // create an ordering
  Ordering ord; ord += X(2),L(1),X(1);

	GaussianFactorGraph fg = createGaussianFactorGraph(ord);
	return fg.error(x);
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, multiplication )
{
  // create an ordering
  Ordering ord; ord += X(2),L(1),X(1);

  GaussianFactorGraph A = createGaussianFactorGraph(ord);
  VectorValues x = createCorrectDelta(ord);
  Errors actual = A * x;
  Errors expected;
  expected += Vector_(2,-1.0,-1.0);
  expected += Vector_(2, 2.0,-1.0);
  expected += Vector_(2, 0.0, 1.0);
  expected += Vector_(2,-1.0, 1.5);
	EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
// Extra test on elimination prompted by Michael's email to Frank 1/4/2010
TEST( GaussianFactorGraph, elimination )
{
  Ordering ord;
  ord += X(1), X(2);
	// Create Gaussian Factor Graph
	GaussianFactorGraph fg;
	Matrix Ap = eye(1), An = eye(1) * -1;
	Vector b = Vector_(1, 0.0);
  SharedDiagonal sigma = noiseModel::Isotropic::Sigma(1,2.0);
	fg.add(ord[X(1)], An, ord[X(2)], Ap, b, sigma);
	fg.add(ord[X(1)], Ap, b, sigma);
	fg.add(ord[X(2)], Ap, b, sigma);

	// Eliminate
	GaussianBayesNet bayesNet = *GaussianSequentialSolver(fg).eliminate();

	// Check sigma
	EXPECT_DOUBLES_EQUAL(1.0,bayesNet[ord[X(2)]]->get_sigmas()(0),1e-5);

	// Check matrix
	Matrix R;Vector d;
	boost::tie(R,d) = matrix(bayesNet);
	Matrix expected = Matrix_(2,2,
			0.707107,	-0.353553,
			0.0,	 0.612372);
	Matrix expected2 = Matrix_(2,2,
			0.707107,	-0.353553,
			0.0,	 -0.612372);
	EXPECT(equal_with_abs_tol(expected, R, 1e-6) || equal_with_abs_tol(expected2, R, 1e-6));
}

 /* ************************************************************************* */
// Tests ported from ConstrainedGaussianFactorGraph
/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_simple )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createSimpleConstraintGraph();
	EXPECT(hasConstraints(fg));


	// eliminate and solve
	VectorValues actual = *GaussianSequentialSolver(fg).optimize();

	// verify
	VectorValues expected = createSimpleConstraintValues();
	EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_single )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createSingleConstraintGraph();
	EXPECT(hasConstraints(fg));

	// eliminate and solve
	VectorValues actual = *GaussianSequentialSolver(fg).optimize();

	// verify
	VectorValues expected = createSingleConstraintValues();
	EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, constrained_multi1 )
{
	// get a graph with a constraint in it
	GaussianFactorGraph fg = createMultiConstraintGraph();
	EXPECT(hasConstraints(fg));

	// eliminate and solve
  VectorValues actual = *GaussianSequentialSolver(fg).optimize();

	// verify
	VectorValues expected = createMultiConstraintValues();
	EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */

static SharedDiagonal model = noiseModel::Isotropic::Sigma(2,1);

/* ************************************************************************* */
TEST(GaussianFactorGraph, replace)
{
  Ordering ord; ord += X(1),X(2),X(3),X(4),X(5),X(6);
	SharedDiagonal noise(noiseModel::Isotropic::Sigma(3, 1.0));

	GaussianFactorGraph::sharedFactor f1(new JacobianFactor(
	    ord[X(1)], eye(3,3), ord[X(2)], eye(3,3), zero(3), noise));
	GaussianFactorGraph::sharedFactor f2(new JacobianFactor(
	    ord[X(2)], eye(3,3), ord[X(3)], eye(3,3), zero(3), noise));
	GaussianFactorGraph::sharedFactor f3(new JacobianFactor(
	    ord[X(3)], eye(3,3), ord[X(4)], eye(3,3), zero(3), noise));
	GaussianFactorGraph::sharedFactor f4(new JacobianFactor(
	    ord[X(5)], eye(3,3), ord[X(6)], eye(3,3), zero(3), noise));

	GaussianFactorGraph actual;
	actual.push_back(f1);
	actual.push_back(f2);
	actual.push_back(f3);
	actual.replace(0, f4);

	GaussianFactorGraph expected;
	expected.push_back(f4);
	expected.push_back(f2);
	expected.push_back(f3);

	EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother2)
{
  using namespace example;
  GaussianFactorGraph fg2;
  Ordering ordering;
  boost::tie(fg2,ordering) = createSmoother(3);
  LONGS_EQUAL(5,fg2.size());

  // eliminate
  vector<Index> x3var; x3var.push_back(ordering[X(3)]);
  vector<Index> x1var; x1var.push_back(ordering[X(1)]);
  GaussianBayesNet p_x3 = *GaussianSequentialSolver(
      *GaussianSequentialSolver(fg2).jointFactorGraph(x3var)).eliminate();
  GaussianBayesNet p_x1 = *GaussianSequentialSolver(
      *GaussianSequentialSolver(fg2).jointFactorGraph(x1var)).eliminate();
  CHECK(assert_equal(*p_x1.back(),*p_x3.front())); // should be the same because of symmetry
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, hasConstraints)
{
	FactorGraph<GaussianFactor> fgc1 = createMultiConstraintGraph();
	EXPECT(hasConstraints(fgc1));

	FactorGraph<GaussianFactor> fgc2 = createSimpleConstraintGraph() ;
	EXPECT(hasConstraints(fgc2));

	Ordering ordering; ordering += X(1), X(2), L(1);
	GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
	EXPECT(!hasConstraints(fg));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
