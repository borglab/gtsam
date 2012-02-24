/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactorGraph.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 **/

#include <string.h>
#include <iostream>
using namespace std;

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/slam/smallExample.h>

using namespace gtsam;
using namespace example;

double tol=1e-5;

Key kx(size_t i) { return Symbol('x',i); }
Key kl(size_t i) { return Symbol('l',i); }

/* ************************************************************************* */
TEST( GaussianFactorGraph, equals ) {

  Ordering ordering; ordering += kx(1),kx(2),kl(1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
  GaussianFactorGraph fg2 = createGaussianFactorGraph(ordering);
  EXPECT(fg.equals(fg2));
}

/* ************************************************************************* */
//TEST( GaussianFactorGraph, error ) {
//  Ordering ordering; ordering += kx(1),kx(2),kl(1);
//  FactorGraph<JacobianFactor> fg = createGaussianFactorGraph(ordering);
//  VectorValues cfg = createZeroDelta(ordering);
//
//  // note the error is the same as in testNonlinearFactorGraph as a
//  // zero delta config in the linear graph is equivalent to noisy in
//  // non-linear, which is really linear under the hood
//  double actual = fg.error(cfg);
//  DOUBLES_EQUAL( 5.625, actual, 1e-9 );
//}

/* ************************************************************************* */
/* unit test for find seperator                                              */
/* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, find_separator )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//
//  set<Symbol> separator = fg.find_separator(kx(2));
//  set<Symbol> expected;
//  expected.insert(kx(1));
//  expected.insert(kl(1));
//
//  EXPECT(separator.size()==expected.size());
//  set<Symbol>::iterator it1 = separator.begin(), it2 = expected.begin();
//  for(; it1!=separator.end(); it1++, it2++)
//    EXPECT(*it1 == *it2);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, combine_factors_x1 )
//{
//  // create a small example for a linear factor graph
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//
//  // combine all factors
//  GaussianFactor::shared_ptr actual = removeAndCombineFactors(fg,kx(1));
//
//  // the expected linear factor
//  Matrix Al1 = Matrix_(6,2,
//			 0., 0.,
//			 0., 0.,
//			 0., 0.,
//			 0., 0.,
//			 5., 0.,
//			 0., 5.
//			 );
//
//  Matrix Ax1 = Matrix_(6,2,
//			 10., 0.,
//			 0., 10.,
//			-10., 0.,
//			 0.,-10.,
//			-5., 0.,
//			 0.,-5.
//			 );
//
//  Matrix Ax2 = Matrix_(6,2,
//			 0., 0.,
//			 0., 0.,
//			 10., 0.,
//			 0., 10.,
//			 0., 0.,
//			 0., 0.
//			 );
//
//  // the expected RHS vector
//  Vector b(6);
//  b(0) = -1;
//  b(1) = -1;
//  b(2) =  2;
//  b(3) = -1;
//  b(4) =  0;
//  b(5) =  1;
//
//  vector<pair<Symbol, Matrix> > meas;
//  meas.push_back(make_pair(kl(1), Al1));
//  meas.push_back(make_pair(kx(1), Ax1));
//  meas.push_back(make_pair(kx(2), Ax2));
//  GaussianFactor expected(meas, b, ones(6));
//  //GaussianFactor expected(kl(1), Al1, kx(1), Ax1, kx(2), Ax2, b);
//
//  // check if the two factors are the same
//  EXPECT(assert_equal(expected,*actual));
//}
//
///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, combine_factors_x2 )
//{
// // create a small example for a linear factor graph
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//
//  // combine all factors
//  GaussianFactor::shared_ptr actual = removeAndCombineFactors(fg,kx(2));
//
//  // the expected linear factor
//  Matrix Al1 = Matrix_(4,2,
//			 // l1
//			 0., 0.,
//			 0., 0.,
//			 5., 0.,
//			 0., 5.
//			 );
//
//  Matrix Ax1 = Matrix_(4,2,
//                         // x1
//			-10., 0., // f2
//			 0.,-10., // f2
//			 0., 0., // f4
//			 0., 0.  // f4
//			 );
//
//  Matrix Ax2 = Matrix_(4,2,
//			 // x2
//			 10., 0.,
//			 0., 10.,
//			-5., 0.,
//			 0.,-5.
//			 );
//
//  // the expected RHS vector
//  Vector b(4);
//  b(0) =  2;
//  b(1) = -1;
//  b(2) = -1;
//  b(3) =  1.5;
//
//  vector<pair<Symbol, Matrix> > meas;
//  meas.push_back(make_pair(kl(1), Al1));
//  meas.push_back(make_pair(kx(1), Ax1));
//  meas.push_back(make_pair(kx(2), Ax2));
//  GaussianFactor expected(meas, b, ones(4));
//
//  // check if the two factors are the same
//  EXPECT(assert_equal(expected,*actual));
//}

///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateOne_x1 )
//{
//  Ordering ordering; ordering += kx(1),kl(1),kx(2);
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//  GaussianConditional::shared_ptr actual = GaussianSequentialSolver::EliminateUntil(fg, 1);
//
//  // create expected Conditional Gaussian
//  Matrix I = 15*eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
//  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = ones(2);
//  GaussianConditional expected(ordering[kx(1)],15*d,R11,ordering[kl(1)],S12,ordering[kx(2)],S13,sigma);
//
//  EXPECT(assert_equal(expected,*actual,tol));
//}
//
///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateOne_x2 )
//{
//   Ordering ordering; ordering += kx(2),kl(1),kx(1);
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//  GaussianConditional::shared_ptr actual = GaussianSequentialSolver::EliminateUntil(fg, 1);
//
//  // create expected Conditional Gaussian
//  double sig = 0.0894427;
//  Matrix I = eye(2)/sig, R11 = I, S12 = -0.2*I, S13 = -0.8*I;
//  Vector d = Vector_(2, 0.2, -0.14)/sig, sigma = ones(2);
//  GaussianConditional expected(ordering[kx(2)],d,R11,ordering[kl(1)],S12,ordering[kx(1)],S13,sigma);
//
//  EXPECT(assert_equal(expected,*actual,tol));
//}
//
///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateOne_l1 )
//{
//  Ordering ordering; ordering += kl(1),kx(1),kx(2);
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//  GaussianConditional::shared_ptr actual = GaussianSequentialSolver::EliminateUntil(fg, 1);
//
//  // create expected Conditional Gaussian
//  double sig = sqrt(2)/10.;
//  Matrix I = eye(2)/sig, R11 = I, S12 = -0.5*I, S13 = -0.5*I;
//  Vector d = Vector_(2, -0.1, 0.25)/sig, sigma = ones(2);
//  GaussianConditional expected(ordering[kl(1)],d,R11,ordering[kx(1)],S12,ordering[kx(2)],S13,sigma);
//
//  EXPECT(assert_equal(expected,*actual,tol));
//}

///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateOne_x1_fast )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  GaussianConditional::shared_ptr actual = fg.eliminateOne(kx(1), false);
//
//  // create expected Conditional Gaussian
//  Matrix I = 15*eye(2), R11 = I, S12 = -0.111111*I, S13 = -0.444444*I;
//  Vector d = Vector_(2, -0.133333, -0.0222222), sigma = ones(2);
//  GaussianConditional expected(kx(1),15*d,R11,kl(1),S12,kx(2),S13,sigma);
//
//  EXPECT(assert_equal(expected,*actual,tol));
//}
//
///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateOne_x2_fast )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  GaussianConditional::shared_ptr actual = fg.eliminateOne(kx(2), false);
//
//  // create expected Conditional Gaussian
//  double sig = 0.0894427;
//  Matrix I = eye(2)/sig, R11 = I, S12 = -0.2*I, S13 = -0.8*I;
//  Vector d = Vector_(2, 0.2, -0.14)/sig, sigma = ones(2);
//  GaussianConditional expected(kx(2),d,R11,kl(1),S12,kx(1),S13,sigma);
//
//  EXPECT(assert_equal(expected,*actual,tol));
//}
//
///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateOne_l1_fast )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  GaussianConditional::shared_ptr actual = fg.eliminateOne(kl(1), false);
//
//  // create expected Conditional Gaussian
//  double sig = sqrt(2)/10.;
//  Matrix I = eye(2)/sig, R11 = I, S12 = -0.5*I, S13 = -0.5*I;
//  Vector d = Vector_(2, -0.1, 0.25)/sig, sigma = ones(2);
//  GaussianConditional expected(kl(1),d,R11,kx(1),S12,kx(2),S13,sigma);
//
//  EXPECT(assert_equal(expected,*actual,tol));
//}

/* ************************************************************************* */
TEST( GaussianFactorGraph, eliminateAll )
{
	// create expected Chordal bayes Net
	Matrix I = eye(2);

  Ordering ordering;
  ordering += kx(2),kl(1),kx(1);

	Vector d1 = Vector_(2, -0.1,-0.1);
	GaussianBayesNet expected = simpleGaussian(ordering[kx(1)],d1,0.1);

	double sig1 = 0.149071;
	Vector d2 = Vector_(2, 0.0, 0.2)/sig1, sigma2 = ones(2);
	push_front(expected,ordering[kl(1)],d2, I/sig1,ordering[kx(1)], (-1)*I/sig1,sigma2);

	double sig2 = 0.0894427;
	Vector d3 = Vector_(2, 0.2, -0.14)/sig2, sigma3 = ones(2);
	push_front(expected,ordering[kx(2)],d3, I/sig2,ordering[kl(1)], (-0.2)*I/sig2, ordering[kx(1)], (-0.8)*I/sig2, sigma3);

	// Check one ordering
	GaussianFactorGraph fg1 = createGaussianFactorGraph(ordering);
	GaussianBayesNet actual = *GaussianSequentialSolver(fg1).eliminate();
	EXPECT(assert_equal(expected,actual,tol));

  GaussianBayesNet actualQR = *GaussianSequentialSolver(fg1, true).eliminate();
  EXPECT(assert_equal(expected,actualQR,tol));
}

///* ************************************************************************* */
//TEST( GaussianFactorGraph, eliminateAll_fast )
//{
//	// create expected Chordal bayes Net
//	Matrix I = eye(2);
//
//	Vector d1 = Vector_(2, -0.1,-0.1);
//	GaussianBayesNet expected = simpleGaussian(kx(1),d1,0.1);
//
//	double sig1 = 0.149071;
//	Vector d2 = Vector_(2, 0.0, 0.2)/sig1, sigma2 = ones(2);
//	push_front(expected,kl(1),d2, I/sig1,kx(1), (-1)*I/sig1,sigma2);
//
//	double sig2 = 0.0894427;
//	Vector d3 = Vector_(2, 0.2, -0.14)/sig2, sigma3 = ones(2);
//	push_front(expected,kx(2),d3, I/sig2,kl(1), (-0.2)*I/sig2, kx(1), (-0.8)*I/sig2, sigma3);
//
//	// Check one ordering
//	GaussianFactorGraph fg1 = createGaussianFactorGraph();
//	Ordering ordering;
//	ordering += kx(2),kl(1),kx(1);
//	GaussianBayesNet actual = fg1.eliminate(ordering, false);
//	EXPECT(assert_equal(expected,actual,tol));
//}

///* ************************************************************************* */
//TEST( GaussianFactorGraph, add_priors )
//{
//  Ordering ordering; ordering += kl(1),kx(1),kx(2);
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//  GaussianFactorGraph actual = fg.add_priors(3, vector<size_t>(3,2));
//  GaussianFactorGraph expected = createGaussianFactorGraph(ordering);
//  Matrix A = eye(2);
//  Vector b = zero(2);
//  SharedDiagonal sigma = sharedSigma(2,3.0);
//  expected.push_back(GaussianFactor::shared_ptr(new JacobianFactor(ordering[kl(1)],A,b,sigma)));
//  expected.push_back(GaussianFactor::shared_ptr(new JacobianFactor(ordering[kx(1)],A,b,sigma)));
//  expected.push_back(GaussianFactor::shared_ptr(new JacobianFactor(ordering[kx(2)],A,b,sigma)));
//  EXPECT(assert_equal(expected,actual));
//}

/* ************************************************************************* */
TEST( GaussianFactorGraph, copying )
{
  // Create a graph
  Ordering ordering; ordering += kx(2),kl(1),kx(1);
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

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, matrix )
//{
//  // render with a given ordering
//  Ordering ord;
//  ord += kx(2),kl(1),kx(1);
//
//  // Create a graph
//  GaussianFactorGraph fg = createGaussianFactorGraph(ordering);
//
//  Matrix A; Vector b;
//  boost::tie(A,b) = fg.matrix();
//
//  Matrix A1 = Matrix_(2*4,3*2,
//		     +0.,  0.,  0.,  0., 10.,  0., // unary factor on x1 (prior)
//		     +0.,  0.,  0.,  0.,  0., 10.,
//		     10.,  0.,  0.,  0.,-10.,  0., // binary factor on x2,x1 (odometry)
//		     +0., 10.,  0.,  0.,  0.,-10.,
//		     +0.,  0.,  5.,  0., -5.,  0., // binary factor on l1,x1 (z1)
//		     +0.,  0.,  0.,  5.,  0., -5.,
//		     -5.,  0.,  5.,  0.,  0.,  0., // binary factor on x2,l1 (z2)
//		     +0., -5.,  0.,  5.,  0.,  0.
//    );
//  Vector b1 = Vector_(8,-1., -1., 2., -1., 0., 1., -1., 1.5);
//
//  EQUALITY(A,A1);
//  EXPECT(b==b1);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, sizeOfA )
//{
//	// create a small linear factor graph
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//
//  pair<size_t, size_t> mn = fg.sizeOfA();
//  EXPECT(8 == mn.first);
//  EXPECT(6 == mn.second);
//}

/* ************************************************************************* */
TEST( GaussianFactorGraph, CONSTRUCTOR_GaussianBayesNet )
{
  Ordering ord;
  ord += kx(2),kl(1),kx(1);
  GaussianFactorGraph fg = createGaussianFactorGraph(ord);

  // render with a given ordering
  GaussianBayesNet CBN = *GaussianSequentialSolver(fg).eliminate();

  // True GaussianFactorGraph
  GaussianFactorGraph fg2(CBN);
  GaussianBayesNet CBN2 = *GaussianSequentialSolver(fg2).eliminate();
  EXPECT(assert_equal(CBN,CBN2));

  // Base FactorGraph only
//  FactorGraph<GaussianFactor> fg3(CBN);
//  GaussianBayesNet CBN3 = gtsam::eliminate<GaussianFactor,GaussianConditional>(fg3,ord);
//  EXPECT(assert_equal(CBN,CBN3));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, getOrdering)
{
  Ordering original; original += kl(1),kx(1),kx(2);
  FactorGraph<IndexFactor> symbolic(createGaussianFactorGraph(original));
  Permutation perm(*Inference::PermutationCOLAMD(VariableIndex(symbolic)));
  Ordering actual = original; actual.permuteWithInverse((*perm.inverse()));
  Ordering expected; expected += kl(1),kx(2),kx(1);
  EXPECT(assert_equal(expected,actual));
}

// SL-FIX TEST( GaussianFactorGraph, getOrdering2)
//{
//  Ordering expected;
//  expected += kl(1),kx(1);
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  set<Symbol> interested; interested += kl(1),kx(1);
//  Ordering actual = fg.getOrdering(interested);
//  EXPECT(assert_equal(expected,actual));
//}

/* ************************************************************************* */
TEST( GaussianFactorGraph, optimize_LDL )
{
  // create an ordering
  Ordering ord; ord += kx(2),kl(1),kx(1);

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
  Ordering ord; ord += kx(2),kl(1),kx(1);

  // create a graph
	GaussianFactorGraph fg = createGaussianFactorGraph(ord);

	// optimize the graph
	VectorValues actual = *GaussianSequentialSolver(fg, true).optimize();

	// verify
	VectorValues expected = createCorrectDelta(ord);

  EXPECT(assert_equal(expected,actual));
}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, optimizeMultiFrontlas )
//{
//  // create an ordering
//  Ordering ord; ord += kx(2),kl(1),kx(1);
//
//	// create a graph
//	GaussianFactorGraph fg = createGaussianFactorGraph(ord);
//
//	// optimize the graph
//	VectorValues actual = fg.optimizeMultiFrontals(ord);
//
//	// verify
//	VectorValues expected = createCorrectDelta();
//
//  EXPECT(assert_equal(expected,actual));
//}

/* ************************************************************************* */
TEST( GaussianFactorGraph, combine)
{
  // create an ordering
  Ordering ord; ord += kx(2),kl(1),kx(1);

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
  Ordering ord; ord += kx(2),kl(1),kx(1);

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

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, factor_lookup)
//{
//	// create a test graph
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//
//	// ask for all factor indices connected to x1
//	list<size_t> x1_factors = fg.factors(kx(1));
//	size_t x1_indices[] = { 0, 1, 2 };
//	list<size_t> x1_expected(x1_indices, x1_indices + 3);
//	EXPECT(x1_factors==x1_expected);
//
//	// ask for all factor indices connected to x2
//	list<size_t> x2_factors = fg.factors(kx(2));
//	size_t x2_indices[] = { 1, 3 };
//	list<size_t> x2_expected(x2_indices, x2_indices + 2);
//	EXPECT(x2_factors==x2_expected);
//}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, findAndRemoveFactors )
//{
//	// create the graph
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//
//  // We expect to remove these three factors: 0, 1, 2
//  GaussianFactor::shared_ptr f0 = fg[0];
//  GaussianFactor::shared_ptr f1 = fg[1];
//  GaussianFactor::shared_ptr f2 = fg[2];
//
//  // call the function
//  vector<GaussianFactor::shared_ptr> factors = fg.findAndRemoveFactors(kx(1));
//
//  // Check the factors
//  EXPECT(f0==factors[0]);
//  EXPECT(f1==factors[1]);
//  EXPECT(f2==factors[2]);
//
//  // EXPECT if the factors are deleted from the factor graph
//  LONGS_EQUAL(1,fg.nrFactors());
//}

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother)
{
	GaussianFactorGraph fg1 = createSmoother(2).first;
	LONGS_EQUAL(3,fg1.size());
	GaussianFactorGraph fg2 = createSmoother(3).first;
	LONGS_EQUAL(5,fg2.size());
}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, variables )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  Dimensions expected;
//  insert(expected)(kl(1), 2)(kx(1), 2)(kx(2), 2);
//  Dimensions actual = fg.dimensions();
//  EXPECT(expected==actual);
//}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, keys )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  Ordering expected;
//  expected += kl(1),kx(1),kx(2);
//  EXPECT(assert_equal(expected,fg.keys()));
//}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, involves )
//{
//  GaussianFactorGraph fg = createGaussianFactorGraph();
//  EXPECT(fg.involves(kl(1)));
//  EXPECT(fg.involves(kx(1)));
//  EXPECT(fg.involves(kx(2)));
//  EXPECT(!fg.involves(kx(3)));
//}

/* ************************************************************************* */
double error(const VectorValues& x) {
  // create an ordering
  Ordering ord; ord += kx(2),kl(1),kx(1);

	GaussianFactorGraph fg = createGaussianFactorGraph(ord);
	return fg.error(x);
}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, gradient )
//{
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//
//	// Construct expected gradient
//	VectorValues expected;
//
//  // 2*f(x) = 100*(x1+c[kx(1)])^2 + 100*(x2-x1-[0.2;-0.1])^2 + 25*(l1-x1-[0.0;0.2])^2 + 25*(l1-x2-[-0.2;0.3])^2
//	// worked out: df/dx1 = 100*[0.1;0.1] + 100*[0.2;-0.1]) + 25*[0.0;0.2] = [10+20;10-10+5] = [30;5]
//  expected.insert(kl(1),Vector_(2,  5.0,-12.5));
//  expected.insert(kx(1),Vector_(2, 30.0,  5.0));
//  expected.insert(kx(2),Vector_(2,-25.0, 17.5));
//
//	// Check the gradient at delta=0
//  VectorValues zero = createZeroDelta();
//	VectorValues actual = fg.gradient(zero);
//	EXPECT(assert_equal(expected,actual));
//
//	// Check it numerically for good measure
//	Vector numerical_g = numericalGradient<VectorValues>(error,zero,0.001);
//	EXPECT(assert_equal(Vector_(6,5.0,-12.5,30.0,5.0,-25.0,17.5),numerical_g));
//
//	// Check the gradient at the solution (should be zero)
//	Ordering ord;
//  ord += kx(2),kl(1),kx(1);
//	GaussianFactorGraph fg2 = createGaussianFactorGraph();
//  VectorValues solution = fg2.optimize(ord); // destructive
//	VectorValues actual2 = fg.gradient(solution);
//	EXPECT(assert_equal(zero,actual2));
//}

/* ************************************************************************* */
TEST( GaussianFactorGraph, multiplication )
{
  // create an ordering
  Ordering ord; ord += kx(2),kl(1),kx(1);

	FactorGraph<JacobianFactor> A = createGaussianFactorGraph(ord);
  VectorValues x = createCorrectDelta(ord);
  Errors actual = A * x;
  Errors expected;
  expected += Vector_(2,-1.0,-1.0);
  expected += Vector_(2, 2.0,-1.0);
  expected += Vector_(2, 0.0, 1.0);
  expected += Vector_(2,-1.0, 1.5);
	EXPECT(assert_equal(expected,actual));
}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, transposeMultiplication )
//{
//  // create an ordering
//  Ordering ord; ord += kx(2),kl(1),kx(1);
//
//	GaussianFactorGraph A = createGaussianFactorGraph(ord);
//  Errors e;
//  e += Vector_(2, 0.0, 0.0);
//  e += Vector_(2,15.0, 0.0);
//  e += Vector_(2, 0.0,-5.0);
//  e += Vector_(2,-7.5,-5.0);
//
//  VectorValues expected = createZeroDelta(ord), actual = A ^ e;
//  expected[ord[kl(1)]] = Vector_(2, -37.5,-50.0);
//  expected[ord[kx(1)]] = Vector_(2,-150.0, 25.0);
//  expected[ord[kx(2)]] = Vector_(2, 187.5, 25.0);
//	EXPECT(assert_equal(expected,actual));
//}

///* ************************************************************************* */
// SL-NEEDED? TEST( GaussianFactorGraph, rhs )
//{
//  // create an ordering
//  Ordering ord; ord += kx(2),kl(1),kx(1);
//
//	GaussianFactorGraph Ab = createGaussianFactorGraph(ord);
//	Errors expected = createZeroDelta(ord), actual = Ab.rhs();
//  expected.push_back(Vector_(2,-1.0,-1.0));
//  expected.push_back(Vector_(2, 2.0,-1.0));
//  expected.push_back(Vector_(2, 0.0, 1.0));
//  expected.push_back(Vector_(2,-1.0, 1.5));
//	EXPECT(assert_equal(expected,actual));
//}

/* ************************************************************************* */
// Extra test on elimination prompted by Michael's email to Frank 1/4/2010
TEST( GaussianFactorGraph, elimination )
{
  Ordering ord;
  ord += kx(1), kx(2);
	// Create Gaussian Factor Graph
	GaussianFactorGraph fg;
	Matrix Ap = eye(1), An = eye(1) * -1;
	Vector b = Vector_(1, 0.0);
  SharedDiagonal sigma = sharedSigma(1,2.0);
	fg.add(ord[kx(1)], An, ord[kx(2)], Ap, b, sigma);
	fg.add(ord[kx(1)], Ap, b, sigma);
	fg.add(ord[kx(2)], Ap, b, sigma);

	// Eliminate
	GaussianBayesNet bayesNet = *GaussianSequentialSolver(fg).eliminate();

	// Check sigma
	EXPECT_DOUBLES_EQUAL(1.0,bayesNet[ord[kx(2)]]->get_sigmas()(0),1e-5);

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

///* ************************************************************************* */
//SL-FIX TEST( GaussianFactorGraph, constrained_single2 )
//{
//	// get a graph with a constraint in it
//	GaussianFactorGraph fg = createSingleConstraintGraph();
//
//	// eliminate and solve
//	Ordering ord;
//	ord += "yk, x";
//	VectorValues actual = fg.optimize(ord);
//
//	// verify
//	VectorValues expected = createSingleConstraintValues();
//	EXPECT(assert_equal(expected, actual));
//}

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

///* ************************************************************************* */
//SL-FIX TEST( GaussianFactorGraph, constrained_multi2 )
//{
//	// get a graph with a constraint in it
//	GaussianFactorGraph fg = createMultiConstraintGraph();
//
//	// eliminate and solve
//	Ordering ord;
//	ord += "zk, xk, y";
//	VectorValues actual = fg.optimize(ord);
//
//	// verify
//	VectorValues expected = createMultiConstraintValues();
//	EXPECT(assert_equal(expected, actual));
//}

/* ************************************************************************* */

SharedDiagonal model = sharedSigma(2,1);

// SL-FIX TEST( GaussianFactorGraph, findMinimumSpanningTree )
//{
//	GaussianFactorGraph g;
//	Matrix I = eye(2);
//	Vector b = Vector_(0, 0, 0);
//	g.add(kx(1), I, kx(2), I, b, model);
//	g.add(kx(1), I, kx(3), I, b, model);
//	g.add(kx(1), I, kx(4), I, b, model);
//	g.add(kx(2), I, kx(3), I, b, model);
//	g.add(kx(2), I, kx(4), I, b, model);
//	g.add(kx(3), I, kx(4), I, b, model);
//
//	map<string, string> tree = g.findMinimumSpanningTree<string, GaussianFactor>();
//	EXPECT(tree[kx(1)].compare(kx(1))==0);
//	EXPECT(tree[kx(2)].compare(kx(1))==0);
//	EXPECT(tree[kx(3)].compare(kx(1))==0);
//	EXPECT(tree[kx(4)].compare(kx(1))==0);
//}

///* ************************************************************************* */
// SL-FIX TEST( GaussianFactorGraph, split )
//{
//	GaussianFactorGraph g;
//	Matrix I = eye(2);
//	Vector b = Vector_(0, 0, 0);
//	g.add(kx(1), I, kx(2), I, b, model);
//	g.add(kx(1), I, kx(3), I, b, model);
//	g.add(kx(1), I, kx(4), I, b, model);
//	g.add(kx(2), I, kx(3), I, b, model);
//	g.add(kx(2), I, kx(4), I, b, model);
//
//	PredecessorMap<string> tree;
//	tree[kx(1)] = kx(1);
//	tree[kx(2)] = kx(1);
//	tree[kx(3)] = kx(1);
//	tree[kx(4)] = kx(1);
//
//	GaussianFactorGraph Ab1, Ab2;
//  g.split<string, GaussianFactor>(tree, Ab1, Ab2);
//	LONGS_EQUAL(3, Ab1.size());
//	LONGS_EQUAL(2, Ab2.size());
//}

/* ************************************************************************* */
TEST(GaussianFactorGraph, replace)
{
  Ordering ord; ord += kx(1),kx(2),kx(3),kx(4),kx(5),kx(6);
	SharedDiagonal noise(sharedSigma(3, 1.0));

	GaussianFactorGraph::sharedFactor f1(new JacobianFactor(
	    ord[kx(1)], eye(3,3), ord[kx(2)], eye(3,3), zero(3), noise));
	GaussianFactorGraph::sharedFactor f2(new JacobianFactor(
	    ord[kx(2)], eye(3,3), ord[kx(3)], eye(3,3), zero(3), noise));
	GaussianFactorGraph::sharedFactor f3(new JacobianFactor(
	    ord[kx(3)], eye(3,3), ord[kx(4)], eye(3,3), zero(3), noise));
	GaussianFactorGraph::sharedFactor f4(new JacobianFactor(
	    ord[kx(5)], eye(3,3), ord[kx(6)], eye(3,3), zero(3), noise));

	GaussianFactorGraph actual;
	actual.push_back(f1);
//	actual.checkGraphConsistency();
	actual.push_back(f2);
//	actual.checkGraphConsistency();
	actual.push_back(f3);
//	actual.checkGraphConsistency();
	actual.replace(0, f4);
//	actual.checkGraphConsistency();

	GaussianFactorGraph expected;
	expected.push_back(f4);
//	actual.checkGraphConsistency();
	expected.push_back(f2);
//	actual.checkGraphConsistency();
	expected.push_back(f3);
//	actual.checkGraphConsistency();

	EXPECT(assert_equal(expected, actual));
}

///* ************************************************************************* */
//TEST ( GaussianFactorGraph, combine_matrix ) {
//	// create a small linear factor graph
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//	Dimensions dimensions = fg.dimensions();
//
//	// get two factors from it and insert the factors into a vector
//	vector<GaussianFactor::shared_ptr> lfg;
//	lfg.push_back(fg[4 - 1]);
//	lfg.push_back(fg[2 - 1]);
//
//	// combine in a factor
//	Matrix Ab; SharedDiagonal noise;
//	Ordering order; order += kx(2), kl(1), kx(1);
//	boost::tie(Ab, noise) = combineFactorsAndCreateMatrix(lfg, order, dimensions);
//
//	// the expected augmented matrix
//	Matrix expAb = Matrix_(4, 7,
//			-5.,  0., 5., 0.,  0.,  0.,-1.0,
//			+0., -5., 0., 5.,  0.,  0., 1.5,
//			10.,  0., 0., 0.,-10.,  0., 2.0,
//			+0., 10., 0., 0.,  0.,-10.,-1.0);
//
//	// expected noise model
//	SharedDiagonal expModel = noiseModel::Unit::Create(4);
//
//	EXPECT(assert_equal(expAb, Ab));
//	EXPECT(assert_equal(*expModel, *noise));
//}

/* ************************************************************************* */
/*
 *   x2 x1 x3 b
 *    1  1    1       1  1  0  1
 *    1    1  1  ->      1  1  1
 *         1  1             1  1
 */
// SL-NEEDED? TEST ( GaussianFactorGraph, eliminateFrontals ) {
//	typedef GaussianFactorGraph::sharedFactor Factor;
//	SharedDiagonal model(Vector_(1, 0.5));
//	GaussianFactorGraph fg;
//	Factor factor1(new JacobianFactor(kx(1), Matrix_(1,1,1.), kx(2), Matrix_(1,1,1.), Vector_(1,1.),  model));
//	Factor factor2(new JacobianFactor(kx(2), Matrix_(1,1,1.), kx(3), Matrix_(1,1,1.), Vector_(1,1.),  model));
//	Factor factor3(new JacobianFactor(kx(3), Matrix_(1,1,1.), kx(3), Matrix_(1,1,1.), Vector_(1,1.),  model));
//	fg.push_back(factor1);
//	fg.push_back(factor2);
//	fg.push_back(factor3);
//
//	Ordering frontals; frontals += kx(2), kx(1);
//	GaussianBayesNet bn = fg.eliminateFrontals(frontals);
//
//	GaussianBayesNet bn_expected;
//	GaussianBayesNet::sharedConditional conditional1(new GaussianConditional(kx(2), Vector_(1, 2.), Matrix_(1, 1, 2.),
//			kx(1), Matrix_(1, 1, 1.), kx(3), Matrix_(1, 1, 1.), Vector_(1, 1.)));
//	GaussianBayesNet::sharedConditional conditional2(new GaussianConditional(kx(1), Vector_(1, 0.), Matrix_(1, 1, -1.),
//			kx(3), Matrix_(1, 1, 1.), Vector_(1, 1.)));
//	bn_expected.push_back(conditional1);
//	bn_expected.push_back(conditional2);
//	EXPECT(assert_equal(bn_expected, bn));
//
//	GaussianFactorGraph::sharedFactor factor_expected(new JacobianFactor(kx(3), Matrix_(1, 1, 2.), Vector_(1, 2.), SharedDiagonal(Vector_(1, 1.))));
//	GaussianFactorGraph fg_expected;
//	fg_expected.push_back(factor_expected);
//	EXPECT(assert_equal(fg_expected, fg));
//}

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother2)
{
  using namespace example;
  GaussianFactorGraph fg2;
  Ordering ordering;
  boost::tie(fg2,ordering) = createSmoother(3);
  LONGS_EQUAL(5,fg2.size());

  // eliminate
  vector<Index> x3var; x3var.push_back(ordering[kx(3)]);
  vector<Index> x1var; x1var.push_back(ordering[kx(1)]);
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

	Ordering ordering; ordering += kx(1), kx(2), kl(1);
	FactorGraph<GaussianFactor> fg = createGaussianFactorGraph(ordering);
	EXPECT(!hasConstraints(fg));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
