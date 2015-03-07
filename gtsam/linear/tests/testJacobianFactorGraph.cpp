/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testJacobianFactorGraph.cpp
 *  @brief  Unit tests for GaussianFactorGraph
 *  @author Yong Dian Jian
 **/

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>


///* ************************************************************************* */
//TEST( GaussianFactorGraph, gradient )
//{
//	GaussianFactorGraph fg = createGaussianFactorGraph();
//
//	// Construct expected gradient
//	VectorValues expected;
//
//  // 2*f(x) = 100*(x1+c[X(1)])^2 + 100*(x2-x1-[0.2;-0.1])^2 + 25*(l1-x1-[0.0;0.2])^2 + 25*(l1-x2-[-0.2;0.3])^2
//	// worked out: df/dx1 = 100*[0.1;0.1] + 100*[0.2;-0.1]) + 25*[0.0;0.2] = [10+20;10-10+5] = [30;5]
//  expected.insert(L(1),Vector_(2,  5.0,-12.5));
//  expected.insert(X(1),Vector_(2, 30.0,  5.0));
//  expected.insert(X(2),Vector_(2,-25.0, 17.5));
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
//  ord += X(2),L(1),X(1);
//	GaussianFactorGraph fg2 = createGaussianFactorGraph();
//  VectorValues solution = fg2.optimize(ord); // destructive
//	VectorValues actual2 = fg.gradient(solution);
//	EXPECT(assert_equal(zero,actual2));
//}

///* ************************************************************************* */
//TEST( GaussianFactorGraph, transposeMultiplication )
//{
//  // create an ordering
//  Ordering ord; ord += X(2),L(1),X(1);
//
//	GaussianFactorGraph A = createGaussianFactorGraph(ord);
//  Errors e;
//  e += Vector_(2, 0.0, 0.0);
//  e += Vector_(2,15.0, 0.0);
//  e += Vector_(2, 0.0,-5.0);
//  e += Vector_(2,-7.5,-5.0);
//
//  VectorValues expected = createZeroDelta(ord), actual = A ^ e;
//  expected[ord[L(1)]] = Vector_(2, -37.5,-50.0);
//  expected[ord[X(1)]] = Vector_(2,-150.0, 25.0);
//  expected[ord[X(2)]] = Vector_(2, 187.5, 25.0);
//	EXPECT(assert_equal(expected,actual));
//}

///* ************************************************************************* */
//TEST( GaussianFactorGraph, rhs )
//{
//  // create an ordering
//  Ordering ord; ord += X(2),L(1),X(1);
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
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
