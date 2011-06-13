/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBlockMatrices
 * @author Alex Cunningham
 */

#include <iostream>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/blockMatrices.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(testBlockMatrices, jacobian_factor1) {
	typedef Matrix AbMatrix;
	typedef VerticalBlockView<AbMatrix> BlockAb;

  AbMatrix matrix; 				// actual matrix - empty to start with

  // from JacobianFactor::Constructor - one variable
  Matrix A1 = Matrix_(2,3,
  		1., 2., 3.,
  		4., 5., 6.);
  Vector b = Vector_(2, 7., 8.);
  size_t dims[] = { A1.cols(), 1};

  // build the structure
  BlockAb Ab(matrix, dims, dims+2, b.size());

  // add a matrix and get back out
  Ab(0) = A1;
  EXPECT(assert_equal(A1, Ab(0)));

  // add vector to the system
  Ab.column(1, 0) = b;
  EXPECT(assert_equal(A1, Ab(0)));
  EXPECT(assert_equal(b, Ab.column(1,0)));

  // examine matrix contents
  EXPECT_LONGS_EQUAL(2, Ab.nBlocks());
  Matrix expFull = Matrix_(2, 4,
  		1., 2., 3., 7.,
  		4., 5., 6., 8.);
  Matrix actFull = Ab.full();
  EXPECT(assert_equal(expFull, actFull));
}

/* ************************************************************************* */
TEST(testBlockMatrices, jacobian_factor2) {
	typedef Matrix AbMatrix;
	typedef VerticalBlockView<AbMatrix> BlockAb;

  AbMatrix matrix; 				// actual matrix - empty to start with

  // from JacobianFactor::Constructor - two variables
  Matrix A1 = Matrix_(2,3,
  		1., 2., 3.,
  		4., 5., 6.);
  Matrix A2 = Matrix_(2,1,
    		10.,
    		11.);
  Vector b = Vector_(2, 7., 8.);
  size_t dims[] = { A1.cols(), A2.cols(), 1};

  // build the structure
  BlockAb Ab(matrix, dims, dims+3, b.size());

  // add blocks
  Ab(0) = A1;
  Ab(1) = A2;
  EXPECT(assert_equal(A1, Ab(0)));
  EXPECT(assert_equal(A2, Ab(1)));

  // add vector to the system
  Ab.column(2, 0) = b;
  EXPECT(assert_equal(A1, Ab(0)));
  EXPECT(assert_equal(A2, Ab(1)));
  EXPECT(assert_equal(b, Ab.column(2,0)));

  // examine matrix contents
  EXPECT_LONGS_EQUAL(3, Ab.nBlocks());
  Matrix expFull = Matrix_(2, 5,
  		1., 2., 3., 10., 7.,
  		4., 5., 6., 11., 8.);
  Matrix actFull = Ab.full();
  EXPECT(assert_equal(expFull, actFull));
}

/* ************************************************************************* */
TEST(testBlockMatrices, hessian_factor1) {
  typedef Matrix InfoMatrix;
  typedef SymmetricBlockView<InfoMatrix> BlockInfo;

	Matrix expected_full = Matrix_(3, 3,
		 3.0,  5.0, -8.0,
		 0.0,  6.0, -9.0,
		 0.0,  0.0, 10.0);

  Matrix G = Matrix_(2,2, 3.0, 5.0, 0.0, 6.0);
  Vector g = Vector_(2, -8.0, -9.0);
  double f = 10.0;

	size_t dims[] = { G.rows(), 1 };
	InfoMatrix fullMatrix = zeros(G.rows() + 1, G.rows() + 1);
	BlockInfo infoView(fullMatrix, dims, dims+2);
	infoView(0,0) = G;
	infoView.column(0,1,0) = g;
	infoView(1,1)(0,0) = f;

	EXPECT_LONGS_EQUAL(0, infoView.blockStart());
	EXPECT_LONGS_EQUAL(2, infoView.nBlocks());
	EXPECT(assert_equal(InfoMatrix(expected_full), fullMatrix));
	EXPECT(assert_equal(InfoMatrix(G), infoView.range(0, 1, 0, 1)))
	EXPECT_DOUBLES_EQUAL(f, infoView(1, 1)(0,0), 1e-10);

	EXPECT(assert_equal(g, Vector(infoView.rangeColumn(0, 1, 1, 0))));
  EXPECT(assert_equal(g, Vector(((const BlockInfo)infoView).rangeColumn(0, 1, 1, 0))));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
