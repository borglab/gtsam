/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRegularHessianFactor.cpp
 * @author  Frank Dellaert
 * @date    March 4, 2014
 */

#include <gtsam/linear/RegularHessianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(RegularHessianFactor, Constructors)
{
  // First construct a regular JacobianFactor
  // 0.5*|x0 + x1 + x3 - [1;2]|^2 = 0.5*|A*x-b|^2, with A=[I I I]
  Matrix A1 = I_2x2, A2 = I_2x2, A3 = I_2x2;
  Vector2 b(1,2);
  const vector<pair<Key, Matrix> > terms {{0, A1}, {1, A2}, {3, A3}};
  RegularJacobianFactor<2> jf(terms, b);

  // Test conversion from JacobianFactor
  RegularHessianFactor<2> factor(jf);

  // 0.5*|A*x-b|^2 = 0.5*(Ax-b)'*(Ax-b) = 0.5*x'*A'A*x - x'*A'b + 0.5*b'*b
  // Compare with comment in HessianFactor: E(x) = 0.5 x^T G x - x^T g + 0.5 f
  // Hence G = I6, g A'*b = [b;b;b], and f = b'*b = 1+4 = 5
  Matrix G11 = I_2x2;
  Matrix G12 = I_2x2;
  Matrix G13 = I_2x2;

  Matrix G22 = I_2x2;
  Matrix G23 = I_2x2;

  Matrix G33 = I_2x2;

  Vector2 g1 = b, g2 = b, g3 = b;

  double f = 5;

  // Test ternary constructor
  RegularHessianFactor<2> factor2(0, 1, 3, G11, G12, G13, g1, G22, G23, g2, G33, g3, f);
  EXPECT(assert_equal(factor,factor2));

  // Test n-way constructor
  KeyVector keys {0, 1, 3};
  vector<Matrix> Gs {G11, G12, G13, G22, G23, G33};
  vector<Vector> gs {g1, g2, g3};
  RegularHessianFactor<2> factor3(keys, Gs, gs, f);
  EXPECT(assert_equal(factor, factor3));

  // Test constructor from Gaussian Factor Graph
  GaussianFactorGraph gfg;
  gfg += jf;
  RegularHessianFactor<2> factor4(gfg);
  EXPECT(assert_equal(factor, factor4));
  GaussianFactorGraph gfg2;
  gfg2 += factor;
  RegularHessianFactor<2> factor5(gfg);
  EXPECT(assert_equal(factor, factor5));

  // Test constructor from Information matrix
  Matrix info = factor.augmentedInformation();
  vector<size_t> dims {2, 2, 2};
  SymmetricBlockMatrix sym(dims, info, true);
  RegularHessianFactor<2> factor6(keys, sym);
  EXPECT(assert_equal(factor, factor6));

  // multiplyHessianAdd:
  {
  // brute force
  Matrix AtA = factor.information();
  HessianFactor::const_iterator i1 = factor.begin();
  HessianFactor::const_iterator i2 = i1 + 1;
  Vector X(6); X << 1,2,3,4,5,6;
  Vector Y(6); Y << 9, 12, 9, 12, 9, 12;
  EXPECT(assert_equal(Y,AtA*X));

  VectorValues x{{0, Vector2(1, 2)}, {1, Vector2(3, 4)}, {3, Vector2(5, 6)}};

  VectorValues expected;
  expected.insert(0, Y.segment<2>(0));
  expected.insert(1, Y.segment<2>(2));
  expected.insert(3, Y.segment<2>(4));

  // VectorValues version
  double alpha = 1.0;
  VectorValues actualVV;
  actualVV.insert(0, Vector2::Zero());
  actualVV.insert(1, Vector2::Zero());
  actualVV.insert(3, Vector2::Zero());
  factor.multiplyHessianAdd(alpha, x, actualVV);
  EXPECT(assert_equal(expected, actualVV));

  // RAW ACCESS
  Vector expected_y(8); expected_y << 9, 12, 9, 12, 0, 0, 9, 12;
  Vector fast_y = Vector8::Zero();
  double xvalues[8] = {1,2,3,4,0,0,5,6};
  factor.multiplyHessianAdd(alpha, xvalues, fast_y.data());
  EXPECT(assert_equal(expected_y, fast_y));

  // now, do it with non-zero y
  factor.multiplyHessianAdd(alpha, xvalues, fast_y.data());
  EXPECT(assert_equal(2*expected_y, fast_y));

  // check some expressions
  EXPECT(assert_equal(G12,factor.info().aboveDiagonalBlock(i1 - factor.begin(), i2 - factor.begin())));
  EXPECT(assert_equal(G22,factor.info().diagonalBlock(i2 - factor.begin())));
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
