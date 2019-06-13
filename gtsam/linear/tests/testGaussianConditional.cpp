/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianConditional.cpp
 *  @brief  Unit tests for Conditional gaussian
 *  @author Christian Potthast
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianBayesNet.h>

#include <boost/assign/std/list.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/assign/list_inserter.hpp>
#include <boost/make_shared.hpp>
#include <boost/assign/list_of.hpp>

#include <iostream>
#include <sstream>
#include <vector>

using namespace gtsam;
using namespace std;
using namespace boost::assign;

static const double tol = 1e-5;

static Matrix R = (Matrix(2, 2) <<
    -12.1244,  -5.1962,
          0.,   4.6904).finished();

/* ************************************************************************* */
TEST(GaussianConditional, constructor)
{
  Matrix S1 = (Matrix(2, 2) <<
      -5.2786,  -8.6603,
      5.0254,   5.5432).finished();
  Matrix S2 = (Matrix(2, 2) <<
      -10.5573,  -5.9385,
      5.5737,   3.0153).finished();
  Matrix S3 = (Matrix(2, 2) <<
      -11.3820,  -7.2581,
      -3.0153,  -3.5635).finished();

  Vector d = Vector2(1.0, 2.0);
  SharedDiagonal s = noiseModel::Diagonal::Sigmas(Vector2(3.0, 4.0));

  vector<pair<Key, Matrix> > terms = pair_list_of
      (1, R)
      (3, S1)
      (5, S2)
      (7, S3);

  GaussianConditional actual(terms, 1, d, s);

  GaussianConditional::const_iterator it = actual.beginFrontals();
  EXPECT(assert_equal(Key(1), *it));
  EXPECT(assert_equal(R, actual.R()));
  ++ it;
  EXPECT(it == actual.endFrontals());

  it = actual.beginParents();
  EXPECT(assert_equal(Key(3), *it));
  EXPECT(assert_equal(S1, actual.S(it)));

  ++ it;
  EXPECT(assert_equal(Key(5), *it));
  EXPECT(assert_equal(S2, actual.S(it)));

  ++ it;
  EXPECT(assert_equal(Key(7), *it));
  EXPECT(assert_equal(S3, actual.S(it)));

  ++it;
  EXPECT(it == actual.endParents());

  EXPECT(assert_equal(d, actual.d()));
  EXPECT(assert_equal(*s, *actual.get_model()));

  // test copy constructor
  GaussianConditional copied(actual);
  EXPECT(assert_equal(d, copied.d()));
  EXPECT(assert_equal(*s, *copied.get_model()));
  EXPECT(assert_equal(R, copied.R()));
}

/* ************************************************************************* */
TEST( GaussianConditional, equals )
{
  // create a conditional gaussian node
  Matrix A1(2,2);
  A1(0,0) = 1 ; A1(1,0) = 2;
  A1(0,1) = 3 ; A1(1,1) = 4;

  Matrix A2(2,2);
  A2(0,0) = 6 ; A2(1,0) = 0.2;
  A2(0,1) = 8 ; A2(1,1) = 0.4;

  Matrix R(2,2);
  R(0,0) = 0.1 ; R(1,0) = 0.3;
  R(0,1) = 0.0 ; R(1,1) = 0.34;

  SharedDiagonal model = noiseModel::Diagonal::Sigmas(Vector2(1.0, 0.34));

  Vector d = Vector2(0.2, 0.5);

  GaussianConditional
    expected(1, d, R, 2, A1, 10, A2, model),
    actual(1, d, R, 2, A1, 10, A2, model);

  EXPECT( expected.equals(actual) );
}

/* ************************************************************************* */
TEST( GaussianConditional, solve )
{
  //expected solution
  Vector expectedX(2);
  expectedX(0) = 20-3-11 ; expectedX(1) = 40-7-15;

  // create a conditional Gaussian node
  Matrix R = (Matrix(2, 2) <<  1., 0.,
                            0., 1.).finished();

  Matrix A1 = (Matrix(2, 2) << 1., 2.,
                            3., 4.).finished();

  Matrix A2 = (Matrix(2, 2) << 5., 6.,
                            7., 8.).finished();

  Vector d(2); d << 20.0, 40.0;

  GaussianConditional cg(1, d, R, 2, A1, 10, A2);

  Vector sx1(2); sx1 << 1.0, 1.0;
  Vector sl1(2); sl1 << 1.0, 1.0;

  VectorValues expected = map_list_of
    (1, expectedX)
    (2, sx1)
    (10, sl1);

  VectorValues solution = map_list_of
    (2, sx1) // parents
    (10, sl1);
  solution.insert(cg.solve(solution));

  EXPECT(assert_equal(expected, solution, tol));
}

/* ************************************************************************* */
TEST( GaussianConditional, solve_simple )
{
  // 2 variables, frontal has dim=4
  VerticalBlockMatrix blockMatrix(list_of(4)(2)(1), 4);
  blockMatrix.matrix() <<
      1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.1,
      0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.2,
      0.0, 0.0, 3.0, 0.0, 4.0, 0.0, 0.3,
      0.0, 0.0, 0.0, 3.0, 0.0, 4.0, 0.4;

  // solve system as a non-multifrontal version first
  GaussianConditional cg(list_of(1)(2), 1, blockMatrix);

  // partial solution
  Vector sx1 = Vector2(9.0, 10.0);

  // elimination order: 1, 2
  VectorValues actual = map_list_of
    (2, sx1); // parent

  VectorValues expected = map_list_of<Key, Vector>
    (2, sx1)
    (1, (Vector(4) << -3.1,-3.4,-11.9,-13.2).finished());

  // verify indices/size
  EXPECT_LONGS_EQUAL(2, (long)cg.size());
  EXPECT_LONGS_EQUAL(4, (long)cg.rows());

  // solve and verify
  actual.insert(cg.solve(actual));
  EXPECT(assert_equal(expected, actual, tol));
}

/* ************************************************************************* */
TEST( GaussianConditional, solve_multifrontal )
{
  // create full system, 3 variables, 2 frontals, all 2 dim
  VerticalBlockMatrix blockMatrix(list_of(2)(2)(2)(1), 4);
  blockMatrix.matrix() <<
      1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.1,
      0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.2,
      0.0, 0.0, 3.0, 0.0, 4.0, 0.0, 0.3,
      0.0, 0.0, 0.0, 3.0, 0.0, 4.0, 0.4;

  // 3 variables, all dim=2
  GaussianConditional cg(list_of(1)(2)(10), 2, blockMatrix);

  EXPECT(assert_equal(Vector(blockMatrix.full().rightCols(1)), cg.d()));

  // partial solution
  Vector sl1 = Vector2(9.0, 10.0);

  // elimination order; _x_, _x1_, _l1_
  VectorValues actual = map_list_of
    (10, sl1); // parent

  VectorValues expected = map_list_of<Key, Vector>
    (1, Vector2(-3.1,-3.4))
    (2, Vector2(-11.9,-13.2))
    (10, sl1);

  // verify indices/size
  EXPECT_LONGS_EQUAL(3, (long)cg.size());
  EXPECT_LONGS_EQUAL(4, (long)cg.rows());

  // solve and verify
  actual.insert(cg.solve(actual));
  EXPECT(assert_equal(expected, actual, tol));

}

/* ************************************************************************* */
TEST( GaussianConditional, solveTranspose ) {
  /** create small Chordal Bayes Net x <- y
   * x y d
   * 1 1 9
   *   1 5
   */
  Matrix R11 = (Matrix(1, 1) << 1.0).finished(), S12 = (Matrix(1, 1) << 1.0).finished();
  Matrix R22 = (Matrix(1, 1) << 1.0).finished();
  Vector d1(1), d2(1);
  d1(0) = 9;
  d2(0) = 5;

  // define nodes and specify in reverse topological sort (i.e. parents last)
  GaussianBayesNet cbn = list_of
    (GaussianConditional(1, d1, R11, 2, S12))
    (GaussianConditional(1, d2, R22));

  // x=R'*y, y=inv(R')*x
  // 2 = 1    2
  // 5   1 1  3

  VectorValues
    x = map_list_of<Key, Vector>
      (1, (Vector(1) << 2.).finished())
      (2, (Vector(1) << 5.).finished()),
    y = map_list_of<Key, Vector>
      (1, (Vector(1) << 2.).finished())
      (2, (Vector(1) << 3.).finished());

  // test functional version
  VectorValues actual = cbn.backSubstituteTranspose(x);
  CHECK(assert_equal(y, actual));
}

/* ************************************************************************* */
TEST( GaussianConditional, information ) {

  // Create R matrix
  Matrix R(4,4); R <<
      1, 2, 3, 4,
      0, 5, 6, 7,
      0, 0, 8, 9,
      0, 0, 0, 10;

  // Create conditional
  GaussianConditional conditional(0, Vector::Zero(4), R);

  // Expected information matrix (using permuted R)
  Matrix IExpected = R.transpose() * R;

  // Actual information matrix (conditional should permute R)
  Matrix IActual = conditional.information();
  EXPECT(assert_equal(IExpected, IActual));
}

/* ************************************************************************* */
TEST( GaussianConditional, isGaussianFactor ) {

  // Create R matrix
  Matrix R(4,4); R <<
      1, 2, 3, 4,
      0, 5, 6, 7,
      0, 0, 8, 9,
      0, 0, 0, 10;

  // Create a conditional
  GaussianConditional conditional(0, Vector::Zero(4), R);

  // Expected information matrix computed by conditional
  Matrix IExpected = conditional.information();

  // Expected information matrix computed by a factor
  JacobianFactor jf = conditional;
  Matrix IActual = jf.information();

  EXPECT(assert_equal(IExpected, IActual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
