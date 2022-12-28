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
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianDensity.h>
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
using symbol_shorthand::X;
using symbol_shorthand::Y;

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

namespace density {
static const Key key = 77;
static const auto unitPrior =
                      GaussianConditional(key, Vector1::Constant(5), I_1x1),
                  widerPrior =
                      GaussianConditional(key, Vector1::Constant(5), I_1x1,
                                          noiseModel::Isotropic::Sigma(1, 3.0));
}  // namespace density

/* ************************************************************************* */
// Check that the evaluate function matches direct calculation with R.
TEST(GaussianConditional, Evaluate1) {
  // Let's evaluate at the mean
  const VectorValues mean = density::unitPrior.solve(VectorValues());

  // We get the Hessian matrix, which has noise model applied!
  const Matrix invSigma = density::unitPrior.information();

  // A Gaussian density ~ exp (-0.5*(Rx-d)'*(Rx-d))
  // which at the mean is 1.0! So, the only thing we need to calculate is
  // the normalization constant 1.0/sqrt((2*pi*Sigma).det()).
  // The covariance matrix inv(Sigma) = R'*R, so the determinant is
  const double expected = sqrt((invSigma / (2 * M_PI)).determinant());
  const double actual = density::unitPrior.evaluate(mean);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
}

// Check the evaluate with non-unit noise.
TEST(GaussianConditional, Evaluate2) {
  // See comments in test above.
  const VectorValues mean = density::widerPrior.solve(VectorValues());
  const Matrix R = density::widerPrior.R();
  const Matrix invSigma = density::widerPrior.information();
  const double expected = sqrt((invSigma / (2 * M_PI)).determinant());
  const double actual = density::widerPrior.evaluate(mean);
  EXPECT_DOUBLES_EQUAL(expected, actual, 1e-9);
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
// Test FromMeanAndStddev named constructors
TEST(GaussianConditional, FromMeanAndStddev) {
  Matrix A1 = (Matrix(2, 2) << 1., 2., 3., 4.).finished();
  Matrix A2 = (Matrix(2, 2) << 5., 6., 7., 8.).finished();
  const Vector2 b(20, 40), x0(1, 2), x1(3, 4), x2(5, 6);
  const double sigma = 3;

  VectorValues values = map_list_of(X(0), x0)(X(1), x1)(X(2), x2);

  auto conditional1 =
      GaussianConditional::FromMeanAndStddev(X(0), A1, X(1), b, sigma);
  Vector2 e1 = (x0 - (A1 * x1 + b)) / sigma;
  double expected1 = 0.5 * e1.dot(e1);
  EXPECT_DOUBLES_EQUAL(expected1, conditional1.error(values), 1e-9);

  auto conditional2 = GaussianConditional::FromMeanAndStddev(X(0), A1, X(1), A2,
                                                             X(2), b, sigma);
  Vector2 e2 = (x0 - (A1 * x1 + A2 * x2 + b)) / sigma;
  double expected2 = 0.5 * e2.dot(e2);
  EXPECT_DOUBLES_EQUAL(expected2, conditional2.error(values), 1e-9);
}

/* ************************************************************************* */
// Test likelihood method (conversion to JacobianFactor)
TEST(GaussianConditional, likelihood) {
  Matrix A1 = (Matrix(2, 2) << 1., 2., 3., 4.).finished();
  const Vector2 b(20, 40), x0(1, 2);
  const double sigma = 0.01;

  // |x0 - A1 x1 - b|^2
  auto conditional =
      GaussianConditional::FromMeanAndStddev(X(0), A1, X(1), b, sigma);

  VectorValues frontalValues;
  frontalValues.insert(X(0), x0);
  auto actual1 = conditional.likelihood(frontalValues);
  CHECK(actual1);

  // |(-A1) x1 - (b - x0)|^2
  JacobianFactor expected(X(1), -A1, b - x0,
                          noiseModel::Isotropic::Sigma(2, sigma));
  EXPECT(assert_equal(expected, *actual1, tol));

  // Check single vector version
  auto actual2 = conditional.likelihood(x0);
  CHECK(actual2);
  EXPECT(assert_equal(expected, *actual2, tol));
}

/* ************************************************************************* */
// Test sampling
TEST(GaussianConditional, sample) {
  Matrix A1 = (Matrix(2, 2) << 1., 2., 3., 4.).finished();
  const Vector2 b(20, 40), x1(3, 4);
  const double sigma = 0.01;

  auto density = GaussianDensity::FromMeanAndStddev(X(0), b, sigma);
  auto actual1 = density.sample();
  EXPECT_LONGS_EQUAL(1, actual1.size());
  EXPECT(assert_equal(b, actual1[X(0)], 50 * sigma));

  VectorValues given;
  given.insert(X(1), x1);

  auto conditional =
      GaussianConditional::FromMeanAndStddev(X(0), A1, X(1), b, sigma);
  auto actual2 = conditional.sample(given);
  EXPECT_LONGS_EQUAL(1, actual2.size());
  EXPECT(assert_equal(A1 * x1 + b, actual2[X(0)], 50 * sigma));

  // Use a specific random generator
  std::mt19937_64 rng(4242);
  auto actual3 = conditional.sample(given, &rng);
  EXPECT_LONGS_EQUAL(1, actual2.size());
  // regression is not repeatable across platforms/versions :-(
  // EXPECT(assert_equal(Vector2(31.0111856, 64.9850775), actual2[X(0)], 1e-5));
}

/* ************************************************************************* */
TEST(GaussianConditional, Print) {
  Matrix A1 = (Matrix(2, 2) << 1., 2., 3., 4.).finished();
  Matrix A2 = (Matrix(2, 2) << 5., 6., 7., 8.).finished();
  const Vector2 b(20, 40);
  const double sigma = 3;

  GaussianConditional conditional(X(0), b, Matrix2::Identity(),
                                  noiseModel::Isotropic::Sigma(2, sigma));

  // Test printing for no parents.
  std::string expected =
    "GaussianConditional p(x0)\n"
    "  R = [ 1 0 ]\n"
    "      [ 0 1 ]\n"
    "  d = [ 20 40 ]\n"
    "isotropic dim=2 sigma=3\n";
  EXPECT(assert_print_equal(expected, conditional, "GaussianConditional"));

  auto conditional1 =
      GaussianConditional::FromMeanAndStddev(X(0), A1, X(1), b, sigma);

  // Test printing for single parent.
  std::string expected1 =
    "GaussianConditional p(x0 | x1)\n"
    "  R = [ 1 0 ]\n"
    "      [ 0 1 ]\n"
    "  S[x1] = [ -1 -2 ]\n"
    "          [ -3 -4 ]\n"
    "  d = [ 20 40 ]\n"
    "isotropic dim=2 sigma=3\n";
  EXPECT(assert_print_equal(expected1, conditional1, "GaussianConditional"));

  // Test printing for multiple parents.
  auto conditional2 = GaussianConditional::FromMeanAndStddev(X(0), A1, Y(0), A2,
                                                             Y(1), b, sigma);
  std::string expected2 =
    "GaussianConditional p(x0 | y0 y1)\n"
    "  R = [ 1 0 ]\n"
    "      [ 0 1 ]\n"
    "  S[y0] = [ -1 -2 ]\n"
    "          [ -3 -4 ]\n"
    "  S[y1] = [ -5 -6 ]\n"
    "          [ -7 -8 ]\n"
    "  d = [ 20 40 ]\n"
    "isotropic dim=2 sigma=3\n";
  EXPECT(assert_print_equal(expected2, conditional2, "GaussianConditional"));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
