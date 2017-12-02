/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGaussianBayesNet.cpp
 * @brief   Unit tests for GaussianBayesNet
 * @author  Frank Dellaert
 */

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/assign/list_of.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

// STL/C++
#include <iostream>
#include <sstream>
#include <CppUnitLite/TestHarness.h>
#include <boost/tuple/tuple.hpp>

using namespace std;
using namespace gtsam;

static const Key _x_=0, _y_=1;

static GaussianBayesNet smallBayesNet =
    list_of(GaussianConditional(_x_, Vector1::Constant(9), I_1x1, _y_, I_1x1))(
        GaussianConditional(_y_, Vector1::Constant(5), I_1x1));

static GaussianBayesNet noisyBayesNet =
    list_of(GaussianConditional(_x_, Vector1::Constant(9), I_1x1, _y_, I_1x1,
                                noiseModel::Diagonal::Sigmas(Vector1::Constant(2))))(
        GaussianConditional(_y_, Vector1::Constant(5), I_1x1,
                            noiseModel::Diagonal::Sigmas(Vector1::Constant(3))));

/* ************************************************************************* */
TEST( GaussianBayesNet, Matrix )
{
  Matrix R; Vector d;
  boost::tie(R,d) = smallBayesNet.matrix(); // find matrix and RHS

  Matrix R1 = (Matrix2() <<
          1.0, 1.0,
          0.0, 1.0
    ).finished();
  Vector d1 = Vector2(9.0, 5.0);

  EXPECT(assert_equal(R,R1));
  EXPECT(assert_equal(d,d1));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, NoisyMatrix )
{
  Matrix R; Vector d;
  boost::tie(R,d) = noisyBayesNet.matrix(); // find matrix and RHS

  Matrix R1 = (Matrix2() <<
          0.5, 0.5,
          0.0, 1./3.
    ).finished();
  Vector d1 = Vector2(9./2., 5./3.);

  EXPECT(assert_equal(R,R1));
  EXPECT(assert_equal(d,d1));
}

/* ************************************************************************* */
TEST(GaussianBayesNet, Optimize) {
  VectorValues expected =
      map_list_of<Key, Vector>(_x_, Vector1::Constant(4))(_y_, Vector1::Constant(5));
  VectorValues actual = smallBayesNet.optimize();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianBayesNet, NoisyOptimize) {
  Matrix R;
  Vector d;
  boost::tie(R, d) = noisyBayesNet.matrix();  // find matrix and RHS
  const Vector x = R.inverse() * d;
  VectorValues expected = map_list_of<Key, Vector>(_x_, x.head(1))(_y_, x.tail(1));

  VectorValues actual = noisyBayesNet.optimize();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimizeIncomplete )
{
  static GaussianBayesNet incompleteBayesNet = list_of
    (GaussianConditional(_x_, Vector1::Constant(9), I_1x1, _y_, I_1x1));

  VectorValues solutionForMissing = map_list_of<Key, Vector>
    (_y_, Vector1::Constant(5));

  VectorValues actual = incompleteBayesNet.optimize(solutionForMissing);

  VectorValues expected = map_list_of<Key, Vector>
    (_x_, Vector1::Constant(4))
    (_y_, Vector1::Constant(5));

  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, optimize3 )
{
  // y = R*x, x=inv(R)*y
  // 4 = 1 1   -1 
  // 5     1    5
  // NOTE: we are supplying a new RHS here

  VectorValues expected = map_list_of<Key, Vector>
    (_x_, Vector1::Constant(-1))
    (_y_, Vector1::Constant(5));

  // Test different RHS version
  VectorValues gx = map_list_of<Key, Vector>
    (_x_, Vector1::Constant(4))
    (_y_, Vector1::Constant(5));
  VectorValues actual = smallBayesNet.backSubstitute(gx);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianBayesNet, backSubstituteTranspose )
{
  // x=R'*y, expected=inv(R')*x
  // 2 = 1    2
  // 5   1 1  3
  VectorValues
    x = map_list_of<Key, Vector>
      (_x_, Vector1::Constant(2))
      (_y_, Vector1::Constant(5)),
    expected = map_list_of<Key, Vector>
      (_x_, Vector1::Constant(2))
      (_y_, Vector1::Constant(3));

  VectorValues actual = smallBayesNet.backSubstituteTranspose(x);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Tests computing Determinant
TEST( GaussianBayesNet, DeterminantTest )
{
  GaussianBayesNet cbn;
  cbn += GaussianConditional(
          0, Vector2(3.0, 4.0), (Matrix2() << 1.0, 3.0, 0.0, 4.0).finished(),
          1, (Matrix2() << 2.0, 1.0, 2.0, 3.0).finished(), noiseModel::Isotropic::Sigma(2, 2.0));

  cbn += GaussianConditional(
          1, Vector2(5.0, 6.0), (Matrix2() << 1.0, 1.0, 0.0, 3.0).finished(),
          2, (Matrix2() << 1.0, 0.0, 5.0, 2.0).finished(), noiseModel::Isotropic::Sigma(2, 2.0));

  cbn += GaussianConditional(
      3, Vector2(7.0, 8.0), (Matrix2() << 1.0, 1.0, 0.0, 5.0).finished(), noiseModel::Isotropic::Sigma(2, 2.0));

  double expectedDeterminant = 60.0 / 64.0;
  double actualDeterminant = cbn.determinant();

  EXPECT_DOUBLES_EQUAL( expectedDeterminant, actualDeterminant, 1e-9);
}

/* ************************************************************************* */
namespace {
  double computeError(const GaussianBayesNet& gbn, const Vector10& values)
  {
    pair<Matrix,Vector> Rd = GaussianFactorGraph(gbn).jacobian();
    return 0.5 * (Rd.first * values - Rd.second).squaredNorm();
  }
}

/* ************************************************************************* */
TEST(GaussianBayesNet, ComputeSteepestDescentPoint) {

  // Create an arbitrary Bayes Net
  GaussianBayesNet gbn;
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
    0, Vector2(1.0,2.0), (Matrix2() << 3.0,4.0,0.0,6.0).finished(),
    3, (Matrix2() << 7.0,8.0,9.0,10.0).finished(),
    4, (Matrix2() << 11.0,12.0,13.0,14.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
    1, Vector2(15.0,16.0), (Matrix2() << 17.0,18.0,0.0,20.0).finished(),
    2, (Matrix2() << 21.0,22.0,23.0,24.0).finished(),
    4, (Matrix2() << 25.0,26.0,27.0,28.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
    2, Vector2(29.0,30.0), (Matrix2() << 31.0,32.0,0.0,34.0).finished(),
    3, (Matrix2() << 35.0,36.0,37.0,38.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
    3, Vector2(39.0,40.0), (Matrix2() << 41.0,42.0,0.0,44.0).finished(),
    4, (Matrix2() << 45.0,46.0,47.0,48.0).finished()));
  gbn += GaussianConditional::shared_ptr(new GaussianConditional(
    4, Vector2(49.0,50.0), (Matrix2() << 51.0,52.0,0.0,54.0).finished()));

  // Compute the Hessian numerically
  Matrix hessian = numericalHessian<Vector10>(
      boost::bind(&computeError, gbn, _1), Vector10::Zero());

  // Compute the gradient numerically
  Vector gradient = numericalGradient<Vector10>(
      boost::bind(&computeError, gbn, _1), Vector10::Zero());

  // Compute the gradient using dense matrices
  Matrix augmentedHessian = GaussianFactorGraph(gbn).augmentedHessian();
  LONGS_EQUAL(11, (long)augmentedHessian.cols());
  Vector denseMatrixGradient = -augmentedHessian.col(10).segment(0,10);
  EXPECT(assert_equal(gradient, denseMatrixGradient, 1e-5));

  // Compute the steepest descent point
  double step = -gradient.squaredNorm() / (gradient.transpose() * hessian * gradient)(0);
  Vector expected = gradient * step;

  // Compute the steepest descent point with the dogleg function
  VectorValues actual = gbn.optimizeGradientSearch();

  // Check that points agree
  FastVector<Key> keys = list_of(0)(1)(2)(3)(4);
  Vector actualAsVector = actual.vector(keys);
  EXPECT(assert_equal(expected, actualAsVector, 1e-5));

  // Check that point causes a decrease in error
  double origError = GaussianFactorGraph(gbn).error(VectorValues::Zero(actual));
  double newError = GaussianFactorGraph(gbn).error(actual);
  EXPECT(newError < origError);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
