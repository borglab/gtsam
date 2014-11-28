/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBasisDecompositions.cpp
 * @date November 23, 2014
 * @author Frank Dellaert
 * @brief unit tests for Basis Decompositions w Expressions
 */

#include <gtsam_unstable/nonlinear/expressions.h>
#include <gtsam_unstable/nonlinear/ExpressionFactor.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

using namespace std;
using namespace gtsam;

noiseModel::Diagonal::shared_ptr model = noiseModel::Unit::Create(1);

/// Fourier
template<int N>
class Fourier {

public:

  typedef Eigen::Matrix<double, N, 1> Coefficients;
  typedef Eigen::Matrix<double, 1, N> Jacobian;

private:

  double x_;
  Jacobian H_;

public:

  /// Constructor
  Fourier(double x) :
      x_(x) {
    H_(0, 0) = 1;
    for (size_t i = 1; i < N; i += 2) {
      H_(0, i) = cos(i * x);
      H_(0, i + 1) = sin(i * x);
    }
  }

  /// Given coefficients c, predict value for x
  double operator()(const Coefficients& c, FixedRef<1, N> H) {
    if (H)
      (*H) = H_;
    return H_ * c;
  }
};

//******************************************************************************
TEST(BasisDecompositions, Fourier) {
  Fourier<3> fx(0);
  Eigen::Matrix<double, 1, 3> expectedH, actualH;
  Vector3 c(1.5661, 1.2717, 1.2717);
  expectedH = numericalDerivative11<double, Vector3>(
      boost::bind(&Fourier<3>::operator(), fx, _1, boost::none), c);
  EXPECT_DOUBLES_EQUAL(c[0]+c[1], fx(c,actualH), 1e-9);
  EXPECT(assert_equal((Matrix)expectedH, actualH));
}

//******************************************************************************
TEST(BasisDecompositions, FourierExpression) {

  // Create linear factor graph
  GaussianFactorGraph g;
  Key key(1);
  Vector3_ c(key);
  for (size_t i = 0; i < 16; i++) {
    double x = i * M_PI / 8, y = exp(sin(x) + cos(x));

    // Manual JacobianFactor
    Matrix A(1, 3);
    A << 1, cos(x), sin(x);
    Vector b(1);
    b << y;
    JacobianFactor f1(key, A, b, model);

    // With ExpressionFactor
    Expression<double> expression(Fourier<3>(x), c);
    ExpressionFactor<double> f2(model, y, expression);

    g.add(f1);
  }

  // Solve
  VectorValues actual = g.optimize();

  // Check
  Vector3 expected(1.5661, 1.2717, 1.2717);
  EXPECT(assert_equal((Vector) expected, actual.at(key),1e-4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

