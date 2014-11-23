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
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>
using boost::assign::list_of;

using namespace std;
using namespace gtsam;

noiseModel::Diagonal::shared_ptr model = noiseModel::Unit::Create(1);

//******************************************************************************
TEST(BasisDecompositions, Fourier) {

  // Create linear factor graph
  GaussianFactorGraph g;
  Key key(1);
  for (size_t i = 0; i < 16; i++) {
    double x = i * M_PI / 8, y = exp(sin(x) + cos(x));
    Matrix A(1, 3);
    A << 1, cos(x), sin(x);
    Vector b(1);
    b << y;
    g.add(key, A, b, model);
  }

  // Solve
  VectorValues actual = g.optimize();

  // Check
  Vector3 expected(1.5661, 1.2717, 1.2717);
  CHECK(assert_equal((Vector) expected, actual.at(key),1e-4));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

