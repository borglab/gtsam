/**
 * @file testLinearTools.cpp
 *
 * @brief 
 *
 * @date Aug 27, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/linear/bayesTreeOperations.h>

using namespace gtsam;

SharedDiagonal model2 = noiseModel::Diagonal::Sigmas(ones(2));
SharedDiagonal model4 = noiseModel::Diagonal::Sigmas(ones(4));
SharedDiagonal model6 = noiseModel::Diagonal::Sigmas(ones(6));

/* ************************************************************************* */
TEST( testLinearTools, splitFactor ) {

  // Build upper-triangular system
  JacobianFactor initFactor(
       0,Matrix_(4, 2,
           1.0, 2.0,
           0.0, 3.0,
           0.0, 0.0,
           0.0, 0.0),
       1,Matrix_(4, 2,
           1.0, 2.0,
           9.0, 3.0,
           6.0, 8.0,
           0.0, 7.0),
       Vector_(4, 0.1, 0.2, 0.3, 0.4),
       model4);

  GaussianFactorGraph actSplit = splitFactor(initFactor.clone());
  GaussianFactorGraph expSplit;

  expSplit.add(
       0,Matrix_(2, 2,
           1.0, 2.0,
           0.0, 3.0),
       1,Matrix_(2, 2,
           1.0, 2.0,
           9.0, 3.0),
       Vector_(2, 0.1, 0.2),
       model2);
  expSplit.add(
       1,Matrix_(2, 2,
           6.0, 8.0,
           0.0, 7.0),
       Vector_(2, 0.3, 0.4),
       model2);

  EXPECT(assert_equal(expSplit, actSplit));
}

/* ************************************************************************* */
TEST_UNSAFE( testLinearTools, splitFactor2 ) {

  // Build upper-triangular system
  JacobianFactor initFactor(
       0,Matrix_(6, 2,
           1.0, 2.0,
           0.0, 3.0,
           0.0, 0.0,
           0.0, 0.0,
           0.0, 0.0,
           0.0, 0.0),
       1,Matrix_(6, 2,
           1.0, 2.0,
           9.0, 3.0,
           6.0, 8.0,
           0.0, 7.0,
           0.0, 0.0,
           0.0, 0.0),
       2,Matrix_(6, 2,
           1.1, 2.2,
           9.1, 3.2,
           6.1, 8.2,
           0.1, 7.2,
           0.1, 3.2,
           0.0, 1.2),
       Vector_(6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
       model6);

  GaussianFactorGraph actSplit = splitFactor(initFactor.clone());
  GaussianFactorGraph expSplit;

  expSplit.add(
       0,Matrix_(2, 2,
           1.0, 2.0,
           0.0, 3.0),
       1,Matrix_(2, 2,
           1.0, 2.0,
           9.0, 3.0),
       2,Matrix_(2, 2,
           1.1, 2.2,
           9.1, 3.2),
       Vector_(2, 0.1, 0.2),
       model2);
  expSplit.add(
       1,Matrix_(2, 2,
           6.0, 8.0,
           0.0, 7.0),
       2,Matrix_(2, 2,
           6.1, 8.2,
           0.1, 7.2),
       Vector_(2, 0.3, 0.4),
       model2);
  expSplit.add(
      2,Matrix_(2, 2,
           0.1, 3.2,
           0.0, 1.2),
       Vector_(2, 0.5, 0.6),
       model2);

  EXPECT(assert_equal(expSplit, actSplit));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
