/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testRFID.cpp
 * @brief   Unit tests for the RFID factor
 * @author  Stephen Williams (swilliams8@gatech.edu)
 * @date    Jan 16, 2012
 */

#include <gtsam_unstable/nonlinear/LinearizedFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
TEST( LinearizedFactor, equals_jacobian )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  Key x1(1);
  Key x2(2);
  Values values;
  values.insert(x1, Point3(-22.4,  +8.5,  +2.4));
  values.insert(x2, Point3(-21.0,  +5.0, +21.0));

  Point3 measured(1.0, -2.5, 17.8);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
  BetweenFactor<Point3> betweenFactor(x1, x2, measured, model);


  // Create two identical factors and make sure they're equal
  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
  LinearizedJacobianFactor jacobian1(jf, values);
  LinearizedJacobianFactor jacobian2(jf, values);

  CHECK(assert_equal(jacobian1, jacobian2));
}

/* ************************************************************************* */
TEST( LinearizedFactor, clone_jacobian )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  Key x1(1);
  Key x2(2);
  Values values;
  values.insert(x1, Point3(-22.4,  +8.5,  +2.4));
  values.insert(x2, Point3(-21.0,  +5.0, +21.0));

  Point3 measured(1.0, -2.5, 17.8);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
  BetweenFactor<Point3> betweenFactor(x1, x2, measured, model);

  // Create one factor that is a clone of the other and make sure they're equal
  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
  LinearizedJacobianFactor jacobian1(jf, values);
  LinearizedJacobianFactor::shared_ptr jacobian2 = std::static_pointer_cast<LinearizedJacobianFactor>(jacobian1.clone());
  CHECK(assert_equal(jacobian1, *jacobian2));

  JacobianFactor::shared_ptr jf1 = std::static_pointer_cast<JacobianFactor>(jacobian1.linearize(values));
  JacobianFactor::shared_ptr jf2 = std::static_pointer_cast<JacobianFactor>(jacobian2->linearize(values));
  CHECK(assert_equal(*jf1, *jf2));

  Matrix information1 = jf1->augmentedInformation();
  Matrix information2 = jf2->augmentedInformation();
  CHECK(assert_equal(information1, information2));
}

/* ************************************************************************* */
TEST( LinearizedFactor, add_jacobian )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  Key x1(1);
  Key x2(2);
  Values values;
  values.insert(x1, Point3(-22.4,  +8.5,  +2.4));
  values.insert(x2, Point3(-21.0,  +5.0, +21.0));

  Point3 measured(1.0, -2.5, 17.8);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
  BetweenFactor<Point3> betweenFactor(x1, x2, measured, model);

  // Create two factor graphs, one using 'push_back' and one using 'add' and make sure they're equal
  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
  LinearizedJacobianFactor::shared_ptr jacobian(new LinearizedJacobianFactor(jf, values));
  NonlinearFactorGraph graph1; graph1.push_back(jacobian);
  NonlinearFactorGraph graph2; graph2.push_back(*jacobian);

  // TODO: When creating a Jacobian from a cached factor, I experienced a problem in the 'add' version
  // However, I am currently unable to reproduce the error in this unit test.
  // I don't know if this affects the Hessian version as well.
  CHECK(assert_equal(graph1, graph2));
}

///* ************************************************************************* */
//TEST( LinearizedFactor, error_jacobian )
//{
//  // Create a Between Factor from a Point3. This is actually a linear factor.
//  Key key1(1);
//  Key key2(2);
//  Ordering ordering;
//  ordering.push_back(key1);
//  ordering.push_back(key2);
//  Values values;
//  values.insert(key1, Point3(-22.4,  +8.5,  +2.4));
//  values.insert(key2, Point3(-21.0,  +5.0, +21.0));
//
//  Point3 measured(1.0, -2.5, 17.8);
//  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
//  BetweenFactor<Point3> betweenFactor(key1, key2, measured, model);
//
//
//  // Create a linearized jacobian factors
//  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
//  LinearizedJacobianFactor jacobian(jf, values);
//
//
//  for(double x1 = -10; x1 < 10; x1 += 2.0) {
//    for(double y1 = -10; y1 < 10; y1 += 2.0) {
//      for(double z1 = -10; z1 < 10; z1 += 2.0) {
//
//        for(double x2 = -10; x2 < 10; x2 += 2.0) {
//          for(double y2 = -10; y2 < 10; y2 += 2.0) {
//            for(double z2 = -10; z2 < 10; z2 += 2.0) {
//
//              Values linpoint;
//              linpoint.insert(key1, Point3(x1, y1, z1));
//              linpoint.insert(key2, Point3(x2, y2, z2));
//
//              // Check that the error of the Linearized Jacobian and the original factor match
//              // This only works because a BetweenFactor on a Point3 is actually a linear system
//              double expected_error = betweenFactor.error(linpoint);
//              double actual_error = jacobian.error(linpoint);
//              EXPECT_DOUBLES_EQUAL(expected_error, actual_error, 1e-9 );
//
//              // Check that the linearized factors are identical
//              GaussianFactor::shared_ptr expected_factor = betweenFactor.linearize(linpoint);
//              GaussianFactor::shared_ptr actual_factor   = jacobian.linearize(linpoint);
//              CHECK(assert_equal(*expected_factor, *actual_factor));
//            }
//          }
//        }
//
//      }
//    }
//  }
//
//}

/* ************************************************************************* */
TEST( LinearizedFactor, equals_hessian )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  Key x1(1);
  Key x2(2);
  Values values;
  values.insert(x1, Point3(-22.4,  +8.5,  +2.4));
  values.insert(x2, Point3(-21.0,  +5.0, +21.0));

  Point3 measured(1.0, -2.5, 17.8);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
  BetweenFactor<Point3> betweenFactor(x1, x2, measured, model);


  // Create two identical factors and make sure they're equal
  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
  HessianFactor::shared_ptr hf(new HessianFactor(*jf));
  LinearizedHessianFactor hessian1(hf, values);
  LinearizedHessianFactor hessian2(hf, values);

  CHECK(assert_equal(hessian1, hessian2));
}

/* ************************************************************************* */
TEST( LinearizedFactor, clone_hessian )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  Key x1(1);
  Key x2(2);
  Values values;
  values.insert(x1, Point3(-22.4,  +8.5,  +2.4));
  values.insert(x2, Point3(-21.0,  +5.0, +21.0));

  Point3 measured(1.0, -2.5, 17.8);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
  BetweenFactor<Point3> betweenFactor(x1, x2, measured, model);


  // Create two identical factors and make sure they're equal
  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
  HessianFactor::shared_ptr hf(new HessianFactor(*jf));
  LinearizedHessianFactor hessian1(hf, values);
  LinearizedHessianFactor::shared_ptr hessian2 = std::static_pointer_cast<LinearizedHessianFactor>(hessian1.clone());

  CHECK(assert_equal(hessian1, *hessian2));
}

/* ************************************************************************* */
TEST( LinearizedFactor, add_hessian )
{
  // Create a Between Factor from a Point3. This is actually a linear factor.
  Key x1(1);
  Key x2(2);
  Values values;
  values.insert(x1, Point3(-22.4,  +8.5,  +2.4));
  values.insert(x2, Point3(-21.0,  +5.0, +21.0));

  Point3 measured(1.0, -2.5, 17.8);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
  BetweenFactor<Point3> betweenFactor(x1, x2, measured, model);


  // Create two identical factors and make sure they're equal
  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
  HessianFactor::shared_ptr hf(new HessianFactor(*jf));
  LinearizedHessianFactor::shared_ptr hessian(new LinearizedHessianFactor(hf, values));
  NonlinearFactorGraph graph1; graph1.push_back(hessian);
  NonlinearFactorGraph graph2; graph2.push_back(*hessian);

  CHECK(assert_equal(graph1, graph2));
}

///* ************************************************************************* */
//TEST( LinearizedFactor, error_hessian )
//{
//  // Create a Between Factor from a Point3. This is actually a linear factor.
//  Key key1(1);
//  Key key2(2);
//  Ordering ordering;
//  ordering.push_back(key1);
//  ordering.push_back(key2);
//  Values values;
//  values.insert(key1, Point3(-22.4,  +8.5,  +2.4));
//  values.insert(key2, Point3(-21.0,  +5.0, +21.0));
//
//  Point3 measured(1.0, -2.5, 17.8);
//  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 0.1);
//  BetweenFactor<Point3> betweenFactor(key1, key2, measured, model);
//
//
//  // Create a linearized hessian factor
//  JacobianFactor::shared_ptr jf = std::static_pointer_cast<JacobianFactor>(betweenFactor.linearize(values));
//  HessianFactor::shared_ptr hf(new HessianFactor(*jf));
//  LinearizedHessianFactor hessian(hf, values);
//
//
//  for(double x1 = -10; x1 < 10; x1 += 2.0) {
//    for(double y1 = -10; y1 < 10; y1 += 2.0) {
//      for(double z1 = -10; z1 < 10; z1 += 2.0) {
//
//        for(double x2 = -10; x2 < 10; x2 += 2.0) {
//          for(double y2 = -10; y2 < 10; y2 += 2.0) {
//            for(double z2 = -10; z2 < 10; z2 += 2.0) {
//
//              Values linpoint;
//              linpoint.insert(key1, Point3(x1, y1, z1));
//              linpoint.insert(key2, Point3(x2, y2, z2));
//
//              // Check that the error of the Linearized Hessian and the original factor match
//              // This only works because a BetweenFactor on a Point3 is actually a linear system
//              double expected_error = betweenFactor.error(linpoint);
//              double actual_error = hessian.error(linpoint);
//              EXPECT_DOUBLES_EQUAL(expected_error, actual_error, 1e-9 );
//
//              // Check that the linearized factors are identical
//              GaussianFactor::shared_ptr expected_factor = HessianFactor::shared_ptr(new HessianFactor(*betweenFactor.linearize(linpoint)));
//              GaussianFactor::shared_ptr actual_factor   = hessian.linearize(linpoint);
//              CHECK(assert_equal(*expected_factor, *actual_factor));
//            }
//          }
//        }
//
//      }
//    }
//  }
//
//}

/* ************************************************************************* */
int main()
{
    TestResult tr; return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

