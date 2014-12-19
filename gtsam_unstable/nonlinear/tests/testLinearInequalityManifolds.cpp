///* ----------------------------------------------------------------------------
//
// * GTSAM Copyright 2010, Georgia Tech Research Corporation,
// * Atlanta, Georgia 30332-0415
// * All Rights Reserved
// * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
//
// * See LICENSE for the license information
//
// * -------------------------------------------------------------------------- */
//
///**
// * @file    testLinearInequalityManifolds.cpp
// * @brief   Unit tests for testLinearInequalityManifolds
// * @author  Krunal Chande
// * @author  Duy-Nguyen Ta
// * @author  Luca Carlone
// * @date    Dec 15, 2014
// */
//
//#include <gtsam/inference/Symbol.h>
//#include <CppUnitLite/TestHarness.h>
//#include <iostream>
//
//using namespace std;
//using namespace gtsam::symbol_shorthand;
//using namespace gtsam;
//const double tol = 1e-10;
//
//namespace gtsam {
//class PositionConstraint : public LinearInequalityManifold1 {
//public:
//  PositionConstraint()
//};
//}
////******************************************************************************
//TEST(testLinearEqualityManifolds, equals ) {
//  // Instantiate a class LinearEqualityManifolds
//  LinearEqualityManifolds le1();
//
//  // Instantiate another class LinearEqualityManifolds
//  LinearEqualityManifolds le2();
//
//  // check equals
//  CHECK(assert_equal(le1,le1));
//  CHECK(le2.equals(le2));
//  CHECK(!le2.equals(le1));
//  CHECK(!le1.equals(le2));
//}
//
//////******************************************************************************
////TEST(testLinearEqualityManifolds, linearize ) {
////}
////
//////******************************************************************************
////TEST(testLinearEqualityManifolds, size ) {
////}
//
////******************************************************************************
//int main() {
//  TestResult tr;
//  return TestRegistry::runAllTests(tr);
//}
////******************************************************************************
//
