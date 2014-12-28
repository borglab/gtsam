/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file chartTesting.h
 * @brief
 * @date November, 2014
 * @author Paul Furgale
 */

#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestResult.h>
#include <CppUnitLite/Test.h>
#include <CppUnitLite/Failure.h>

namespace gtsam {

// Do a comprehensive test of Lie Group derivatives
template<typename G>
void testLieGroupDerivatives(TestResult& result_, const std::string& name_,
    const G& t1, const G& t2) {

  Matrix H1, H2;
  typedef traits<G> T;

  // Inverse
  EXPECT(assert_equal(t1.inverse(),T::Inverse(t1, H1)));
  EXPECT(assert_equal(numericalDerivative11(T::Inverse, t1),H1));

  EXPECT(assert_equal(t2.inverse(),T::Inverse(t2, H1)));
  EXPECT(assert_equal(numericalDerivative11(T::Inverse, t2),H1));

  // Compose
  EXPECT(assert_equal(t1 * t2,T::Compose(t1, t2, H1, H2)));
  EXPECT(assert_equal(numericalDerivative21(T::Compose, t1, t2), H1));
  EXPECT(assert_equal(numericalDerivative22(T::Compose, t1, t2), H2));

  // Between
  EXPECT(assert_equal(t1.inverse() * t2,T::Between(t1, t2, H1, H2)));
  EXPECT(assert_equal(numericalDerivative21(T::Between, t1, t2), H1));
  EXPECT(assert_equal(numericalDerivative22(T::Between, t1, t2), H2));
}
// Do a comprehensive test of Lie Group Chart derivatives
template<typename G>
void testChartDerivatives(TestResult& result_, const std::string& name_,
    const G& t1, const G& t2) {

  Matrix H1, H2;
  typedef traits<G> T;

  // Retract
  typename G::TangentVector w12 = T::Local(t1, t2);
  EXPECT(assert_equal(t2, T::Retract(t1,w12, H1, H2)));
  EXPECT(assert_equal(numericalDerivative21(T::Retract, t1, w12), H1));
  EXPECT(assert_equal(numericalDerivative22(T::Retract, t1, w12), H2));

  // Local
  EXPECT(assert_equal(w12, t1.localCoordinates(t2, H1, H2)));
  EXPECT(assert_equal(numericalDerivative21(T::Local, t1, t2), H1));
  EXPECT(assert_equal(numericalDerivative22(T::Local, t1, t2), H2));
}
} // namespace gtsam

#define CHECK_LIE_GROUP_DERIVATIVES(t1,t2) \
    { gtsam::testLieGroupDerivatives(result_, name_, t1, t2); }

#define CHECK_CHART_DERIVATIVES(t1,t2) \
    { gtsam::testChartDerivatives(result_, name_, t1, t2); }
