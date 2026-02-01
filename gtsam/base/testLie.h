/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testLie.h
 * @brief Test utilities for Lie groups
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
  typedef OptionalJacobian<T::dimension,T::dimension> OJ;

  // Inverse
  OJ none;
  EXPECT(assert_equal<G>(t1.inverse(),T::Inverse(t1, H1)));
  EXPECT(assert_equal(numericalDerivative21<G,G,OJ>(T::Inverse, t1, none),H1));

  EXPECT(assert_equal<G>(t2.inverse(),T::Inverse(t2, H1)));
  EXPECT(assert_equal(numericalDerivative21<G,G,OJ>(T::Inverse, t2, none),H1));

  // Compose
  EXPECT(assert_equal<G>(t1 * t2,T::Compose(t1, t2, H1, H2)));
  EXPECT(assert_equal(numericalDerivative41<G,G,G,OJ,OJ>(T::Compose, t1, t2, none, none), H1));
  EXPECT(assert_equal(numericalDerivative42<G,G,G,OJ,OJ>(T::Compose, t1, t2, none, none), H2));

  // Between
  EXPECT(assert_equal<G>(t1.inverse() * t2,T::Between(t1, t2, H1, H2)));
  EXPECT(assert_equal(numericalDerivative41<G,G,G,OJ,OJ>(T::Between, t1, t2, none, none), H1));
  EXPECT(assert_equal(numericalDerivative42<G,G,G,OJ,OJ>(T::Between, t1, t2, none, none), H2));
}

// Do a comprehensive test of Lie Group Chart derivatives
template<typename G>
void testChartDerivatives(TestResult& result_, const std::string& name_,
    const G& t1, const G& t2) {

  Matrix H1, H2;
  typedef traits<G> T;
  typedef typename T::TangentVector V;
  typedef OptionalJacobian<T::dimension,T::dimension> OJ;

  // Retract
  OJ none;
  V w12 = T::Local(t1, t2);
  EXPECT(assert_equal<G>(t2, T::Retract(t1,w12, H1, H2)));
  EXPECT(assert_equal(numericalDerivative41<G,G,V,OJ,OJ>(T::Retract, t1, w12, none, none), H1));
  EXPECT(assert_equal(numericalDerivative42<G,G,V,OJ,OJ>(T::Retract, t1, w12, none, none), H2));

  // Local
  EXPECT(assert_equal(w12, T::Local(t1, t2, H1, H2)));
  EXPECT(assert_equal(numericalDerivative41<V,G,G,OJ,OJ>(T::Local, t1, t2, none, none), H1));
  EXPECT(assert_equal(numericalDerivative42<V,G,G,OJ,OJ>(T::Local, t1, t2, none, none), H2));
}
} // namespace gtsam

#define CHECK_LIE_GROUP_DERIVATIVES(t1,t2) \
    { gtsam::testLieGroupDerivatives(result_, name_, t1, t2); }

#define CHECK_CHART_DERIVATIVES(t1,t2) \
    { gtsam::testChartDerivatives(result_, name_, t1, t2); }
