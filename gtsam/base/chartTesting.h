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

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestResult.h>
#include <CppUnitLite/Test.h>
#include <CppUnitLite/Failure.h>

namespace gtsam {
// Do a full concept check and test the invertibility of local() vs. retract().
template<typename T>
void testDefaultChart(TestResult& result_,
                      const std::string& name_,
                      const T& value) {

  GTSAM_CONCEPT_TESTABLE_TYPE(T)

  typedef typename gtsam::DefaultChart<T> Chart;
  typedef typename Chart::vector Vector;

  // First, check the basic chart concept. This checks that the interface is satisfied.
  // The rest of the function is even more detailed, checking the correctness of the chart.
  BOOST_CONCEPT_ASSERT((ChartConcept<Chart>));

  T other = value;

  // Check that the dimension of the local value matches the chart dimension.
  Vector dx = Chart::local(value, other);
  EXPECT_LONGS_EQUAL(Chart::getDimension(value), dx.size());
  // And that the "local" of a value vs. itself is zero.
  EXPECT(assert_equal(Matrix(dx), Matrix(Eigen::VectorXd::Zero(dx.size()))));

  // Test the invertibility of retract/local
  dx.setRandom();
  T updated = Chart::retract(value, dx);
  Vector invdx = Chart::local(value, updated);
  EXPECT(assert_equal(Matrix(dx), Matrix(invdx), 1e-9));

  // And test that negative steps work as well.
  dx = -dx;
  updated = Chart::retract(value, dx);
  invdx = Chart::local(value, updated);
  EXPECT(assert_equal(Matrix(dx), Matrix(invdx), 1e-9));
}
}  // namespace gtsam

/// \brief Perform a concept check on the default chart for a type.
/// \param value An instantiation of the type to be tested.
#define CHECK_CHART_CONCEPT(value) \
    { gtsam::testDefaultChart(result_, name_, value); }
