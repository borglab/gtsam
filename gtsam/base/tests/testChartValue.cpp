/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testChartValue.cpp
 * @author Michael Bosse
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/ChartValue.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(ChartValue<double>)
GTSAM_CONCEPT_TESTABLE_INST(ChartValue<Vector3>)

/* ************************************************************************* */
TEST( testChartValue, construction_double ) {
  double d = 5.0;
  DefaultChart<double> chart(d);
  ChartValue<double> value1(d), value2(chart);

  EXPECT(value1.dim() == 1);
  EXPECT(assert_equal(value1, value2));
}

/* ************************************************************************* */
TEST( testChartValue, construction_Vector3 ) {
  Vector3 v; v << 1.0 , 2.0, 3.0;
  DefaultChart<Vector3> chart(v);
  ChartValue<Vector3> value1(v), value2(chart);

  EXPECT(value1.dim() == 3);
  EXPECT(assert_equal(value1, value2));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


