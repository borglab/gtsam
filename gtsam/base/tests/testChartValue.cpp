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

#include <gtsam/base/ChartValue.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(ChartValue<double>)
GTSAM_CONCEPT_TESTABLE_INST(ChartValue<Vector3>)
GTSAM_CONCEPT_TESTABLE_INST(ChartValue<Pose3>)

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
TEST( testChartValue, construction_Pose3 ) {
  Pose3 p;
  DefaultChart<Pose3> chart(p);
  ChartValue<Pose3> value1(p), value2(chart);

  EXPECT(value1.dim() == 6);
  EXPECT(assert_equal(value1, value2));
}

/* ************************************************************************* */
TEST( testChartValue, addition_to_values ) {
  ChartValue<Pose3> pose((Pose3()));
  ChartValue<double> x(5.0);
  Vector4 v_; v_ << 1.0,2.0,3.0,4.0;
  ChartValue<Vector4> v(v_); 

  Values values;
  values.insert(42,pose);
  values.insert(43,x);
  values.insert(44,v);

  ChartValue<Pose3> p = values.at<ChartValue<Pose3> >(42);
  ChartValue<double> y = values.at<ChartValue<double> >(43);
  THROWS_EXCEPTION(ChartValue<Vector3> v3 = values.at<ChartValue<Vector3> >(44)); // throws !
  ChartValue<Vector4> v4 = values.at<ChartValue<Vector4> >(44);
  EXPECT(assert_equal(pose, p));
  EXPECT(assert_equal(x, y));
  EXPECT(assert_equal(v4, v));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


