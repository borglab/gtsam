/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testFactorTesting.cpp
 * @date September 18, 2014
 * @author Brice Rebsamen
 * @brief unit tests for testFactorJacobians and testExpressionJacobians
 */

#include <gtsam/nonlinear/expressions.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressionTesting.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
class ScaleAndCompare {
public:
  ScaleAndCompare() = default;
  explicit ScaleAndCompare(const Vector3& other) : other_(other) {}
  Vector3 operator()(const Vector3& v,
                        const double& s,
                        OptionalJacobian<3, 3> Dv,
                        OptionalJacobian<3, 1> Ds) const {
    const Vector3 e = v - other_ * s;
    if(Dv) {
      *Dv = I_3x3;
    }
    if(Ds) {
      *Ds = -other_;
    }
    return e;
  }

private:
  Vector3 other_ = Z_3x1;
};

/* ************************************************************************* */
// Constant
TEST(ExpressionTesting, ScaleAndCompare) {
  const double tol = 1e-4;
  const double numerical_step = 1e-3;

  const Vector3 other(1, 0, 0);
  Values values;
  values.insert<Vector3>(Symbol('v', 0), Vector3(1.1, 0, 0));
  values.insert<double>(Symbol('s', 0), 1.1);

  const auto err_expr = Expression<Vector3>(ScaleAndCompare{other},
                                            Vector3_(Symbol('v', 0)),
                                            Double_(Symbol('s', 0)));

  const auto err = err_expr.value(values);
  EXPECT_LONGS_EQUAL(3, err.size());
  EXPECT(assert_equal(Vector3(Z_3x1), err));
  EXPECT(internal::testExpressionJacobians(
      "ScaleAndCompare", err_expr, values, numerical_step, tol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

