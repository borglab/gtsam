/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testCustomChartExpression.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam_unstable/nonlinear/expressionTesting.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

// A simple prototype custom chart that does two things:
// 1. it reduces the dimension of the variable being estimated
// 2. it scales the input to retract.
//
// The Jacobian of this chart is:
// [ 2  0 ]
// [ 0  3 ]
// [ 0  0 ]
struct ProjectionChart {
  typedef Eigen::Vector3d type;
  typedef Eigen::Vector2d vector;
  static vector local(const type& origin, const type& other) {
    return (other - origin).head<2>();
  }
  static type retract(const type& origin, const vector& d) {
    return origin + Eigen::Vector3d(2.0 * d[0], 3.0 * d[1], 0.0);
  }
  static int getDimension(const type& origin) {
    return 2;
  }
};

namespace gtsam {
namespace traits {
template<> struct is_chart<ProjectionChart> : public boost::true_type {};
template<> struct dimension<ProjectionChart> : public boost::integral_constant<int, 2> {};
}  // namespace traits
}  // namespace gtsam

TEST(ExpressionCustomChart, projection) {
  // Create expression
  Expression<Eigen::Vector3d> point(1);

  Eigen::Vector3d pval(1.0, 2.0, 3.0);
  Values standard;
  standard.insert(1, pval);

  Values custom;
  custom.insert(1, pval, ProjectionChart());

  Eigen::Vector3d pstandard = point.value(standard);
  Eigen::Vector3d pcustom = point.value(custom);

  // The values should be the same.
  EXPECT(assert_equal(Matrix(pstandard), Matrix(pcustom), 1e-10));


  EXPECT_LONGS_EQUAL(3, standard.at(1).dim());
  VectorValues vstandard = standard.zeroVectors();
  EXPECT_LONGS_EQUAL(3, vstandard.at(1).size());


  EXPECT_LONGS_EQUAL(2, custom.at(1).dim());
  VectorValues vcustom = custom.zeroVectors();
  EXPECT_LONGS_EQUAL(2, vcustom.at(1).size());

  ExpressionFactor<Eigen::Vector3d> f(noiseModel::Unit::Create(pval.size()), pval, point);

  std::shared_ptr<GaussianFactor> gfstandard = f.linearize(standard);
  std::shared_ptr<JacobianFactor> jfstandard = //
      boost::dynamic_pointer_cast<JacobianFactor>(gfstandard);

  typedef std::pair<Eigen::MatrixXd, Eigen::VectorXd> Jacobian;
  Jacobian Jstandard = jfstandard->jacobianUnweighted();
  EXPECT(assert_equal(Eigen::Matrix3d::Identity(), Jstandard.first, 1e-10));

  std::shared_ptr<GaussianFactor> gfcustom = f.linearize(custom);
  std::shared_ptr<JacobianFactor> jfcustom = //
      boost::dynamic_pointer_cast<JacobianFactor>(gfcustom);

  Eigen::MatrixXd expectedJacobian = Eigen::MatrixXd::Zero(3,2);
  expectedJacobian(0,0) = 2.0;
  expectedJacobian(1,1) = 3.0;
  Jacobian Jcustom = jfcustom->jacobianUnweighted();
  EXPECT(assert_equal(expectedJacobian, Jcustom.first, 1e-10));

  // Amazingly, the finite differences get the expected Jacobian right.
  const double fd_step = 1e-5;
  const double tolerance = 1e-5;
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(point, custom, fd_step, tolerance);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

