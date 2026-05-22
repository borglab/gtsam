/** This unit test exercises the functionality of the Riemannian
 * gradient descent method */

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "Optimization/Riemannian/GradientDescent.h"

// Typedef for the numerical type will use in the following tests
typedef double Scalar;

// Threshold for considering two values to be numerically equal in the following
// tests
Scalar GD_test_error_threshold = 1e-4;

TEST(GradientDescentUnitTest, EuclideanGradientDescentRosenbrock) {

  /// Test minimization of the Rosenbrock function
  ///
  /// f(x,y) = (a - x)^2 + b(y - x^2)^2
  ///
  /// whose (global) minimizer is f(a,a^2) = 0,
  ///
  /// using the simplified Euclidean interface

  // Rosenbrock function parameters
  Scalar a = 1;
  Scalar b = 100;

  typedef Eigen::Matrix<Scalar, 2, 1> Vector;
  typedef Eigen::Matrix<Scalar, 2, 2> Matrix;

  // Objective: f(x,y) = (a - x)^2  + b(y - x^2)^2
  Optimization::Objective<Vector, Scalar> F = [a, b](const Vector &x) {
    return std::pow(a - x(0), 2) + b * std::pow((x(1) - std::pow(x(0), 2)), 2);
  };

  // Euclidean gradient operator: returns the Euclidean gradient nablaF(X) at
  // each X df/dx = -2(a-x) - 4bx(y-x^2) df / dy = 2b(y-x^2)
  Optimization::Riemannian::EuclideanVectorField<Vector> nabla_F =
      [a, b](const Vector &x) {
        Vector df;
        df(0) = -2 * (a - x(0)) - 4 * b * x(0) * (x(1) - std::pow(x(0), 2));
        df(1) = 2 * b * (x(1) - std::pow(x(0), 2));
        return df;
      };

  /// Sample initial point

  Vector x0(.1, .1);
  Vector x_min(a, a * a);

  /// Run gradient descent optimizer

  // Set TNT options
  Optimization::Riemannian::GradientDescentParams<Scalar> params;
  params.gradient_tolerance = 1e-6;
  params.relative_decrease_tolerance = 0;
  params.stepsize_tolerance = 0;
  params.max_iterations = 1000000;

  Optimization::Riemannian::GradientDescentResult<Vector, Scalar> result =
      Optimization::Riemannian::EuclideanGradientDescent<Vector, Scalar>(
          F, nabla_F, x0, params);

  // Check function value
  EXPECT_NEAR(result.f, 0, GD_test_error_threshold);

  // Check gradient value
  EXPECT_NEAR(result.gradfx_norm, 0, GD_test_error_threshold);

  // Check final solution
  EXPECT_NEAR((result.x - x_min).norm(), 0, GD_test_error_threshold);
}

TEST(GradientDescentUnitTest, RiemannianGradientDescentSphere) {

  /// Test minimization of the function f(X; P) := || X - P ||^2, over the
  /// sphere S^2 in R^3, where P is a fixed point on the sphere
  ///
  /// In the example that follows, we will treat the point P as a fixed (i.e.
  /// unoptimized) parameter of the objective f, and pass this to the gradient
  /// descent optimization function as a user-supplied optional argument.

  typedef Eigen::Matrix<Scalar, 3, 1> Vector;

  Vector P = {0.0, 0.0, 1.0}; // P is the north pole

  /// Set up function handles

  /// Utility function: projection of the vector V \in R^3 onto the tangent
  /// space T_X(S^2) of S^2 at the point
  auto project = [](const Vector &X, const Vector &V) -> Vector {
    return V - X.dot(V) * X;
  };

  /// Objective
  Optimization::Objective<Vector, Scalar, Vector> F =
      [](const Vector &X, const Vector &P) { return (X - P).squaredNorm(); };

  /// Gradient
  Optimization::Riemannian::VectorField<Vector, Vector, Vector> grad_F =
      [&project](const Vector &X, const Vector &P) {
        // Euclidean gradient
        Vector nabla_f = 2 * (X - P);

        // Compute Riemannian gradient from Euclidean one
        return project(X, nabla_f);
      };

  /// Riemannian metric on S^2: this is just the usual inner-product on R^3
  Optimization::Riemannian::RiemannianMetric<Vector, Vector, Scalar, Vector>
      metric = [](const Vector &X, const Vector &V1, const Vector &V2,
                  const Vector &P) { return V1.dot(V2); };

  /// Projection-based retraction operator for S^2
  Optimization::Riemannian::Retraction<Vector, Vector, Vector> retract =
      [](const Vector &X, const Vector &V, const Vector &P) {
        return (X + V).normalized();
      };

  /// Set initial point

  // X0 will be a point on the equator
  Vector X0 = {-0.5, -0.5, -0.707107};

  /// Run gradient descent optimizer

  // Set gradient descent options
  Optimization::Riemannian::GradientDescentParams<Scalar> params;
  params.gradient_tolerance = 1e-6;
  params.relative_decrease_tolerance = 0;
  params.stepsize_tolerance = 0;
  params.max_iterations = 1000000;

  Optimization::Riemannian::GradientDescentResult<Vector, Scalar> result =
      Optimization::Riemannian::GradientDescent<Vector, Vector, Scalar, Vector>(
          F, grad_F, metric, retract, X0, P, params);

  // Check function value
  EXPECT_NEAR(result.f, 0, GD_test_error_threshold);

  // Check gradient value
  EXPECT_NEAR(result.gradfx_norm, 0, GD_test_error_threshold);

  // Check final solution
  EXPECT_NEAR((result.x - P).norm(), 0, GD_test_error_threshold);
}
