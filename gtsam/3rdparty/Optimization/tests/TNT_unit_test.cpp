/** This unit test exercises the functionality of the Riemannian
 * truncated-Newton trust-region method */

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "Optimization/Riemannian/TNT.h"

class TNTUnitTest : public testing::Test {
protected:
  // Typedef for the numerical type will use in the following tests
  typedef double Scalar;

  // Typedefs for Eigen matrix types
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

  // Threshold for considering two values to be numerically equal in the
  // following tests
  Scalar eps_abs = 1e-8;

  // Boolean variable controlling whether to print output -- useful for quickly
  // checking failed tests
  bool verbose = false;

  /// We will test the Riemannian truncated-Newton trust-region method by
  /// minimizing a simple quadratic function over the sphere

  ///  We will minimize the function f(X; P) := | X - P |^2, where P in S^2 is
  ///  a fixed point on the sphere
  ///
  /// In the example that follows, we will treat the point P as a fixed (i.e.
  /// unoptimized) parameter of the objective f, and pass this to the TNT
  /// optimization library as a user-supplied optional argument.

  // Target point P on S^2
  Vector P;

  // Initial point on S^2
  Vector X0;

  /// Objective function
  Optimization::Objective<Vector, Scalar, Vector> F;

  /// Gradient function
  Optimization::Riemannian::VectorField<Vector, Vector, Vector> gradF;

  /// Riemannian Hessian constructor: Returns the Riemannian Hessian operator
  /// H(X): T_X(S^2) -> T_X(S^2) at X
  Optimization::Riemannian::LinearOperatorConstructor<Vector, Vector, Vector>
      HessCon;

  /// Riemannian metric on S^2: this is just the usual inner-product on R^3
  Optimization::Riemannian::RiemannianMetric<Vector, Vector, Scalar, Vector>
      metric;

  /// Projection-based retraction operator for S^2
  Optimization::Riemannian::Retraction<Vector, Vector, Vector> retract;

  /// Preconditioning operator
  Optimization::Riemannian::LinearOperator<Vector, Vector, Vector> precon;

  virtual void SetUp() override {

    // Set P: the north pole
    P.resize(3);
    P << 0.0, 0.0, 1.0;

    /// Set up function handles

    /// Utility function: projection of the vector V \in R^3 onto the tangent
    /// space T_X(S^2) of S^2 at the point
    auto project = [](const Vector &X, const Vector &V) -> Vector {
      return V - X.dot(V) * X;
    };

    F = [](const Vector &X, const Vector &P) { return (X - P).squaredNorm(); };

    gradF = [project](const Vector &X, const Vector &P) {
      // Euclidean gradient
      Vector nabla_f = 2 * (X - P);

      // Compute Riemannian gradient from Euclidean one
      return project(X, nabla_f);
    };

    HessCon = [project, this](const Vector &X, Vector &P) {
      // Euclidean Hessian matrix
      Matrix EucHess = 2 * Matrix::Identity(3, 3);

      // Return Riemannian Hessian-vector product operator using the
      // Euclidean Hessian
      Optimization::Riemannian::LinearOperator<Vector, Vector, Vector> Hessian =
          [project, EucHess, this](const Vector &X, const Vector &Xdot,
                                   Vector &P) -> Vector {
        return project(X, EucHess * Xdot) - X.dot(this->gradF(X, P)) * Xdot;
      };

      return Hessian;
    };

    metric = [](const Vector &X, const Vector &V1, const Vector &V2,
                const Vector &P) { return V1.dot(V2); };

    /// Projection-based retraction operator for S^2
    retract = [](const Vector &X, const Vector &V, const Vector &P) {
      return (X + V).normalized();
    };

    /// Preconditioning operator: just return a simple diagonal scaling matrix
    precon = [](const Vector &X, const Vector &V, const Vector &P) -> Vector {
      Vector D(3);
      D << 1.0, 2.0, 3.0;

      // Simple scaling: return 2 * V
      return D.asDiagonal() * V;
    };

    /// Set initial point
    X0.resize(3);
    X0 << -0.5, -0.5, -0.707107;
  }
};

/// Test optimization over the sphere
TEST_F(TNTUnitTest, RiemannianTNTSphere) {

  /// Run TNT optimizer
  // Set TNT options
  Optimization::Riemannian::TNTParams<Scalar> tnt_params;
  tnt_params.relative_decrease_tolerance = 0;
  tnt_params.stepsize_tolerance = 0;
  tnt_params.preconditioned_gradient_tolerance = 0;
  tnt_params.stepsize_tolerance = 0;
  tnt_params.gradient_tolerance = eps_abs;
  tnt_params.verbose = verbose;

  Optimization::Riemannian::TNTResult<Vector, Scalar> result =
      Optimization::Riemannian::TNT<Vector, Vector, Scalar, Vector>(
          F, gradF, HessCon, metric, retract, X0, P, std::nullopt, tnt_params);

  /// Extract solution
  const Vector &X = result.x;

  /// Verify that the method reports termination at a first-order critical point
  EXPECT_EQ(result.status, Optimization::Riemannian::TNTStatus::Gradient);

  /// Verify that the gradient norm at the returned solution satisfies the
  /// required tolerance
  EXPECT_LT(gradF(X, P).norm(), eps_abs);

  /// Verify that the value of the objective at the returned solution is
  /// *strictly smaller* than the initial point
  EXPECT_LT(F(X, P), F(X0, P));
}

/// Test optimization over the sphere, using a simple (test) preconditioner
TEST_F(TNTUnitTest, RiemannianTNTSphereWithPrecon) {

  /// Run TNT optimizer
  // Set TNT options
  Optimization::Riemannian::TNTParams<Scalar> tnt_params;
  tnt_params.relative_decrease_tolerance = 0;
  tnt_params.stepsize_tolerance = 0;
  tnt_params.preconditioned_gradient_tolerance = 0;
  tnt_params.stepsize_tolerance = 0;
  tnt_params.gradient_tolerance = eps_abs;
  tnt_params.verbose = verbose;

  Optimization::Riemannian::TNTResult<Vector, Scalar> result =
      Optimization::Riemannian::TNT<Vector, Vector, Scalar, Vector>(
          F, gradF, HessCon, metric, retract, X0, P, precon, tnt_params);

  /// Extract solution
  const Vector &X = result.x;

  /// Verify that the method reports termination at a first-order critical point
  EXPECT_EQ(result.status, Optimization::Riemannian::TNTStatus::Gradient);

  /// Verify that the gradient norm at the returned solution satisfies the
  /// required tolerance
  EXPECT_LT(gradF(X, P).norm(), eps_abs);

  /// Verify that the value of the objective at the returned solution is
  /// *strictly smaller* than the initial point
  EXPECT_LT(F(X, P), F(X0, P));
}
