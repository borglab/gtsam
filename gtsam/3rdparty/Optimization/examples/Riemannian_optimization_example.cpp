/** This simple example demonstrates how to use Riemannian gradient descent and
 * truncated-Newton trust-region (TNT) algorithms to solve a simple optimization
 * problem over the sphere S^2 in R^3. */

#include <Eigen/Dense>

#include <fstream>
#include <time.h>

#include "Optimization/Riemannian/GradientDescent.h"
#include "Optimization/Riemannian/TNT.h"

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix;

using namespace std;
using namespace Optimization;
using namespace Riemannian;
int main() {

  /// SET UP OPTIMIZATION

  ///  We will minimize the function f(X; P) := || X - P ||^2, where P in S^2 is
  ///  a fixed point on the sphere
  ///
  /// In the example that follows, we will treat the point P as a fixed (i.e.
  /// unoptimized) parameter of the objective f, and pass this to the TNT
  /// optimization library as a user-supplied optional argument.

  Vector P = {0.0, 0.0, 1.0}; // P is the north pole

  /// SET UP FUNCTION HANDLES

  /// Utility function: projection of the vector V \in R^3 onto the tangent
  /// space T_X(S^2) of S^2 at the point
  auto project = [](const Vector &X, const Vector &V) -> Vector {
    return V - X.dot(V) * X;
  };

  /// Objective
  Objective<Vector, Scalar, Vector> F = [](const Vector &X, const Vector &P) {
    return (X - P).squaredNorm();
  };

  /// Gradient
  VectorField<Vector, Vector, Vector> grad_F = [&project](const Vector &X,
                                                          const Vector &P) {
    // Euclidean gradient
    Vector nabla_f = 2 * (X - P);

    // Compute Riemannian gradient from Euclidean one
    return project(X, nabla_f);
  };

  /// Riemannian Hessian constructor: Returns the Riemannian Hessian operator
  /// H(X): T_X(S^2) -> T_X(S^2) at X
  LinearOperatorConstructor<Vector, Vector, Vector> HC =
      [&project, &grad_F](const Vector &X, Vector &P) {
        // Euclidean Hessian matrix
        Matrix EucHess = 2 * Matrix::Identity();

        // Return Riemannian Hessian-vector product operator using the
        // Euclidean Hessian
        LinearOperator<Vector, Vector, Vector> Hessian =
            [&project, EucHess, &grad_F](const Vector &X, const Vector &Xdot,
                                         Vector &P) -> Vector {
          return project(X, EucHess * Xdot) - X.dot(grad_F(X, P)) * Xdot;
        };

        return Hessian;
      };

  /// Riemannian metric on S^2: this is just the usual inner-product on R^3
  RiemannianMetric<Vector, Vector, Scalar, Vector> metric =
      [](const Vector &X, const Vector &V1, const Vector &V2, const Vector &P) {
        return V1.dot(V2);
      };

  /// Projection-based retraction operator for S^2
  Retraction<Vector, Vector, Vector> retract =
      [](const Vector &X, const Vector &V, const Vector &P) {
        return (X + V).normalized();
      };

  /// SET INITIAL POINT

  // X0 will be a point on the equator
  Vector X0 = {-0.5, -0.5, -0.707107};

  cout << "Target point: P = " << endl << P << endl << endl;
  cout << "X0 = " << endl << X0 << endl << endl;

  cout << "Initial distance between X0 and P: " << (X0 - P).norm() << endl
       << endl;

  /// RUN GRADIENT DESCENT OPTIMIZER!

  cout << "RUNNING GRADIENT DESCENT OPTIMIZER!" << endl << endl;

  // Set gradient descent options
  GradientDescentParams<Scalar> gd_params;
  gd_params.max_iterations = 1000;
  gd_params.verbose = true;

  GradientDescentResult<Vector, Scalar> gd_result =
      GradientDescent<Vector, Vector, Scalar, Vector>(
          F, grad_F, metric, retract, X0, P, gd_params);

  cout << "Gradient descent estimate x_final:  " << endl
       << gd_result.x << endl
       << endl;
  cout << "Distance between P and x_final: " << (P - gd_result.x).norm() << endl
       << endl;

  /// RUN TNT OPTIMIZER!

  cout << "RUNNING TNT OPTIMIZER!" << endl << endl;

  // Set TNT options
  TNTParams<Scalar> tnt_params;
  tnt_params.verbose = true;

  TNTResult<Vector, Scalar> tnt_result = TNT<Vector, Vector, Scalar, Vector>(
      F, grad_F, HC, metric, retract, X0, P, std::nullopt, tnt_params);

  cout << "Truncated-Newton trust-region estimate x_final:  " << endl
       << tnt_result.x << endl
       << endl;
  cout << "Distance between P and x_final: " << (P - tnt_result.x).norm()
       << endl
       << endl;
}
