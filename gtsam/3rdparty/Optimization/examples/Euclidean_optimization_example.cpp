/** This simple example demonstrates how to use the simplified Euclidean
 * interfaces for the gradient descent and truncated Newton trust-region (TNT)
 * algorithms to minimize the Rosenbrock function:
 *
 * f(x,y) = (a - x)^2 + b(y - x^2)^2
 *
 * whose (global) minimizer is f(a,a^2) = 0
 */

#include "Optimization/Riemannian/GradientDescent.h"
#include "Optimization/Riemannian/TNT.h"

#include <Eigen/Dense>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 2, 1> Vector;
typedef Eigen::Matrix<Scalar, 2, 2> Matrix;

using namespace std;
using namespace Optimization;
using namespace Riemannian;

int main() {

  /// SET UP OPTIMIZATION PROBLEM

  // Rosenbrock function parameters
  Scalar a = 1;
  Scalar b = 100;

  // Objective: f(x,y) = (a - x)^2  + b(y - x^2)^2
  Objective<Vector, Scalar> F = [a, b](const Vector &x) {
    return std::pow(a - x(0), 2) + b * std::pow((x(1) - std::pow(x(0), 2)), 2);
  };

  // Euclidean gradient operator: returns the Euclidean gradient nablaF(X) at
  // each X df/dx = -2(a-x) - 4bx(y-x^2) df / dy = 2b(y-x^2)
  EuclideanVectorField<Vector> nabla_F = [a, b](const Vector &x) {
    Vector df;
    df(0) = -2 * (a - x(0)) - 4 * b * x(0) * (x(1) - std::pow(x(0), 2));
    df(1) = 2 * b * (x(1) - std::pow(x(0), 2));
    return df;
  };

  // Euclidean Hessian constructor: Returns the Hessian operator H(X) at X
  // H(X) = [2 - 4by + 12bx^2   -4bx
  //      -4bx               2b]
  EuclideanLinearOperatorConstructor<Vector> HC = [a, b](const Vector &x) {
    // Compute Euclidean Hessian at X
    Matrix H;
    H(0, 0) = 2 - 4 * b * x(1) + 12 * b * std::pow(x(0), 2);
    H(0, 1) = -4 * b * x(0);
    H(1, 0) = H(0, 1);
    H(1, 1) = 2 * b;

    // Construct and return Euclidean Hessian operator: this is a function that
    // accepts as input a point X and tangent vector V, and returns H(X)[V], the
    // value of the Hessian operator at X applied to V
    EuclideanLinearOperator<Vector> Hess =
        [H](const Vector &x, const Vector &v) { return H * v; };

    return Hess;
  };

  /// SAMPLE INITIAL POINT

  Vector x0(.1, .1);
  Vector x_min(a, a * a);

  /// RUN GRADIENT DESCENT OPTIMIZER!
  cout << "Minimizing Rosenbrock function with parameters a = " << a
       << ", b = " << b << " using gradient descent ... " << endl
       << endl;
  GradientDescentParams<Scalar> gd_params;
  gd_params.max_iterations = 1000;
  gd_params.verbose = true;

  GradientDescentResult<Vector, Scalar> gd_result =
      EuclideanGradientDescent<Vector, Scalar>(F, nabla_F, x0, gd_params);

  cout << "Final objective value (should be 0): = " << gd_result.f << endl
       << endl;

  cout << "Estimated minimizer: " << endl << gd_result.x << endl << endl;
  cout << "Global minimizer: " << endl << x_min << endl << endl;
  cout << "Error in minimizer estimate: " << (gd_result.x - x_min).norm()
       << endl
       << endl;

  /// RUN TNT OPTIMIZER!

  cout << "Minimizing Rosenbrock function with parameters a = " << a
       << ", b = " << b << " using truncated-Newton trust-region optimizer ... "
       << endl
       << endl;
  // Set TNT options
  Optimization::Riemannian::TNTParams<Scalar> tnt_params;
  tnt_params.verbose = true;

  TNTResult<Vector, Scalar> tnt_result = EuclideanTNT<Vector, Scalar>(
      F, nabla_F, HC, x0, std::optional<EuclideanLinearOperator<Vector>>(),
      tnt_params);

  cout << "Final objective value (should be 0): = " << tnt_result.f << endl
       << endl;

  cout << "Estimated minimizer: " << endl << tnt_result.x << endl << endl;
  cout << "Global minimizer: " << endl << x_min << endl << endl;
  cout << "Error in minimizer estimate: " << (tnt_result.x - x_min).norm()
       << endl
       << endl;
}
