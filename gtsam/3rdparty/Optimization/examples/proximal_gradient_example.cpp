#include "Optimization/Convex/ProximalGradient.h"

#include <Eigen/Dense>
#include <fstream>

using namespace std;
using namespace Optimization;
using namespace Optimization::Convex;
int main() {
  /// SET UP TEST FUNCTION HANDLES

  /** We solve a group-sparse lasso problem of the form
   *
   * min || A *  x - b ||_2^2 + mu * || x ||_2
   *
   */

  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, 2, 1> Variable;
  Eigen::Matrix<Scalar, 2, 2> A;
  A << 1000, 0, 0, 1.0;
  Variable b;
  b << 1.0, 1.0;
  Scalar mu = 10;

  InnerProduct<Variable, Scalar> inner_product =
      [&](const Variable &v1, const Variable &v2) { return v1.dot(v2); };

  // Define f and grad_f
  Objective<Variable, Scalar> f = [&](const Variable &x) {
    return (A * x - b).squaredNorm();
  };

  GradientOperator<Variable> grad_f = [&](const Variable &x) {
    return 2 * A.transpose() * (A * x - b);
  };

  // Define g and prox_{lambda g}
  Objective<Variable, Scalar> g = [&](const Variable &x) {
    return mu * x.norm();
  };

  ProximalOperator<Variable, Scalar> prox_g = [&](const Variable &x,
                                                  Scalar lambda) {
    /** The proximal operator for the weighted L2 norm is the block-soft
     * thresholding operator:
     *
     * (1 - mu*lambda / |x|)_{+} * x
     *
     * (cf. Section 6.5.4 of Parikh and Boyd's "Proximal Algorithms"
     */

    return std::max<Scalar>(1 - mu * lambda / std::sqrt(x.dot(x)), 0.0) * x;
  };

  /// INITIALIZATION
  Variable x0;
  x0 << 4.0, 4.0;

  cout << "Solving group LASSO problem: " << endl
       << endl
       << "min |Ax - b|_2^2 + mu * |x|_2" << endl
       << endl
       << "with A = " << endl
       << A << endl
       << endl
       << "b =  " << endl
       << b << endl
       << "and mu = " << mu << endl
       << endl;

  cout << "Initial value of x = " << endl << x0 << endl << endl;

  /// OPTIMIZE!

  /// Set optimization algorithm parameters
  ProximalGradientParams<Scalar> params;
  params.verbose = true;
  params.max_iterations = 1000000;
  // params.adaptive_restart = false;
  params.mode = ProximalGradientMode::ACCELERATED;
  params.composite_gradient_tolerance = 1e-4;

  cout << "Optimizing!" << endl << endl;
  ProximalGradientResult<Variable, Scalar> result =
      ProximalGradient<Variable, Scalar>(f, grad_f, g, prox_g, inner_product,
                                         x0, params);

  cout << "Final result:" << endl;
  cout << "F(x) = " << result.f << endl;
  cout << "x = " << endl << result.x << endl << endl;
}
