#include "Optimization/Convex/ADMM.h"

#include <Eigen/Cholesky>
#include <Eigen/Dense>

#include <fstream>
#include <random> // To get random number generation

using namespace Optimization;
using namespace Optimization::Convex;
using namespace std;

bool write_output = false;

int main() {
  /// Here we solve the small dense LASSO regression problem:
  ///
  /// min_x (1/2) |Ax - b|_2^2 + mu * |x|_1
  ///
  /// described in Section 11.1 of "Distributed Optimization and Statistical
  /// Learning via the Alternating Direction Method of Multipliers".
  ///
  /// We solve this problem using ADMM by addressing the following (equivalent)
  /// formulation:
  ///
  /// min_{x,y} (1/2) |Ax - b|_2^2 + mu * |y|_1
  ///
  /// s.t. x - y = 0
  ///
  /// with augmented Lagrangian:
  ///
  /// L_rho(x,y, lambda) = (1/2) |Ax-b|_2^2 + mu * |y|_1
  ///                       + lambda' * (x-y) + (rho/2) * |x-y|_2^2

  /// SETUP

  typedef double Scalar;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;

  size_t m = 1500;   // Dimensionality of observations
  size_t n = 5000;   // Dimensionality of latent vector x
  size_t nnz = 100;  // Number of nonzero elements of x
  Scalar sigma = .1; // Standard deviation of additive noise on measurements
  Scalar mu;         // Regularization parameter for Lasso

  // Set up some standard random number generators
  std::default_random_engine generator;

  // Standard normal distribution
  std::normal_distribution<Scalar> std_norm(0.0, 1.0);

  // Standard (integer-valued) uniform distribution on the interval [0, n)
  std::uniform_int_distribution<size_t> uniform(0, n);

  /// Construct matrix A

  cout << "Constructing sensing matrix A ... " << endl;
  Matrix A(m, n);

  // Sample elements of A iid from a standard Gaussian distribution
  for (size_t r = 0; r < m; ++r)
    for (size_t c = 0; c < n; ++c) {
      A(r, c) = std_norm(generator);
    }

  // Normalize the columns of A
  for (size_t c = 0; c < n; ++c)
    A.col(c) *= (1 / A.col(c).norm());

  /// Construct ground-truth vector x_true
  cout << "Constructing ground-truth vector x ... " << endl;
  Vector x_true = Vector::Zero(n);

  // Randomly set nnz elements of x_true to be nonzero, sampled from a standard
  // uniform gaussian
  for (size_t i = 0; i < nnz; ++i) {
    // Sample index of nonzero element
    size_t idx = uniform(generator);

    // Sample a random value for this element from the standard normal
    // distribution
    x_true(idx) = std_norm(generator);
  }

  /// Now generate a noisy observation of x_true via the sensing matrix A
  cout << "Generating noisy observation b ... " << endl;
  Vector b = A * x_true;

  for (size_t i = 0; i < m; ++i)
    b(i) += std_norm(generator);

  /// SET UP ADMM
  cout << endl;
  cout << "Setting up ADMM ... " << endl << endl;

  // ADMM params
  ADMMParams<Scalar> params;
  params.max_iterations = 250;
  params.verbose = true; // Turn on verbose output
  params.log_iterates = true;
  params.mode = ADMMMode::Simple;
  params.penalty_adaptation_mode = ADMMPenaltyAdaptation::None;
  params.penalty_adaptation_period = 1;

  // Set the stopping criteria for this example to match those used in
  // the reference paper
  params.eps_rel = 1e-4;
  params.eps_abs_pri = 1e-2;
  params.eps_abs_dual = 1e-2;

  /// Minimization over x:
  ///
  /// Since L_rho(x,y,lambda) is (convex) quadratic (hence differentiable) in x,
  /// we can find its minimizer as a function of x by setting its derivative to
  /// zero.  Observing that:
  ///
  /// dL_rho/dx = (A^T A + rho*I)x - A^T b + lambda - rho*y
  ///
  /// we compute the minimizing value of x as:
  ///
  /// x = (A^T A + rho * I) * (A^T b + rho * y - lambda)

  // Compute and cache matrix A^T A
  Matrix AtA = A.transpose() * A;

  // Compute and cache constant vector A^T b
  Vector Atb = A.transpose() * b;

  // Set regularization parameter to .1 |A^T * b|_{infty}
  mu = .1 * Atb.lpNorm<Eigen::Infinity>();

  // Construct x-minimization function, making use of the (cached)
  // coefficient matrix A and Cholesky factor L
  AugLagMinX<Vector, Vector, Vector, Scalar> minLx =
      [&](const Vector &y, const Vector &lambda, double rho) -> Vector {
    // Compute Cholseky factorization of A^T*A + rho * I
    Eigen::LLT<Matrix> L(AtA + rho * Matrix::Identity(n, n));

    return L.solve(Atb + rho * y - lambda);
  };

  /// Minimization over y:
  ///
  /// Ignoring terms that are constant in y, minimizing the augmented Lagrangian
  /// over y is equivalent to:
  ///
  /// min_y mu * |y|_1 + lambda' * (x-y) + (rho/2) * |x-y|_2^2 n
  ///
  /// Ignoring terms that are constant wrt y, and completing the square in order
  /// to absorb the linear term in y into the squared norm term, the above
  /// minimization is equivalent to:
  ///
  /// min_y mu * |y|_1 + (rho/2) * | y - (x + (1/rho) * lambda) |_2^2
  ///
  /// We recognize the above minimization problem as a specific case of the
  /// proximity operator for mu * |y|_1, whose solution is given by the
  /// soft-thresholding operator (cf. Section 4.4.3 of "Distributed Optimization
  /// and Statistical Learning via the Alternating Direction Method of
  /// Multipliers").

  // Define soft-thresholding operator for a vector v
  auto S = [](const Vector &v, Scalar kappa) -> Vector {
    Vector k = Vector::Constant(v.rows(), kappa);

    return (v - k).cwiseMax(0) - (-v - k).cwiseMax(0);
  };

  AugLagMinY<Vector, Vector, Vector> minLy =
      [&](const Vector &x, const Vector &lambda, double rho) -> Vector {
    return S(x + (1 / rho) * lambda, mu / rho);
  };

  /// Linear operator A in linear constraint
  LinearOperator<Vector, Vector> Aop = [](const Vector &x) { return x; };

  /// Linear operator B in linear constraint
  LinearOperator<Vector, Vector> Bop = [](const Vector &y) { return -y; };

  /// Inner product
  Optimization::Convex::InnerProduct<Vector> inner_product =
      [](const Vector &x, const Vector &y) -> double { return x.dot(y); };

  // All-0's vector of dimension n
  Vector Z = Vector::Zero(n);

  /// RUN ADMM!
  auto result = ADMM<Vector, Scalar>(minLx, minLy, Aop, Bop, Aop, inner_product,
                                     Z, Z, Z, params);

  /// PROCESS OUTPUT

  /// Compute minimum-norm subgradient of final solution
  Vector &xopt = std::get<0>(result.x);

  // First, compute gradient of smooth part
  Vector subgrad = A.transpose() * (A * xopt - b);

  for (size_t i = 0; i < subgrad.rows(); ++i) {
    if (fabs(xopt(i)) < 1e-4) {
      // If xopt_i ~ 0, pick a value from the range [-mu, mu] that will
      // minimize the absolute value of the resulting element in the subgradient
      if (fabs(subgrad(i)) < mu)
        subgrad(i) = 0;
      else
        subgrad(i) -= copysign(mu, subgrad(i));
    } else {
      // If xopt =/= 0, then the subgradient is simply sign(x) mu
      subgrad(i) += copysign(mu, xopt(i));
    }
  }

  double subgradient_norm = subgrad.norm();
  cout << "Norm of subgradient at solution: " << subgradient_norm << endl
       << endl;

  if (write_output) {
    // Define a lambda function that computes the objective of the LASSO
    // problem
    auto f = [&](const Vector &x) {
      return .5 * (A * x - b).squaredNorm() + mu * x.lpNorm<1>();
    };

    string primal_residuals_filename = "primal_residuals.txt";
    string dual_residuals_filename = "dual_residuals.txt";
    string objective_values_filename = "objective_values.txt";
    string penalty_values_filename = "penalty_values.txt";

    cout << endl
         << "Writing out primal residuals to file: "
         << primal_residuals_filename << " ... " << endl;
    ofstream primal_residuals_file(primal_residuals_filename);
    for (const auto &r : result.primal_residuals)
      primal_residuals_file << r << " ";
    primal_residuals_file.close();

    cout << "Writing out dual residuals to file: " << dual_residuals_filename
         << " ... " << endl;
    ofstream dual_residuals_file(dual_residuals_filename);
    for (const auto &d : result.dual_residuals)
      dual_residuals_file << d << " ";
    dual_residuals_file.close();

    cout << "Writing out objective values to file: "
         << objective_values_filename << " ... " << endl;
    ofstream objective_values_file(objective_values_filename);
    for (const auto &p : result.iterates)
      objective_values_file << f(std::get<0>(p)) << " ";
    objective_values_file.close();

    cout << "Writing out penalty values to file: " << penalty_values_filename
         << " ... " << endl;
    ofstream penalty_values_file(penalty_values_filename);
    for (const auto &rho : result.penalty_parameters)
      penalty_values_file << rho << " ";
    penalty_values_file.close();
  }
}
