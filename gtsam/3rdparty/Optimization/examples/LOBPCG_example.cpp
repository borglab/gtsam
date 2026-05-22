/** This simple example demonstrates the use of the LOBPCG method to recover a
 * few of the algebraically-smallest eigenpairs of the eigenvalue problem:
 *
 * Ax = lambda*x
 *
 */

#include "Optimization/LinearAlgebra/LOBPCG.h"

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Sparse>

// Typedef for the numerical type will use in the following tests
typedef double Scalar;

// Typedefs for Eigen matrix types
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
typedef Eigen::SparseMatrix<Scalar, Eigen::ColMajor> SparseMatrix;
typedef Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> DiagonalMatrix;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix;
typedef Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>
    LinearOperator;
typedef Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix> StopFun;

using namespace std;

int main() {

  /// Test configuration

  // Dimension of linear operators
  size_t m = 500;

  // Block size
  size_t nx = 10;

  // Number of desired eigenvalues k
  size_t nev = 5;

  /// Termination criteria
  size_t max_iters = 3 * m;
  double tau = 1e-6;

  /// Test operators

  /// Construct linear operator A as a diagonal operator with a difficult
  /// spectrum
  Vector Lambda = Vector::LinSpaced(m, -.5 * m, .5 * m);

  LinearOperator A = [&Lambda](const Matrix &X) -> Matrix {
    return Lambda.asDiagonal() * X;
  };

  cout << "Requested spectrum of operator A: " << Lambda.head(nev).transpose()
       << endl
       << endl;

  /// Run LOBPCG

  cout << "Running LOBPCG ... " << endl;

  // Pass in a user function to track the evolution of
  // - Eigenvalue estimates
  // - Eigenpair residuals
  // - Number of converged eigenpairs

  std::vector<Vector> Thetas;
  std::vector<Vector> residuals;
  std::vector<size_t> ncvs;

  Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix> user_fun =
      [&Thetas, &residuals, &ncvs](
          size_t i,
          const Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix> &A,
          const std::optional<
              Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>> &B,
          const std::optional<
              Optimization::LinearAlgebra::SymmetricLinearOperator<Matrix>> &T,
          size_t nev, const Vector &Theta, const Matrix &X, const Vector &r,
          size_t nc) -> bool {
    // Record eigenvector estimates
    Thetas.emplace_back(Theta);

    // Record residuals
    residuals.emplace_back(r);

    // Record number of converged eigenpairs
    ncvs.emplace_back(nc);

    return false;
  };

  Vector Theta;
  Matrix X;
  size_t num_iters;
  size_t num_converged;
  std::tie(Theta, X) = Optimization::LinearAlgebra::LOBPCG<Vector, Matrix>(
      A, std::optional<LinearOperator>(nullopt),
      std::optional<LinearOperator>(nullopt), m, nx, nev, max_iters, num_iters,
      num_converged, tau,
      std::optional<
          Optimization::LinearAlgebra::LOBPCGUserFunction<Vector, Matrix>>(
          user_fun));

  cout << "LOBPCG terminated after " << num_iters << " iterations" << endl;
  cout << "Returned eigenvalue estimates: " << Theta.transpose() << endl;

  /// Calculate residuals of returned eigenvector estimates

  Matrix R = A(X) - X * Theta.asDiagonal();
  Vector rnorms = R.colwise().norm();
  Vector xnorms = X.colwise().norm();

  cout << "Residuals of returned vectors:  " << rnorms.transpose() << endl;

  /// Record state traces

  // Record sequence of eigenvalue estimates
  std::string eigs_filename = "eigenvalues.txt";
  ofstream eigs_file(eigs_filename);
  for (const auto l : Thetas)
    eigs_file << l.transpose() << std::endl;
  eigs_file.close();

  // Record sequence of residuals
  std::string residuals_filename = "residuals.txt";
  ofstream residuals_file(residuals_filename);
  for (const auto r : residuals)
    residuals_file << r.transpose() << std::endl;
  residuals_file.close();

  // Record sequence of number of converged eigenpairs
  std::string ncvs_filename = "ncvs.txt";
  ofstream ncvs_file(ncvs_filename);
  for (const auto c : ncvs)
    ncvs_file << c << " ";
  ncvs_file << std::endl;
  ncvs_file.close();
}
