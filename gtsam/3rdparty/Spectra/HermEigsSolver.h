// Copyright (C) 2024-2025 Yixuan Qiu <yixuan.qiu@cos.name>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SPECTRA_HERM_EIGS_SOLVER_H
#define SPECTRA_HERM_EIGS_SOLVER_H

#include <Eigen/Core>

#include "HermEigsBase.h"
#include "Util/SelectionRule.h"
#include "MatOp/DenseHermMatProd.h"

namespace Spectra {

///
/// \ingroup EigenSolver
///
/// This class implements the eigen solver for Hermitian matrices, i.e.,
/// to solve \f$Ax=\lambda x\f$ where \f$A\f$ is Hermitian.
/// An Hermitian matrix is a complex square matrix that is equal to its
/// own conjugate transpose. It is known that all Hermitian matrices have
/// real-valued eigenvalues.
///
/// \tparam OpType  The name of the matrix operation class. Users could either
///                 use the wrapper classes such as DenseHermMatProd and
///                 SparseHermMatProd, or define their own that implements the type
///                 definition `Scalar` and all the public member functions as in
///                 DenseHermMatProd.
///
/// Below is an example that demonstrates the usage of this class.
///
/// \code{.cpp}
/// #include <Eigen/Core>
/// #include <Spectra/HermEigsSolver.h>
/// // <Spectra/MatOp/DenseHermMatProd.h> is implicitly included
/// #include <iostream>
///
/// using namespace Spectra;
///
/// int main()
/// {
///     // We are going to calculate the eigenvalues of M
///     Eigen::MatrixXcd A = Eigen::MatrixXcd::Random(10, 10);
///     Eigen::MatrixXcd M = A + A.adjoint();
///
///     // Construct matrix operation object using the wrapper class DenseHermMatProd
///     using OpType = DenseHermMatProd<std::complex<double>>;
///     OpType op(M);
///
///     // Construct eigen solver object, requesting the largest three eigenvalues
///     HermEigsSolver<OpType> eigs(op, 3, 6);
///
///     // Initialize and compute
///     eigs.init();
///     int nconv = eigs.compute(SortRule::LargestAlge);
///
///     // Retrieve results
///     // Eigenvalues are real-valued, and eigenvectors are complex-valued
///     Eigen::VectorXd evalues;
///     if (eigs.info() == CompInfo::Successful)
///         evalues = eigs.eigenvalues();
///
///     std::cout << "Eigenvalues found:\n" << evalues << std::endl;
///
///     return 0;
/// }
/// \endcode
///
/// And here is an example for user-supplied matrix operation class.
///
/// \code{.cpp}
/// #include <Eigen/Core>
/// #include <Spectra/HermEigsSolver.h>
/// #include <iostream>
///
/// using namespace Spectra;
///
/// // M = diag(1+0i, 2+0i, ..., 10+0i)
/// class MyDiagonalTen
/// {
/// public:
///     using Scalar = std::complex<double>;  // A typedef named "Scalar" is required
///     int rows() const { return 10; }
///     int cols() const { return 10; }
///     // y_out = M * x_in
///     void perform_op(Scalar *x_in, Scalar *y_out) const
///     {
///         for (int i = 0; i < rows(); i++)
///         {
///             y_out[i] = x_in[i] * Scalar(i + 1, 0);
///         }
///     }
/// };
///
/// int main()
/// {
///     MyDiagonalTen op;
///     HermEigsSolver<MyDiagonalTen> eigs(op, 3, 6);
///     eigs.init();
///     eigs.compute(SortRule::LargestAlge);
///     if (eigs.info() == CompInfo::Successful)
///     {
///         Eigen::VectorXd evalues = eigs.eigenvalues();
///         // Will get (10, 9, 8)
///         std::cout << "Eigenvalues found:\n" << evalues << std::endl;
///     }
///
///     return 0;
/// }
/// \endcode
///
template <typename OpType = DenseHermMatProd<double>>
class HermEigsSolver : public HermEigsBase<OpType, IdentityBOp>
{
private:
    using Index = Eigen::Index;

public:
    ///
    /// Constructor to create a solver object.
    ///
    /// \param op   The matrix operation object that implements
    ///             the matrix-vector multiplication operation of \f$A\f$:
    ///             calculating \f$Av\f$ for any vector \f$v\f$. Users could either
    ///             create the object from the wrapper class such as DenseHermMatProd, or
    ///             define their own that implements all the public members
    ///             as in DenseHermMatProd.
    /// \param nev  Number of eigenvalues requested. This should satisfy \f$1\le nev \le n-1\f$,
    ///             where \f$n\f$ is the size of matrix.
    /// \param ncv  Parameter that controls the convergence speed of the algorithm.
    ///             Typically a larger `ncv` means faster convergence, but it may
    ///             also result in greater memory use and more matrix operations
    ///             in each iteration. This parameter must satisfy \f$nev < ncv \le n\f$,
    ///             and is advised to take \f$ncv \ge 2\cdot nev\f$.
    ///
    HermEigsSolver(OpType& op, Index nev, Index ncv) :
        HermEigsBase<OpType, IdentityBOp>(op, IdentityBOp(), nev, ncv)
    {}
};

}  // namespace Spectra

#endif  // SPECTRA_HERM_EIGS_SOLVER_H
