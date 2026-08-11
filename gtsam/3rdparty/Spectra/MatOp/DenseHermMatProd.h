// Copyright (C) 2024-2025 Yixuan Qiu <yixuan.qiu@cos.name>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SPECTRA_DENSE_HERM_MAT_PROD_H
#define SPECTRA_DENSE_HERM_MAT_PROD_H

#include <Eigen/Core>

namespace Spectra {

///
/// \ingroup MatOp
///
/// This class defines the matrix-vector multiplication operation on a
/// Hermitian complex matrix \f$A\f$, i.e., calculating \f$y=Ax\f$ for any vector
/// \f$x\f$. It is mainly used in the HermEigsSolver eigen solver.
///
/// \tparam Scalar_ The element type of the matrix, for example,
///                 `std::complex<float>`, `std::complex<double>`,
///                 and `std::complex<long double>`.
/// \tparam Uplo    Either `Eigen::Lower` or `Eigen::Upper`, indicating which
///                 triangular part of the matrix is used.
/// \tparam Flags   Either `Eigen::ColMajor` or `Eigen::RowMajor`, indicating
///                 the storage format of the input matrix.
///
template <typename Scalar_, int Uplo = Eigen::Lower, int Flags = Eigen::ColMajor>
class DenseHermMatProd
{
public:
    ///
    /// Element type of the matrix.
    ///
    using Scalar = Scalar_;

private:
    using Index = Eigen::Index;
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Flags>;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MapConstVec = Eigen::Map<const Vector>;
    using MapVec = Eigen::Map<Vector>;
    using ConstGenericMatrix = const Eigen::Ref<const Matrix>;

    ConstGenericMatrix m_mat;

public:
    ///
    /// Constructor to create the matrix operation object.
    ///
    /// \param mat An **Eigen** matrix object, whose type can be
    /// `Eigen::Matrix<Scalar, ...>` (e.g. `Eigen::MatrixXcd` and
    /// `Eigen::MatrixXcf`), or its mapped version
    /// (e.g. `Eigen::Map<Eigen::MatrixXcd>`).
    ///
    template <typename Derived>
    DenseHermMatProd(const Eigen::MatrixBase<Derived>& mat) :
        m_mat(mat)
    {
        static_assert(
            static_cast<int>(Derived::PlainObject::IsRowMajor) == static_cast<int>(Matrix::IsRowMajor),
            "DenseHermMatProd: the \"Flags\" template parameter does not match the input matrix (Eigen::ColMajor/Eigen::RowMajor)");
    }

    ///
    /// Return the number of rows of the underlying matrix.
    ///
    Index rows() const { return m_mat.rows(); }
    ///
    /// Return the number of columns of the underlying matrix.
    ///
    Index cols() const { return m_mat.cols(); }

    ///
    /// Perform the matrix-vector multiplication operation \f$y=Ax\f$.
    ///
    /// \param x_in  Pointer to the \f$x\f$ vector.
    /// \param y_out Pointer to the \f$y\f$ vector.
    ///
    // y_out = A * x_in
    void perform_op(const Scalar* x_in, Scalar* y_out) const
    {
        MapConstVec x(x_in, m_mat.cols());
        MapVec y(y_out, m_mat.rows());
        y.noalias() = m_mat.template selfadjointView<Uplo>() * x;
    }
};

}  // namespace Spectra

#endif  // SPECTRA_DENSE_HERM_MAT_PROD_H
