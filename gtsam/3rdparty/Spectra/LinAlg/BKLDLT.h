// Copyright (C) 2019-2025 Yixuan Qiu <yixuan.qiu@cos.name>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SPECTRA_BK_LDLT_H
#define SPECTRA_BK_LDLT_H

#include <Eigen/Core>
#include <vector>
#include <stdexcept>
#include <type_traits>  // std::is_same

#include "../Util/CompInfo.h"

namespace Spectra {

// We need a generic conj() function for both real and complex values,
// and hope that conj(x) == x if x is real-valued. However, in STL,
// conj(x) == std::complex(x, 0) for such cases, meaning that the
// return value type is not necessarily the same as x. To avoid this
// inconvenience, we define a simple class that does this task
//
// Similarly, define a real(x) function that returns x itself if
// x is real-valued, and returns std::complex(x, 0) if x is complex-valued
template <typename Scalar>
struct ScalarOp
{
    static Scalar conj(const Scalar& x)
    {
        return x;
    }

    static Scalar real(const Scalar& x)
    {
        return x;
    }
};
// Specialization for complex values
template <typename RealScalar>
struct ScalarOp<std::complex<RealScalar>>
{
    static std::complex<RealScalar> conj(const std::complex<RealScalar>& x)
    {
        using std::conj;
        return conj(x);
    }

    static std::complex<RealScalar> real(const std::complex<RealScalar>& x)
    {
        return std::complex<RealScalar>(x.real(), RealScalar(0));
    }
};

// Bunch-Kaufman LDLT decomposition
// References:
// 1. Bunch, J. R., & Kaufman, L. (1977). Some stable methods for calculating inertia and solving symmetric linear systems.
//    Mathematics of computation, 31(137), 163-179.
// 2. Golub, G. H., & Van Loan, C. F. (2012). Matrix computations (Vol. 3). JHU press. Section 4.4.
// 3. Bunch-Parlett diagonal pivoting <http://oz.nthu.edu.tw/~d947207/Chap13_GE3.ppt>
// 4. Ashcraft, C., Grimes, R. G., & Lewis, J. G. (1998). Accurate symmetric indefinite linear equation solvers.
//    SIAM Journal on Matrix Analysis and Applications, 20(2), 513-561.
template <typename Scalar = double>
class BKLDLT
{
private:
    // The real part type of the matrix element
    using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
    using Index = Eigen::Index;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MapVec = Eigen::Map<Vector>;
    using MapConstVec = Eigen::Map<const Vector>;
    using IntVector = Eigen::Matrix<Index, Eigen::Dynamic, 1>;
    using GenericVector = Eigen::Ref<Vector>;
    using ConstGenericVector = const Eigen::Ref<const Vector>;

    Index m_n;
    Vector m_data;                                 // storage for a lower-triangular matrix
    std::vector<Scalar*> m_colptr;                 // pointers to columns
    IntVector m_perm;                              // [-2, -1, 3, 1, 4, 5]: 0 <-> 2, 1 <-> 1, 2 <-> 3, 3 <-> 1, 4 <-> 4, 5 <-> 5
    std::vector<std::pair<Index, Index>> m_permc;  // compressed version of m_perm: [(0, 2), (2, 3), (3, 1)]

    bool m_computed;
    CompInfo m_info;

    // Access to elements
    // Pointer to the k-th column
    Scalar* col_pointer(Index k) { return m_colptr[k]; }
    // A[i, j] -> m_colptr[j][i - j], i >= j
    Scalar& coeff(Index i, Index j) { return m_colptr[j][i - j]; }
    const Scalar& coeff(Index i, Index j) const { return m_colptr[j][i - j]; }
    // A[i, i] -> m_colptr[i][0]
    Scalar& diag_coeff(Index i) { return m_colptr[i][0]; }
    const Scalar& diag_coeff(Index i) const { return m_colptr[i][0]; }

    // Compute column pointers
    void compute_pointer()
    {
        m_colptr.clear();
        m_colptr.reserve(m_n);
        Scalar* head = m_data.data();

        for (Index i = 0; i < m_n; i++)
        {
            m_colptr.push_back(head);
            head += (m_n - i);
        }
    }

    // Copy mat - shift * I to m_data
    template <typename Derived>
    void copy_data(const Eigen::MatrixBase<Derived>& mat, int uplo, const RealScalar& shift)
    {
        // If mat is an expression, first evaluate it into a temporary object
        // This can be achieved by assigning mat to a const Eigen::Ref<const Matrix>&
        // If mat is a plain object, no temporary object is created
        const Eigen::Ref<const typename Derived::PlainObject>& src(mat);

        // Efficient copying for column-major matrices with lower triangular part
        if ((!Derived::PlainObject::IsRowMajor) && uplo == Eigen::Lower)
        {
            for (Index j = 0; j < m_n; j++)
            {
                const Scalar* begin = &src.coeffRef(j, j);
                const Index len = m_n - j;
                std::copy(begin, begin + len, col_pointer(j));
                diag_coeff(j) -= Scalar(shift);
            }
        }
        else
        {
            Scalar* dest = m_data.data();
            for (Index j = 0; j < m_n; j++)
            {
                for (Index i = j; i < m_n; i++, dest++)
                {
                    if (uplo == Eigen::Lower)
                        *dest = src.coeff(i, j);
                    else
                        *dest = ScalarOp<Scalar>::conj(src.coeff(j, i));
                }
                diag_coeff(j) -= Scalar(shift);
            }
        }
    }

    // Compute compressed permutations
    void compress_permutation()
    {
        for (Index i = 0; i < m_n; i++)
        {
            // Recover the permutation action
            const Index perm = (m_perm[i] >= 0) ? (m_perm[i]) : (-m_perm[i] - 1);
            if (perm != i)
                m_permc.push_back(std::make_pair(i, perm));
        }
    }

    // Working on the A[k:end, k:end] submatrix
    // Exchange k <-> r
    // Assume r >= k
    void pivoting_1x1(Index k, Index r)
    {
        m_perm[k] = r;

        // No permutation
        if (k == r)
            return;

        // A[k, k] <-> A[r, r]
        std::swap(diag_coeff(k), diag_coeff(r));

        // A[(r+1):end, k] <-> A[(r+1):end, r]
        std::swap_ranges(&coeff(r + 1, k), col_pointer(k + 1), &coeff(r + 1, r));

        // A[(k+1):(r-1), k] <-> A[r, (k+1):(r-1)]
        // Note: for Hermitian matrices, also need to do conjugate
        Scalar* src = &coeff(k + 1, k);
        if (std::is_same<Scalar, RealScalar>::value)
        {
            // Simple swapping for real values
            for (Index j = k + 1; j < r; j++, src++)
            {
                std::swap(*src, coeff(r, j));
            }
        }
        else
        {
            // For complex values
            for (Index j = k + 1; j < r; j++, src++)
            {
                const Scalar src_conj = ScalarOp<Scalar>::conj(*src);
                *src = ScalarOp<Scalar>::conj(coeff(r, j));
                coeff(r, j) = src_conj;
            }
        }

        // A[r, k] <- Conj(A[r, k])
        if (!std::is_same<Scalar, RealScalar>::value)
        {
            coeff(r, k) = ScalarOp<Scalar>::conj(coeff(r, k));
        }
    }

    // Working on the A[k:end, k:end] submatrix
    // Exchange [k+1, k] <-> [r, p]
    // Assume p >= k, r >= k+1
    void pivoting_2x2(Index k, Index r, Index p)
    {
        pivoting_1x1(k, p);
        pivoting_1x1(k + 1, r);

        // A[k+1, k] <-> A[r, k]
        std::swap(coeff(k + 1, k), coeff(r, k));

        // Use negative signs to indicate a 2x2 block
        // Also minus one to distinguish a negative zero from a positive zero
        m_perm[k] = -m_perm[k] - 1;
        m_perm[k + 1] = -m_perm[k + 1] - 1;
    }

    // A[r1, c1:c2] <-> A[r2, c1:c2]
    // Assume r2 >= r1 > c2 >= c1
    void interchange_rows(Index r1, Index r2, Index c1, Index c2)
    {
        if (r1 == r2)
            return;

        for (Index j = c1; j <= c2; j++)
        {
            std::swap(coeff(r1, j), coeff(r2, j));
        }
    }

    // lambda = |A[r, k]| = max{|A[k+1, k]|, ..., |A[end, k]|}
    // Largest (in magnitude) off-diagonal element in the first column of the current reduced matrix
    // r is the row index
    // Assume k < end
    RealScalar find_lambda(Index k, Index& r)
    {
        using std::abs;

        const Scalar* head = col_pointer(k);  // => A[k, k]
        const Scalar* end = col_pointer(k + 1);
        // Start with r=k+1, lambda=A[k+1, k]
        r = k + 1;
        RealScalar lambda = abs(head[1]);
        // Scan remaining elements
        for (const Scalar* ptr = head + 2; ptr < end; ptr++)
        {
            const RealScalar abs_elem = abs(*ptr);
            if (lambda < abs_elem)
            {
                lambda = abs_elem;
                r = k + (ptr - head);
            }
        }

        return lambda;
    }

    // sigma = |A[p, r]| = max {|A[k, r]|, ..., |A[end, r]|} \ {A[r, r]}
    // Largest (in magnitude) off-diagonal element in the r-th column of the current reduced matrix
    // p is the row index
    // Assume k < r < end
    RealScalar find_sigma(Index k, Index r, Index& p)
    {
        using std::abs;

        // First search A[r+1, r], ...,  A[end, r], which has the same task as find_lambda()
        // If r == end, we skip this search
        RealScalar sigma = RealScalar(-1);
        if (r < m_n - 1)
            sigma = find_lambda(r, p);

        // Then search A[k, r], ..., A[r-1, r], which maps to A[r, k], ..., A[r, r-1]
        for (Index j = k; j < r; j++)
        {
            const RealScalar abs_elem = abs(coeff(r, j));
            if (sigma < abs_elem)
            {
                sigma = abs_elem;
                p = j;
            }
        }

        return sigma;
    }

    // Generate permutations and apply to A
    // Return true if the resulting pivoting is 1x1, and false if 2x2
    bool permutate_mat(Index k, const RealScalar& alpha)
    {
        using std::abs;

        Index r = k, p = k;
        const RealScalar lambda = find_lambda(k, r);

        // If lambda=0, no need to interchange
        if (lambda > RealScalar(0))
        {
            const RealScalar abs_akk = abs(diag_coeff(k));
            // If |A[k, k]| >= alpha * lambda, no need to interchange
            if (abs_akk < alpha * lambda)
            {
                const RealScalar sigma = find_sigma(k, r, p);

                // If sigma * |A[k, k]| >= alpha * lambda^2, no need to interchange
                if (sigma * abs_akk < alpha * lambda * lambda)
                {
                    if (abs_akk >= alpha * sigma)
                    {
                        // Permutation on A
                        pivoting_1x1(k, r);

                        // Permutation on L
                        interchange_rows(k, r, 0, k - 1);
                        return true;
                    }
                    else
                    {
                        // There are two versions of permutation here
                        // 1. A[k+1, k] <-> A[r, k]
                        // 2. A[k+1, k] <-> A[r, p], where p >= k and r >= k+1
                        //
                        // Version 1 and 2 are used by Ref[1] and Ref[2], respectively

                        // Version 1 implementation
                        p = k;

                        // Version 2 implementation
                        // [r, p] and [p, r] are symmetric, but we need to make sure
                        // p >= k and r >= k+1, so it is safe to always make r > p
                        // One exception is when min{r,p} == k+1, in which case we make
                        // r = k+1, so that only one permutation needs to be performed
                        /* const Index rp_min = std::min(r, p);
                        const Index rp_max = std::max(r, p);
                        if (rp_min == k + 1)
                        {
                            r = rp_min; p = rp_max;
                        } else {
                            r = rp_max; p = rp_min;
                        } */

                        // Right now we use Version 1 since it reduces the overhead of interchange

                        // Permutation on A
                        pivoting_2x2(k, r, p);

                        // Permutation on L
                        interchange_rows(k, p, 0, k - 1);
                        interchange_rows(k + 1, r, 0, k - 1);
                        return false;
                    }
                }
            }
        }

        return true;
    }

    // E = [e11, e12]
    //     [e21, e22]
    // Overwrite E with inv(E)
    void inverse_inplace_2x2(Scalar& e11, Scalar& e21, Scalar& e22) const
    {
        // inv(E) = [d11, d12], d11 = e22/delta, d21 = -e21/delta, d22 = e11/delta
        //          [d21, d22]
        // delta = e11 * e22 - e12 * e21
        const Scalar e12 = ScalarOp<Scalar>::conj(e21);
        const Scalar delta = e11 * e22 - e12 * e21;
        std::swap(e11, e22);
        e11 /= delta;
        e22 /= delta;
        e21 = -e21 / delta;
    }

    // E = [e11, e12]
    //     [e21, e22]
    // Overwrite b with x = inv(E) * b, which is equivalent to solving E * x = b
    void solve_inplace_2x2(
        const Scalar& e11, const Scalar& e21, const Scalar& e22,
        Scalar& b1, Scalar& b2) const
    {
        using std::abs;

        const Scalar e12 = ScalarOp<Scalar>::conj(e21);
        const RealScalar e11_abs = abs(e11);
        const RealScalar e21_abs = abs(e21);
        // If |e11| >= |e21|, no need to exchange rows
        if (e11_abs >= e21_abs)
        {
            const Scalar fac = e21 / e11;
            const Scalar x2 = (b2 - fac * b1) / (e22 - fac * e12);
            const Scalar x1 = (b1 - e12 * x2) / e11;
            b1 = x1;
            b2 = x2;
        }
        else
        {
            // Exchange row 1 and row 2, so the system becomes
            // E* = [e21, e22], b* = [b2], x* = [x1]
            //      [e11, e12]       [b1]       [x2]
            const Scalar fac = e11 / e21;
            const Scalar x2 = (b1 - fac * b2) / (e12 - fac * e22);
            const Scalar x1 = (b2 - e22 * x2) / e21;
            b1 = x1;
            b2 = x2;
        }
    }

    // Compute C * inv(E), which is equivalent to solving X * E = C
    // X [n x 2], E [2 x 2], C [n x 2]
    // X = [x1, x2], E = [e11, e12], C = [c1 c2]
    //                   [e21, e22]
    void solve_left_2x2(
        const Scalar& e11, const Scalar& e21, const Scalar& e22,
        const MapVec& c1, const MapVec& c2,
        Eigen::Matrix<Scalar, Eigen::Dynamic, 2>& x) const
    {
        using std::abs;

        const Scalar e12 = ScalarOp<Scalar>::conj(e21);
        const RealScalar e11_abs = abs(e11);
        const RealScalar e12_abs = abs(e12);
        // If |e11| >= |e12|, no need to exchange rows
        if (e11_abs >= e12_abs)
        {
            const Scalar fac = e12 / e11;
            // const Scalar x2 = (c2 - fac * c1) / (e22 - fac * e21);
            // const Scalar x1 = (c1 - e21 * x2) / e11;
            x.col(1).array() = (c2 - fac * c1).array() / (e22 - fac * e21);
            x.col(0).array() = (c1 - e21 * x.col(1)).array() / e11;
        }
        else
        {
            // Exchange column 1 and column 2, so the system becomes
            // X* = [x1, x2], E* = [e12, e11], C* = [c2 c1]
            //                     [e22, e21]
            const Scalar fac = e11 / e12;
            // const Scalar x2 = (c1 - fac * c2) / (e21 - fac * e22);
            // const Scalar x1 = (c2 - e22 * x2) / e12;
            x.col(1).array() = (c1 - fac * c2).array() / (e21 - fac * e22);
            x.col(0).array() = (c2 - e22 * x.col(1)).array() / e12;
        }
    }

    // Return value is the status, CompInfo::Successful/NumericalIssue
    CompInfo gaussian_elimination_1x1(Index k)
    {
        // A[k, k] is known to be real-valued, so we force its imaginary
        // part to be zero when Scalar is a complex type
        // Interestingly, this has a significant effect on the accuracy
        // and numerical stability of the final solution
        const Scalar akk = ScalarOp<Scalar>::real(diag_coeff(k));
        diag_coeff(k) = akk;
        // Return CompInfo::NumericalIssue if not invertible
        if (akk == Scalar(0))
            return CompInfo::NumericalIssue;

        // [inverse]
        // diag_coeff(k) = Scalar(1) / akk;

        // B -= l * l^H / A[k, k], B := A[(k+1):end, (k+1):end], l := L[(k+1):end, k]
        Scalar* lptr = col_pointer(k) + 1;
        const Index ldim = m_n - k - 1;
        MapVec l(lptr, ldim);
        for (Index j = 0; j < ldim; j++)
        {
            Scalar l_conj = ScalarOp<Scalar>::conj(lptr[j]);
            MapVec(col_pointer(j + k + 1), ldim - j).noalias() -= (l_conj / akk) * l.tail(ldim - j);
        }

        // l /= A[k, k]
        l /= akk;

        return CompInfo::Successful;
    }

    // Return value is the status, CompInfo::Successful/NumericalIssue
    CompInfo gaussian_elimination_2x2(Index k)
    {
        Scalar& e11 = diag_coeff(k);
        Scalar& e21 = coeff(k + 1, k);
        Scalar& e22 = diag_coeff(k + 1);

        // A[k, k] and A[k+1, k+1] are known to be real-valued,
        // so we force their imaginary parts to be zero when Scalar
        // is a complex type
        // Interestingly, this has a significant effect on the accuracy
        // and numerical stability of the final solution
        e11 = ScalarOp<Scalar>::real(e11);
        e22 = ScalarOp<Scalar>::real(e22);
        Scalar e12 = ScalarOp<Scalar>::conj(e21);
        // Return CompInfo::NumericalIssue if not invertible
        if (e11 * e22 - e12 * e21 == Scalar(0))
            return CompInfo::NumericalIssue;

        // [inverse]
        // inverse_inplace_2x2(e11, e21, e22);

        // X = l * inv(E), l := L[(k+2):end, k:(k+1)]
        Scalar* l1ptr = &coeff(k + 2, k);
        Scalar* l2ptr = &coeff(k + 2, k + 1);
        const Index ldim = m_n - k - 2;
        MapVec l1(l1ptr, ldim), l2(l2ptr, ldim);

        Eigen::Matrix<Scalar, Eigen::Dynamic, 2> X(ldim, 2);
        // [inverse]
        // e12 = ScalarOp<Scalar>::conj(e21);
        // X.col(0).noalias() = l1 * e11 + l2 * e21;
        // X.col(1).noalias() = l1 * e12 + l2 * e22;
        // [solve]
        solve_left_2x2(e11, e21, e22, l1, l2, X);

        // B -= l * inv(E) * l^H = X * l^H, B = A[(k+2):end, (k+2):end]
        for (Index j = 0; j < ldim; j++)
        {
            const Scalar l1j_conj = ScalarOp<Scalar>::conj(l1ptr[j]);
            const Scalar l2j_conj = ScalarOp<Scalar>::conj(l2ptr[j]);
            MapVec(col_pointer(j + k + 2), ldim - j).noalias() -= (X.col(0).tail(ldim - j) * l1j_conj + X.col(1).tail(ldim - j) * l2j_conj);
        }

        // l = X
        l1.noalias() = X.col(0);
        l2.noalias() = X.col(1);

        return CompInfo::Successful;
    }

public:
    BKLDLT() :
        m_n(0), m_computed(false), m_info(CompInfo::NotComputed)
    {}

    // Factorize mat - shift * I
    template <typename Derived>
    BKLDLT(const Eigen::MatrixBase<Derived>& mat, int uplo = Eigen::Lower, const RealScalar& shift = RealScalar(0)) :
        m_n(mat.rows()), m_computed(false), m_info(CompInfo::NotComputed)
    {
        compute(mat, uplo, shift);
    }

    template <typename Derived>
    void compute(const Eigen::MatrixBase<Derived>& mat, int uplo = Eigen::Lower, const RealScalar& shift = RealScalar(0))
    {
        using std::abs;

        m_n = mat.rows();
        if (m_n != mat.cols())
            throw std::invalid_argument("BKLDLT: matrix must be square");

        m_perm.setLinSpaced(m_n, 0, m_n - 1);
        m_permc.clear();

        // Copy data
        m_data.resize((m_n * (m_n + 1)) / 2);
        compute_pointer();
        copy_data(mat, uplo, shift);

        const RealScalar alpha = (1.0 + std::sqrt(17.0)) / 8.0;
        Index k = 0;
        for (k = 0; k < m_n - 1; k++)
        {
            // 1. Interchange rows and columns of A, and save the result to m_perm
            bool is_1x1 = permutate_mat(k, alpha);

            // 2. Gaussian elimination
            if (is_1x1)
            {
                m_info = gaussian_elimination_1x1(k);
            }
            else
            {
                m_info = gaussian_elimination_2x2(k);
                k++;
            }

            // 3. Check status
            if (m_info != CompInfo::Successful)
                break;
        }
        // Invert the last 1x1 block if it exists
        if (k == m_n - 1)
        {
            const Scalar akk = ScalarOp<Scalar>::real(diag_coeff(k));
            diag_coeff(k) = akk;
            if (akk == Scalar(0))
                m_info = CompInfo::NumericalIssue;

            // [inverse]
            // diag_coeff(k) = Scalar(1) / diag_coeff(k);
        }

        compress_permutation();

        m_computed = true;
    }

    // Solve Ax=b
    void solve_inplace(GenericVector b) const
    {
        if (!m_computed)
            throw std::logic_error("BKLDLT: need to call compute() first");

        // PAP' = LD(L^H), A = P'LD(L^H)P
        // Ax = b ==> P'LD(L^H)Px = b ==> LD(L^H)Px = Pb
        // 1. b -> Pb
        Scalar* x = b.data();
        MapVec res(x, m_n);
        Index npermc = m_permc.size();
        for (Index i = 0; i < npermc; i++)
        {
            std::swap(x[m_permc[i].first], x[m_permc[i].second]);
        }

        // z = D(L^H)Px
        // 2. Lz = Pb
        // If m_perm[end] < 0, then end with m_n - 3, otherwise end with m_n - 2
        const Index end = (m_perm[m_n - 1] < 0) ? (m_n - 3) : (m_n - 2);
        for (Index i = 0; i <= end; i++)
        {
            const Index b1size = m_n - i - 1;
            const Index b2size = b1size - 1;
            if (m_perm[i] >= 0)
            {
                MapConstVec l(&coeff(i + 1, i), b1size);
                res.segment(i + 1, b1size).noalias() -= l * x[i];
            }
            else
            {
                MapConstVec l1(&coeff(i + 2, i), b2size);
                MapConstVec l2(&coeff(i + 2, i + 1), b2size);
                res.segment(i + 2, b2size).noalias() -= (l1 * x[i] + l2 * x[i + 1]);
                i++;
            }
        }

        // w = (L^H)Px
        // 3. Dw = z
        for (Index i = 0; i < m_n; i++)
        {
            const Scalar e11 = diag_coeff(i);
            if (m_perm[i] >= 0)
            {
                // [inverse]
                // x[i] *= e11;
                // [solve]
                x[i] /= e11;
            }
            else
            {
                const Scalar e21 = coeff(i + 1, i), e22 = diag_coeff(i + 1);
                // [inverse]
                // const Scalar e12 = ScalarOp<Scalar>::conj(e21);
                // const Scalar wi = x[i] * e11 + x[i + 1] * e12;
                // x[i + 1] = x[i] * e21 + x[i + 1] * e22;
                // x[i] = wi;
                // [solve]
                solve_inplace_2x2(e11, e21, e22, x[i], x[i + 1]);

                i++;
            }
        }

        // y = Px
        // 4. (L^H)y = w
        // If m_perm[end] < 0, then start with m_n - 3, otherwise start with m_n - 2
        Index i = (m_perm[m_n - 1] < 0) ? (m_n - 3) : (m_n - 2);
        for (; i >= 0; i--)
        {
            const Index ldim = m_n - i - 1;
            MapConstVec l(&coeff(i + 1, i), ldim);
            x[i] -= l.dot(res.segment(i + 1, ldim));

            if (m_perm[i] < 0)
            {
                MapConstVec l2(&coeff(i + 1, i - 1), ldim);
                x[i - 1] -= l2.dot(res.segment(i + 1, ldim));
                i--;
            }
        }

        // 5. x = P'y
        for (Index i = npermc - 1; i >= 0; i--)
        {
            std::swap(x[m_permc[i].first], x[m_permc[i].second]);
        }
    }

    Vector solve(ConstGenericVector& b) const
    {
        Vector res = b;
        solve_inplace(res);
        return res;
    }

    CompInfo info() const { return m_info; }
};

}  // namespace Spectra

#endif  // SPECTRA_BK_LDLT_H
