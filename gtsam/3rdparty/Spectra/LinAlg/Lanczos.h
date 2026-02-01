// Copyright (C) 2018-2025 Yixuan Qiu <yixuan.qiu@cos.name>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#ifndef SPECTRA_LANCZOS_H
#define SPECTRA_LANCZOS_H

#include <Eigen/Core>
#include <cmath>      // std::sqrt
#include <utility>    // std::forward
#include <stdexcept>  // std::invalid_argument

#include "Arnoldi.h"

namespace Spectra {

// Lanczos factorization A * V = V * H + f * e'
// A: n x n
// V: n x k
// H: k x k
// f: n x 1
// e: [0, ..., 0, 1]
// V and H are allocated of dimension m, so the maximum value of k is m
template <typename Scalar, typename ArnoldiOpType>
class Lanczos : public Arnoldi<Scalar, ArnoldiOpType>
{
private:
    // The real part type of the matrix element
    using RealScalar = typename Eigen::NumTraits<Scalar>::Real;
    using Index = Eigen::Index;
    using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using MapMat = Eigen::Map<Matrix>;
    using MapVec = Eigen::Map<Vector>;
    using MapConstMat = Eigen::Map<const Matrix>;
    using RealMatrix = Eigen::Matrix<RealScalar, Eigen::Dynamic, Eigen::Dynamic>;

    using Arnoldi<Scalar, ArnoldiOpType>::m_op;
    using Arnoldi<Scalar, ArnoldiOpType>::m_n;
    using Arnoldi<Scalar, ArnoldiOpType>::m_m;
    using Arnoldi<Scalar, ArnoldiOpType>::m_k;
    using Arnoldi<Scalar, ArnoldiOpType>::m_fac_V;
    using Arnoldi<Scalar, ArnoldiOpType>::m_fac_H;
    using Arnoldi<Scalar, ArnoldiOpType>::m_fac_f;
    using Arnoldi<Scalar, ArnoldiOpType>::m_beta;
    using Arnoldi<Scalar, ArnoldiOpType>::m_near_0;
    using Arnoldi<Scalar, ArnoldiOpType>::m_eps;

public:
    // Forward parameter `op` to the constructor of Arnoldi
    template <typename T>
    Lanczos(T&& op, Index m) :
        Arnoldi<Scalar, ArnoldiOpType>(std::forward<T>(op), m)
    {}

    // Lanczos factorization starting from step-k
    void factorize_from(Index from_k, Index to_m, Index& op_counter) override
    {
        using std::abs;
        using std::sqrt;

        if (to_m <= from_k)
            return;

        if (from_k > m_k)
        {
            std::string msg = "Lanczos: from_k (= " + std::to_string(from_k) +
                ") is larger than the current subspace dimension (= " + std::to_string(m_k) + ")";
            throw std::invalid_argument(msg);
        }

        const RealScalar beta_thresh = m_eps * sqrt(RealScalar(m_n));
        const RealScalar eps_sqrt = sqrt(m_eps);

        // Pre-allocate vectors
        Vector Vf(to_m);
        Vector w(m_n);

        // Keep the upperleft k x k submatrix of H and set other elements to 0
        m_fac_H.rightCols(m_m - from_k).setZero();
        m_fac_H.block(from_k, 0, m_m - from_k, from_k).setZero();

        for (Index i = from_k; i <= to_m - 1; i++)
        {
            // If beta = 0, then the next V is not full rank
            // We need to generate a new residual vector that is orthogonal
            // to the current V, which we call a restart
            //
            // A simple criterion is beta < near_0, but it may be too stringent
            // Another heuristic is to test whether (V^H)B(f/||f||) ~= 0 when ||f|| is small,
            // and to reduce the computational cost, we only use the latest Vi

            // Test the first criterion
            bool restart = (m_beta < m_near_0);
            // If not met, test the second criterion
            // v is the (i+1)-th column of V
            MapVec v(&m_fac_V(0, i), m_n);
            if (!restart)
            {
                // Save v <- f / ||f|| to the (i+1)-th column of V
                v.noalias() = m_fac_f / m_beta;
                if (m_beta < eps_sqrt)
                {
                    // Test (Vi^H)v
                    const Scalar Viv = m_op.inner_product(m_fac_V.col(i - 1), v);
                    // Restart V if (Vi^H)v is much larger than eps
                    restart = (abs(Viv) > eps_sqrt);
                }
            }

            if (restart)
            {
                MapConstMat V(m_fac_V.data(), m_n, i);  // The first i columns
                this->expand_basis(V, 2 * i, m_fac_f, m_beta, op_counter);
                v.noalias() = m_fac_f / m_beta;
            }

            // Whether there is a restart or not, right now the (i+1)-th column of V
            // contains f / ||f||

            // Note that H[i+1, i] equals to the unrestarted beta
            m_fac_H(i, i - 1) = restart ? Scalar(0) : Scalar(m_beta);
            m_fac_H(i - 1, i) = m_fac_H(i, i - 1);  // Due to symmetry

            // w <- A * v
            m_op.perform_op(v.data(), w.data());
            op_counter++;

            // f <- w - V * (V^H)Bw = w - H[i+1, i] * V{i} - H[i+1, i+1] * V{i+1}
            // If restarting, we know that H[i+1, i] = 0
            // First do w <- w - H[i+1, i] * V{i}, see the discussions in Section 2.3 of
            // Cullum and Willoughby (2002). Lanczos Algorithms for Large Symmetric Eigenvalue Computations: Vol. 1
            if (!restart)
                w.noalias() -= m_fac_H(i, i - 1) * m_fac_V.col(i - 1);

            // H[i+1, i+1] = <v, w> = (v^H)Bw
            m_fac_H(i, i) = m_op.inner_product(v, w);

            // f <- w - H[i+1, i+1] * V{i+1}
            m_fac_f.noalias() = w - m_fac_H(i, i) * v;
            m_beta = m_op.norm(m_fac_f);

            // f/||f|| is going to be the next column of V, so we need to test
            // whether (V^H)B(f/||f||) ~= 0
            const Index i1 = i + 1;
            MapMat Vs(m_fac_V.data(), m_n, i1);  // The first (i+1) columns
            m_op.adjoint_product(Vs, m_fac_f, Vf.head(i1));
            RealScalar ortho_err = Vf.head(i1).cwiseAbs().maxCoeff();
            // If not, iteratively correct the residual
            int count = 0;
            while (count < 5 && ortho_err > m_eps * m_beta)
            {
                // There is an edge case: when beta=||f|| is close to zero, f mostly consists
                // of noises of rounding errors, so the test [ortho_err < eps * beta] is very
                // likely to fail. In particular, if beta=0, then the test is ensured to fail.
                // Hence when this happens, we force f to be zero, and then restart in the
                // next iteration.
                if (m_beta < beta_thresh)
                {
                    m_fac_f.setZero();
                    m_beta = RealScalar(0);
                    break;
                }

                // f <- f - V * Vf
                m_fac_f.noalias() -= Vs * Vf.head(i1);
                // h <- h + Vf
                m_fac_H(i - 1, i) += Vf[i - 1];
                m_fac_H(i, i - 1) = m_fac_H(i - 1, i);
                m_fac_H(i, i) += Vf[i];
                // beta <- ||f||
                m_beta = m_op.norm(m_fac_f);

                m_op.adjoint_product(Vs, m_fac_f, Vf.head(i1));
                ortho_err = Vf.head(i1).cwiseAbs().maxCoeff();
                count++;
            }
        }

        // Indicate that this is a step-m factorization
        m_k = to_m;
    }

    // Apply H -> Q'HQ, where Q is from a tridiagonal QR decomposition
    // Function overloading here, not overriding
    //
    // Note that H is by nature a real symmetric matrix, but it may be stored
    // as a complex matrix (e.g. in HermEigsSolver).
    // Therefore, if m_fac_H has a real type (as in SymEigsSolver), then we
    // directly overwrite m_fac_H. Otherwise, m_fac_H has a complex type
    // (as in HermEigsSolver), so we first compute the real-typed result,
    // and then cast to the complex type. This is done in the TridiagQR class
    void compress_H(const TridiagQR<RealScalar>& decomp)
    {
        decomp.matrix_QtHQ(m_fac_H);
        m_k--;
    }

    // In some cases we know that H has the form H = [X   e   0],
    //                                               [e'  s   0]
    //                                               [0   0   D]
    // where X is an irreducible tridiagonal matrix, D is a diagonal matrix,
    // s is a scalar, and e = (0, ..., 0, eps), eps ~= 0
    //
    // In this case we can force H[m+1, m] = H[m, m+1] = 0 and H[m+1, m+1] = s,
    // where m is the size of X
    void deflate_H(Index irr_size, const Scalar& s)
    {
        m_fac_H(irr_size, irr_size - 1) = Scalar(0);
        m_fac_H(irr_size - 1, irr_size) = Scalar(0);
        m_fac_H(irr_size, irr_size) = s;
    }
};

}  // namespace Spectra

#endif  // SPECTRA_LANCZOS_H
