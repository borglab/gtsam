// -*- mode: c++ -*-
#ifndef _BLOCK_DIAG_MATRIX_H_
#define _BLOCK_DIAG_MATRIX_H_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef SYM_ILDL_DEBUG
template <class el_type>
std::ostream &operator<<(std::ostream &os, const std::vector<el_type> &vec) {
  os << "[";
  if (!vec.empty()) {
    for (typename std::vector<el_type>::size_type index = 0;
         index < vec.size() - 1; index++) {
      os << vec[index] << ", ";
    }

    os << vec[vec.size() - 1];
  }
  os << "]";
  return os;
}
#endif

using std::abs;

/*! \brief A quick implementation of a diagonal matrix with 1x1 and 2x2 blocks.
 */
template <class el_type> class block_diag_matrix {
public:
  typedef std::unordered_map<int, el_type> int_elt_map;
  typedef std::vector<el_type> elt_vector_type;

  /*! Allows outputting the contents of the matrix via << operators. */
  friend std::ostream &operator<<(std::ostream &os,
                                  const block_diag_matrix &D) {
    os << D.to_string();
    return os;
  };

  int m_n_size;              ///< Dimension of the matrix.
  int nnz_count;             ///< Number of non-zeros in the matrix.
  elt_vector_type main_diag; ///< Stores main diagonal elements.
  int_elt_map off_diag;      ///< Stores off-diagonal elements of 2x2 pivots.

  /*!	\brief Constructor for diagonal class. Initializes a 0x0 matrix when
   * given no arguments.
   */
  block_diag_matrix(int n_rows = 0, int n_cols = 0) : m_n_size(n_rows) {
    assert(n_rows == n_cols);
    nnz_count = n_rows;
    main_diag.clear();
    main_diag.resize(n_rows);
  }

  /*!	\brief Resizes this matrix to an n*n matrix with default_value on the
   * main diagonal.
   */
  void resize(int n, el_type default_value) {
    m_n_size = n;
    main_diag.clear();
    main_diag.resize(n, default_value);
    off_diag.clear();
    nnz_count = n;
  }

  /*!	\brief Resizes this matrix to an n*n matrix.
   */
  void resize(int n) {
    m_n_size = n;
    resize(n, 0);
    nnz_count = n;
  }

  /*! \return Number of rows in the matrix. */
  int n_rows() const { return m_n_size; }

  /*! \return Number of cols in the matrix. */
  int n_cols() const { return m_n_size; }

  /*! \return Number of nonzeros in the matrix. */
  int nnz() const { return nnz_count; };

  /*!	\param i the index of the element.
          \return The D(i,i)th element.
  */
  el_type &operator[](int i) { return main_diag.at(i); }

  /*!	\param i the index of the element.
          \return The D(i+1,i)th element.
  */
  el_type &off_diagonal(int i) {
    if (!off_diag.count(i)) {
      off_diag.insert(std::make_pair(i, 0));
      nnz_count++;
    }

    return off_diag[i];
  }

  /*!	\param i the index of the element.
          \return 2 if there is a diagonal pivot at D(i,i) and D(i+1,i+1).
                          -2 if there is a diagonal pivot at D(i-1,i-1) and
     D(i,i). 1 if the pivot is only a 1x1 block.
  */
  int block_size(int i) const {
    if (off_diag.count(i)) {
      return 2;
    } else if (off_diag.count(i - 1)) {
      return -2;
    } else {
      return 1;
    }
  }

  /*!	\brief Solves the preconditioned problem |D| = Q|V|Q', where QVQ' is the
  eigendecomposition of D, and |.| is applied elementwise. \param b the right
  hand side. \param x a storage vector for the solution (must be same size as
  b). \param transposed solves |V|^(1/2)Q' if true, Q|V|^(1/2) if false.
  */
  void sqrt_solve(const elt_vector_type &b, elt_vector_type &x,
                  bool transposed = false) {
    assert(b.size() == x.size());

    const double eps = 1e-8;
    double alpha, beta, gamma, eig0, eig1, disc;
    double Q[2][2], tx[2];
    for (int i = 0; i < m_n_size; i += block_size(i)) {
      if (block_size(i) == 2) {
        alpha = main_diag[i];
        beta = main_diag[i + 1];
        gamma = off_diag[i];

        disc = sqrt((alpha - beta) * (alpha - beta) + 4 * gamma * gamma);
        eig0 = 0.5 * (alpha + beta + disc);
        eig1 = 0.5 * (alpha + beta - disc);

        if (abs(gamma / std::min(alpha, beta)) < eps) {
          eig0 = alpha;
          eig1 = beta;
          Q[0][0] = 1;
          Q[1][0] = 0;
          Q[0][1] = 0;
          Q[1][1] = 1;
        } else {
          double sin2t = 2 * gamma / disc, cos2t = (alpha - beta) / disc;
          double theta = 0.5 * atan2(sin2t, cos2t);

          Q[0][0] = cos(theta);
          Q[0][1] = -sin(theta);
          Q[1][0] = sin(theta);
          Q[1][1] = cos(theta);
        }

        // solves Q|V|^(1/2) x = b or solves the transposed version |V|^(1/2)Q'
        // x = b.
        if (!transposed) {
          // tx = Q'*b
          tx[0] = Q[0][0] * b[i] + Q[1][0] * b[i + 1];
          tx[1] = Q[0][1] * b[i] + Q[1][1] * b[i + 1];

          // x = |V|^(-1/2)*tx
          x[i] = tx[0] / sqrt(abs(eig0));
          x[i + 1] = tx[1] / sqrt(abs(eig1));
        } else {
          // tx = |V|^(-1/2)*b
          tx[0] = b[i] / sqrt(abs(eig0));
          tx[1] = b[i + 1] / sqrt(abs(eig1));

          // x = Q*tx
          x[i] = Q[0][0] * tx[0] + Q[0][1] * tx[1];
          x[i + 1] = Q[1][0] * tx[0] + Q[1][1] * tx[1];
        }
      } else {
        x[i] = b[i] / sqrt(abs(main_diag[i]));
      }
    }
  }

  /** Solves Q|V|Q'x = b, where Q V Q' = D is the eigendecomposition of
   * D, and Lambda is the matrix obtained by replacing the eigenvalues of D with
   * their absolute values
   *
   * Added by DM Rosen 6-11-2020
   */
  void pos_def_solve(const elt_vector_type &b, elt_vector_type &x) {
    assert(b.size() == x.size());

    const double eps = 1e-8;
    double alpha, beta, gamma, eig0, eig1, disc;
    double Q[2][2], tx[2];
    for (int i = 0; i < m_n_size; i += block_size(i)) {
      if (block_size(i) == 2) {
        alpha = main_diag[i];
        beta = main_diag[i + 1];
        gamma = off_diag[i];

        /// Compute eigendecomposition of this 2x2 block

        disc = sqrt((alpha - beta) * (alpha - beta) + 4 * gamma * gamma);
        eig0 = 0.5 * (alpha + beta + disc);
        eig1 = 0.5 * (alpha + beta - disc);

        if (abs(gamma / std::min(alpha, beta)) < eps) {
          eig0 = alpha;
          eig1 = beta;
          Q[0][0] = 1;
          Q[1][0] = 0;
          Q[0][1] = 0;
          Q[1][1] = 1;
        } else {
          double sin2t = 2 * gamma / disc, cos2t = (alpha - beta) / disc;
          double theta = 0.5 * atan2(sin2t, cos2t);

          Q[0][0] = cos(theta);
          Q[0][1] = -sin(theta);
          Q[1][0] = sin(theta);
          Q[1][1] = cos(theta);
        }

        /// Now compute x = Q |V|^-1 Q' b

        // tx = Q'*b
        tx[0] = Q[0][0] * b[i] + Q[1][0] * b[i + 1];
        tx[1] = Q[0][1] * b[i] + Q[1][1] * b[i + 1];

        // tx = |V|^-1 tx
        tx[0] /= abs(eig0);
        tx[1] /= abs(eig1);

        // x = Q*tx
        x[i] = Q[0][0] * tx[0] + Q[0][1] * tx[1];
        x[i + 1] = Q[1][0] * tx[0] + Q[1][1] * tx[1];
      } else {
        x[i] = b[i] / abs(main_diag[i]);
      }
    }
  }

  /**	\brief Solves the system Dx = b.
   *    \param b the right hand side.
   *    \param x a storage vector for the solution (must be same size as b).
   */
  void solve(const elt_vector_type &b, elt_vector_type &x) {
    assert(b.size() == x.size());

    double a, d, c, det;
    for (int i = 0; i < m_n_size; i += block_size(i)) {
      if (block_size(i) == 2) {
        a = main_diag[i];
        d = main_diag[i + 1];
        c = off_diag[i];
        det = a * d - c * c;
        // system is (a c; c d)
        // inverse is 1/(ad - c^2) * (d -c; -c a)
        x[i] = (d * b[i] - c * b[i + 1]) / det;
        x[i + 1] = (-c * b[i] + a * b[i + 1]) / det;
      } else {
        x[i] = b[i] / main_diag[i];
      }
    }
  }

  /*! \return A string reprepsentation of this matrix.
   */
  std::string to_string() const;

  /*! \param filename the filename of the matrix to be saved. All matrices saved
     are in matrix market format (.mtx). \return True if the save succeeded,
     false otherwise.
  */
  bool save(std::string filename) const;

  /*! \brief Generic class destructor.
   */
  ~block_diag_matrix() {}
};

#include "block_diag_matrix_save.h"
#include "block_diag_matrix_to_string.h"

#endif
