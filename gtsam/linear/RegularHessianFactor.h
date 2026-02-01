/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file   RegularHessianFactor.h
  * @brief  Specialized HessianFactor class for regular problems (fixed-size blocks).
  * @details This factor is specifically designed for quadratic cost functions
  *          where all variables involved have the same, fixed dimension `D`.
  *          It stores the Hessian and gradient terms and provides optimized
  *          methods for operations like Hessian-vector products, crucial for
  *          iterative solvers like Conjugate Gradient. It ensures that all
  *          involved blocks have the expected dimension `D`.
  * @author Sungtae An
  * @author Frank Dellaert
  * @date   March 2014
  */

#pragma once

#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/RegularJacobianFactor.h>
#include <gtsam/linear/VectorValues.h> // For VectorValues and Scatter
#include <vector>
#include <stdexcept> // For std::invalid_argument

namespace gtsam {

  /**
   * @brief A HessianFactor where all variables have the same dimension D.
   * @details Represents a quadratic factor \f$ E(x) = 0.5 x^T G x - g^T x + 0.5 f \f$,
   * corresponding to a Gaussian probability density \f$ P(x) \propto \exp(-E(x)) \f$.
   * Here, G is the Hessian (or information matrix), g is the gradient vector,
   * and f is a constant term (negative log-likelihood at x=0).
   *
   * This class is templated on the dimension `D` and enforces that all variables
   * associated with this factor have this dimension during construction.
   * It inherits from HessianFactor but provides specialized, efficient implementations
   * for operations like `multiplyHessianAdd` using raw memory access, assuming
   * a regular block structure. This can significantly improve performance in
   * iterative optimization algorithms when dealing with many variables of the
   * same type and dimension (e.g., Pose3 variables in SLAM).
   *
   * It can be constructed directly from Hessian blocks, from a
   * RegularJacobianFactor, or by linearizing a NonlinearFactorGraph.
   *
   * @tparam D The dimension of each variable block involved in the factor.
   */
  template<size_t D>
  class RegularHessianFactor : public HessianFactor {

  public:

    typedef Eigen::Matrix<double, D, 1> VectorD;
    typedef Eigen::Matrix<double, D, D> MatrixD;

    /**
     * @brief Construct an n-way factor from supplied components.
     * @details Represents the energy function \f$ E(x) = 0.5 \sum_{i,j} x_i^T G_{ij} x_j - \sum_i g_i^T x_i + 0.5 f \f$.
     * Note that the keys in `js` must correspond order-wise to the blocks in `Gs` and `gs`.
     * @param js Vector of keys involved in the factor.
     * @param Gs Vector of upper-triangular Hessian blocks (row-major order, e.g., G11, G12, G13, G22, G23, G33 for a ternary factor).
     * @param gs Vector of gradient vector blocks (-J^T b).
     * @param f Constant term \f$ 0.5*f = 0.5*\|b\|^2 \f$.
     */
    RegularHessianFactor(const KeyVector& js,
      const std::vector<Matrix>& Gs, const std::vector<Vector>& gs, double f) :
      HessianFactor(js, Gs, gs, f) {
      checkInvariants();
    }

    /** Construct a binary factor.  Gxx are the upper-triangle blocks of the
     * quadratic term (the Hessian matrix), gx the pieces of the linear vector
     * term, and f the constant term.
     */
    RegularHessianFactor(Key j1, Key j2, const MatrixD& G11, const MatrixD& G12,
      const VectorD& g1, const MatrixD& G22, const VectorD& g2, double f) :
      HessianFactor(j1, j2, G11, G12, g1, G22, g2, f) {
    }

    /** Construct a ternary factor.  Gxx are the upper-triangle blocks of the
     * quadratic term (the Hessian matrix), gx the pieces of the linear vector
     * term, and f the constant term.
     */
    RegularHessianFactor(Key j1, Key j2, Key j3,
      const MatrixD& G11, const MatrixD& G12, const MatrixD& G13, const VectorD& g1,
      const MatrixD& G22, const MatrixD& G23, const VectorD& g2,
      const MatrixD& G33, const VectorD& g3, double f) :
      HessianFactor(j1, j2, j3, G11, G12, G13, g1, G22, G23, g2, G33, g3, f) {
    }

    /**
     * @brief Constructor with an arbitrary number of keys and the augmented information matrix
     *   specified as a block matrix.
     * @details The augmented information matrix contains the Hessian blocks G and the
     * gradient blocks g, arranged as \f$ [ G, -g ; -g^T, f ] \f$.
     * @param keys Container of keys specifying the variables involved and their order.
     * @param augmentedInformation The augmented information matrix [H -g; -g' f], laid out as a SymmetricBlockMatrix.
     */
    template<typename KEYS>
    RegularHessianFactor(const KEYS& keys,
      const SymmetricBlockMatrix& augmentedInformation) :
      HessianFactor(keys, augmentedInformation) {
      checkInvariants();
    }

    /**
     * @brief Construct a RegularHessianFactor from a RegularJacobianFactor.
     * @details Computes \f$ G = J^T J \f$, \f$ g = J^T b \f$, and \f$ f = b^T b \f$.
     * Requires that the JacobianFactor also has regular block sizes matching `D`.
     * @param jf The RegularJacobianFactor to convert.
     */
    RegularHessianFactor(const RegularJacobianFactor<D>& jf)
      : HessianFactor(jf) {
    }

    /**
     * @brief Construct from a GaussianFactorGraph combined using a Scatter.
     * @details This is typically used internally by optimizers. It sums the Hessian
     * contributions from multiple factors in the graph.
     * @param factors The graph containing factors to combine.
     * @param scatter A Scatter structure indicating the layout of variables.
     */
    RegularHessianFactor(const GaussianFactorGraph& factors,
      const Scatter& scatter)
      : HessianFactor(factors, scatter) {
      checkInvariants();
    }

    /**
     * @brief Construct from a GaussianFactorGraph.
     * @details This is typically used internally by optimizers. It sums the Hessian
     * contributions from multiple factors in the graph. Assumes keys are ordered 0..n-1.
     * @param factors The graph containing factors to combine.
     */
    RegularHessianFactor(const GaussianFactorGraph& factors)
      : HessianFactor(factors) {
      checkInvariants();
    }

  private:

    /**
     * @brief Check if the factor has regular block structure.
     * @details Verifies that the dimensions of the stored augmented information matrix
     * `info_` correspond to the expected size based on the number of keys and the
     * fixed block dimension `D`. Throws `std::invalid_argument` if the structure
     * is not regular. This is called internally by constructors.
     */
    void checkInvariants() const {
      if (info_.cols() != 0 && // Allow zero-key factors (e.g. priors on anchor nodes)
        info_.cols() != 1 + (info_.nBlocks() - 1) * static_cast<DenseIndex>(D))
        throw std::invalid_argument(
          "RegularHessianFactor constructor was given non-regular factors or "
          "incorrect template dimension D");
    }

    // Use Eigen magic to access raw memory for efficiency in Hessian products
    typedef Eigen::Map<VectorD> DMap;
    typedef Eigen::Map<const VectorD> ConstDMap;

    // Scratch space for multiplyHessianAdd to avoid re-allocation
    // According to link below this is thread-safe.
    // http://stackoverflow.com/questions/11160964/multiple-copies-of-the-same-object-c-thread-safe
    mutable std::vector<VectorD, Eigen::aligned_allocator<VectorD>> y_;

  public:

    /**
     * @brief Multiply the Hessian part of the factor times a VectorValues `x` and add the result to `y`.
     * @details Computes `y += alpha * H * x`.
     * @param alpha Scalar multiplier.
     * @param x VectorValues containing the vector to multiply with.
     * @param[in,out] y VectorValues to store the result (accumulates).
     */
    void multiplyHessianAdd(double alpha, const VectorValues& x,
      VectorValues& y) const override {
      // Note: This implementation just calls the base class. The raw memory versions
      // below are specifically optimized for the regular structure of this class.
      // Consider using those directly or ensuring the base class implementation
      // is efficient enough for your use case if calling this version.
      HessianFactor::multiplyHessianAdd(alpha, x, y);
    }

    /**
     * @brief Multiply the Hessian part of the factor times a raw vector `x` and add the result to `y`. (Raw memory version)
     * @details Computes `y += alpha * H * x`, optimized for regular block structure.
     *          Assumes `x` and `yvalues` are pre-allocated, contiguous memory blocks
     *          where variable `j` corresponds to memory chunk `x + keys_[j] * D`
     *          and `yvalues + keys_[j] * D`.
     * @param alpha Scalar multiplier.
     * @param x Raw pointer to the input vector data.
     * @param[in,out] yvalues Raw pointer to the output vector data (accumulates).
     */
    void multiplyHessianAdd(double alpha, const double* x,
      double* yvalues) const {
      // Ensure scratch space is properly sized
      const size_t n = size();
      if (y_.size() != n) {
        y_.resize(n);
      }
      for (VectorD& yi : y_)
        yi.setZero();

      // Loop over columns (j) of the Hessian H=info_
      // Accessing x only once per column j
      // Fill temporary y_ vector corresponding to rows i
      for (DenseIndex j = 0; j < static_cast<DenseIndex>(n); ++j) {
        Key key_j = keys_[j];
        ConstDMap xj(x + key_j * D); // Map to the j-th block of x

        DenseIndex i = 0;
        // Off-diagonal blocks G_ij * x_j for i < j
        for (; i < j; ++i) {
          y_[i] += info_.aboveDiagonalBlock(i, j) * xj;
        }
        // Diagonal block G_jj * x_j
        y_[i] += info_.diagonalBlock(j) * xj;
        // Off-diagonal blocks G_ij * x_j for i > j (using transpose G_ji^T * x_j)
        for (i = j + 1; i < static_cast<DenseIndex>(n); ++i) {
          y_[i] += info_.aboveDiagonalBlock(j, i).transpose() * xj;
        }
      }

      // Add accumulated results from y_ to the output yvalues
      for (DenseIndex i = 0; i < static_cast<DenseIndex>(n); ++i) {
        Key key_i = keys_[i];
        DMap map_yi(yvalues + key_i * D); // Map to the i-th block of yvalues
        map_yi += alpha * y_[i];
      }
    }

    /**
     * @brief Multiply the Hessian part of the factor times a raw vector `x` and add the result to `y`.
     * @details Computes `y += alpha * H * x`, optimized for regular block structure,
     *          but handles potentially non-contiguous or scattered memory layouts
     *          for `x` and `yvalues` as defined by the `offsets` vector.
     *          `offsets[k]` should give the starting index of variable `k` in the
     *          raw memory arrays. `offsets[k+1] - offsets[k]` should equal `D`.
     * @param alpha Scalar multiplier.
     * @param x Raw pointer to the input vector data.
     * @param[in,out] yvalues Raw pointer to the output vector data (accumulates).
     * @param offsets Vector mapping variable keys to their starting index in `x` and `yvalues`.
     */
    void multiplyHessianAdd(double alpha, const double* x, double* yvalues,
      const std::vector<size_t>& offsets) const {
      // Ensure scratch space is properly sized
      const size_t n = size();
      if (y_.size() != n) {
        y_.resize(n);
      }
      for (VectorD& yi : y_)
        yi.setZero();

      // Loop over columns (j) of the Hessian H=info_
      for (DenseIndex j = 0; j < static_cast<DenseIndex>(n); ++j) {
        Key key_j = keys_[j];
        size_t offset_j = offsets[key_j];
        // Ensure block size matches D (redundant if checkInvariants worked, but safe)
        size_t dim_j = offsets[key_j + 1] - offset_j;
        if (dim_j != D) throw std::runtime_error("RegularHessianFactor::multiplyHessianAdd: Mismatched dimension in offset map.");
        ConstDMap xj(x + offset_j); // Map to the j-th block of x using offset

        DenseIndex i = 0;
        // Off-diagonal blocks G_ij * x_j for i < j
        for (; i < j; ++i) {
          y_[i] += info_.aboveDiagonalBlock(i, j) * xj;
        }
        // Diagonal block G_jj * x_j
        y_[i] += info_.diagonalBlock(j) * xj;
        // Off-diagonal blocks G_ij * x_j for i > j (using transpose G_ji^T * x_j)
        for (i = j + 1; i < static_cast<DenseIndex>(n); ++i) {
          y_[i] += info_.aboveDiagonalBlock(j, i).transpose() * xj;
        }
      }

      // Add accumulated results from y_ to the output yvalues using offsets
      for (DenseIndex i = 0; i < static_cast<DenseIndex>(n); ++i) {
        Key key_i = keys_[i];
        size_t offset_i = offsets[key_i];
        size_t dim_i = offsets[key_i + 1] - offset_i;
        if (dim_i != D) throw std::runtime_error("RegularHessianFactor::multiplyHessianAdd: Mismatched dimension in offset map.");
        DMap map_yi(yvalues + offset_i); // Map to the i-th block of yvalues using offset
        map_yi += alpha * y_[i];
      }
    }

    /**
     * @brief Return the diagonal of the Hessian for this factor (Raw memory version).
     * @details Adds the diagonal elements of the Hessian matrix \f$ H = G \f$ associated
     *          with this factor to the pre-allocated raw memory block `d`.
     *          Assumes `d` has the standard contiguous layout `d + keys_[i] * D`.
     * @param[in,out] d Raw pointer to the memory block where Hessian diagonals should be added.
     */
    void hessianDiagonal(double* d) const override {
      // Loop over all variables (diagonal blocks) in the factor
      const size_t n = size();
      for (DenseIndex pos = 0; pos < static_cast<DenseIndex>(n); ++pos) {
        Key j = keys_[pos];
        // Get the diagonal block G_jj and add its diagonal elements to d
        DMap(d + D * j) += info_.diagonal(pos);
      }
    }

    /**
     * @brief Add the gradient vector \f$ -g \f$ (gradient at zero) to a raw memory block `d`.
     * @details Adds the gradient vector \f$ -g \f$ (where the factor represents
     *          \f$ 0.5 x^T G x - g^T x + 0.5 f \f$) to the pre-allocated raw
     *          memory block `d`. Assumes `d` has the standard contiguous layout
     *          `d + keys_[i] * D`.
     * @param[in,out] d Raw pointer to the memory block where the gradient should be added.
     */
    void gradientAtZero(double* d) const override {
      // The linear term g is stored as the last block column of info_ (negated).
      // We add (-g) to d.
      const size_t n = size();
      for (DenseIndex pos = 0; pos < static_cast<DenseIndex>(n); ++pos) {
        Key j = keys_[pos];
        // info_.aboveDiagonalBlock(pos, n) accesses the block corresponding to g_j
        DMap(d + D * j) += info_.aboveDiagonalBlock(pos, n);
      }
    }

    /* ************************************************************************* */

  }; // end class RegularHessianFactor

  // traits
  template<size_t D> struct traits<RegularHessianFactor<D> > : public Testable<
    RegularHessianFactor<D> > {
  };

} // namespace gtsam