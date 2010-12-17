/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor.h
 * @brief   
 * @author  Richard Roberts
 * @created Dec 8, 2010
 */

#pragma once

#include <gtsam/base/blockMatrices.h>
#include <gtsam/linear/GaussianFactor.h>

// Forward declarations for friend unit tests
class ConversionConstructorHessianFactorTest;

namespace gtsam {

  // Forward declarations
  class JacobianFactor;
  class SharedDiagonal;

  // Definition of Scatter
  struct SlotEntry {
    size_t slot;
    size_t dimension;
    SlotEntry(size_t _slot, size_t _dimension) : slot(_slot), dimension(_dimension) {}
  };
  typedef FastMap<Index, SlotEntry> Scatter;

  class HessianFactor : public GaussianFactor {
  protected:
    typedef MatrixColMajor InfoMatrix;
    typedef SymmetricBlockView<InfoMatrix> BlockInfo;

    InfoMatrix matrix_; // The full information matrix [A b]^T * [A b]
    BlockInfo info_;    // The block view of the full information matrix.

    GaussianBayesNet::shared_ptr splitEliminatedFactor(size_t nrFrontals, const std::vector<Index>& keys);
    void updateATA(const HessianFactor& update, const Scatter& scatter);

  public:

    typedef boost::shared_ptr<HessianFactor> shared_ptr;
    typedef BlockInfo::Block Block;
    typedef BlockInfo::constBlock constBlock;

    /** Copy constructor */
    HessianFactor(const HessianFactor& gf);

    /** default constructor for I/O */
    HessianFactor();

    /** Construct Null factor */
    HessianFactor(const Vector& b_in);

    /** Construct unary factor */
    HessianFactor(Index i1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model);

    /** Construct binary factor */
    HessianFactor(Index i1, const Matrix& A1,
        Index i2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model);

    /** Construct ternary factor */
    HessianFactor(Index i1, const Matrix& A1, Index i2,
        const Matrix& A2, Index i3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model);

    /** Construct an n-ary factor */
    HessianFactor(const std::vector<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model);

    HessianFactor(const std::list<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model);

    /** Construct from Conditional Gaussian */
    HessianFactor(const GaussianConditional& cg);

    /** Convert from a JacobianFactor or HessianFactor (computes A^T * A) */
    HessianFactor(const GaussianFactor& factor);

    virtual ~HessianFactor() {}

    // Implementing Testable interface
    virtual void print(const std::string& s = "") const;
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

    virtual double error(const VectorValues& c) const; /**  0.5*(A*x-b)'*D*(A*x-b) */

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    virtual size_t getDim(const_iterator variable) const { return info_(variable-this->begin(), 0).size1(); }

    /** Return a view of a block of the information matrix */
    constBlock info(const_iterator j1, const_iterator j2) const { return info_(j1-begin(), j2-begin()); }

    /**
     * Permutes the GaussianFactor, but for efficiency requires the permutation
     * to already be inverted.  This acts just as a change-of-name for each
     * variable.  The order of the variables within the factor is not changed.
     */
    virtual void permuteWithInverse(const Permutation& inversePermutation) {
      FactorBase<Index>::permuteWithInverse(inversePermutation); }

    /**
     * Combine and eliminate several factors.
     */
    static std::pair<GaussianBayesNet::shared_ptr, shared_ptr> CombineAndEliminate(
        const FactorGraph<HessianFactor>& factors, size_t nrFrontals=1);

    // Friend unit test classes
    friend class ::ConversionConstructorHessianFactorTest;

    // Friend JacobianFactor for conversion
    friend class JacobianFactor;

  };

}

