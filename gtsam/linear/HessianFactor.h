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
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/GaussianFactor.h>

// Forward declarations for friend unit tests
class ConversionConstructorHessianFactorTest;
class Constructor1HessianFactorTest;
class combineHessianFactorTest;


namespace gtsam {

  // Forward declarations
  class JacobianFactor;
  struct SharedDiagonal;
  class GaussianConditional;
  template<class C> class BayesNet;

  // Definition of Scatter
  struct SlotEntry {
    size_t slot;
    size_t dimension;
    SlotEntry(size_t _slot, size_t _dimension)
    : slot(_slot), dimension(_dimension) {}
    std::string toString() const;
  };
  typedef FastMap<Index, SlotEntry> Scatter;

  class HessianFactor : public GaussianFactor {
  protected:
    typedef Matrix InfoMatrix;
    typedef SymmetricBlockView<InfoMatrix> BlockInfo;

    InfoMatrix matrix_; // The full information matrix, s.t. the quadratic error is 0.5*[x -1]'*H*[x -1]
    BlockInfo info_;    // The block view of the full information matrix.

    void updateATA(const JacobianFactor& update, const Scatter& scatter);
    void updateATA(const HessianFactor& update, const Scatter& scatter);

  public:

    typedef boost::shared_ptr<HessianFactor> shared_ptr;
    typedef BlockInfo::Block Block;
    typedef BlockInfo::constBlock constBlock;
    typedef BlockInfo::Column Column;
    typedef BlockInfo::constColumn constColumn;

    /** Copy constructor */
    HessianFactor(const HessianFactor& gf);

    /** default constructor for I/O */
    HessianFactor();

    /** Construct a unary factor.  G is the quadratic term (Hessian matrix), g
     * the linear term (a vector), and f the constant term.  The quadratic
     * error is:
     * 0.5*(f - 2*x'*g + x'*G*x)
     */
    HessianFactor(Index j, const Matrix& G, const Vector& g, double f);

    /** Construct a binary factor.  Gxx are the upper-triangle blocks of the
     * quadratic term (the Hessian matrix), gx the pieces of the linear vector
     * term, and f the constant term.  The quadratic error is:
     * 0.5*(f - 2*x1'*g1 - 2*x2'*g2 + x1'*G11*x1 + x1'*G12*x2)
     */
    HessianFactor(Index j1, Index j2,
        const Matrix& G11, const Matrix& G12, const Vector& g1,
        const Matrix& G22, const Vector& g2, double f);

    /** Construct from Conditional Gaussian */
    HessianFactor(const GaussianConditional& cg);

    /** Convert from a JacobianFactor or HessianFactor (computes A^T * A) */
    HessianFactor(const GaussianFactor& factor);

    /** Special constructor used in EliminateCholesky */
    HessianFactor(const FactorGraph<GaussianFactor>& factors,
				const std::vector<size_t>& dimensions, const Scatter& scatter);

		virtual ~HessianFactor() {}

    // Implementing Testable interface
    virtual void print(const std::string& s = "") const;
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

    virtual double error(const VectorValues& c) const; /** 0.5*[x -1]'*H*[x -1] (also see constructor documentation) */

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    virtual size_t getDim(const_iterator variable) const { return info_(variable-this->begin(), 0).rows(); }

    /** Return the number of columns and rows of the Hessian matrix */
    size_t rows() const { return info_.rows(); }

    /** Return a view of a block of the information matrix */
    constBlock info(const_iterator j1, const_iterator j2) const { return info_(j1-begin(), j2-begin()); }

    /** returns the constant term - f from the constructors */
    double constant_term() const;

    /** returns the full linear term - g from the constructors */
    constColumn linear_term() const;

    /**
     * Permutes the GaussianFactor, but for efficiency requires the permutation
     * to already be inverted.  This acts just as a change-of-name for each
     * variable.  The order of the variables within the factor is not changed.
     */
    virtual void permuteWithInverse(const Permutation& inversePermutation) {
      IndexFactor::permuteWithInverse(inversePermutation); }

    // Friend unit test classes
    friend class ::ConversionConstructorHessianFactorTest;
    friend class ::Constructor1HessianFactorTest;
    friend class ::combineHessianFactorTest;

    // Friend JacobianFactor for conversion
    friend class JacobianFactor;

    // used in eliminateCholesky:

    /**
		 * Do Cholesky. Note that after this, the lower triangle still contains
		 * some untouched non-zeros that should be zero.  We zero them while
		 * extracting submatrices in splitEliminatedFactor. Frank says :-(
		 */
    void partialCholesky(size_t nrFrontals);

    /**
     * Do LDL.
     */
    Eigen::LDLT<Matrix>::TranspositionType partialLDL(size_t nrFrontals);

    /** split partially eliminated factor */
    boost::shared_ptr<GaussianConditional> splitEliminatedFactor(
				size_t nrFrontals, const std::vector<Index>& keys, const Eigen::LDLT<Matrix>::TranspositionType& permutation = Eigen::LDLT<Matrix>::TranspositionType());

    /** assert invariants */
    void assertInvariants() const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
    	ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GaussianFactor);
    	ar & BOOST_SERIALIZATION_NVP(info_);
    	ar & BOOST_SERIALIZATION_NVP(matrix_);
    }
  };

}

