/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HessianFactor.h
 * @brief   Contains the HessianFactor class, a general quadratic factor
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
class Constructor1bHessianFactorTest;
class combineHessianFactorTest;


namespace gtsam {

  // Forward declarations
  class JacobianFactor;
  struct SharedDiagonal;
  class GaussianConditional;
  template<class C> class BayesNet;

  // Definition of Scatter, which is an intermediate data structure used when
  // building a HessianFactor incrementally, to get the keys in the right
  // order.
  struct SlotEntry {
    size_t slot;
    size_t dimension;
    SlotEntry(size_t _slot, size_t _dimension)
    : slot(_slot), dimension(_dimension) {}
    std::string toString() const;
  };
  typedef FastMap<Index, SlotEntry> Scatter;

  /**
   * A general quadratic factor of the form
   * \f[ e(x) = x^T G x + gx + f \f]
   * that stores the matrix \f$ G \f$, the vector \f$ g \f$, and the constant term \f$ f \f$.
   *
   * When \f$ G \f$ is positive semidefinite, this factor represents a Gaussian,
   * in which case \f$ G \f$ is the information
   * matrix \f$ \Lambda \f$, which is the inverse of the covariance matrix \f$ \Sigma \f$,
   * \f$ g \f$ is the information vector \f$ \eta = \Lambda \mu \f$, and \f$ f \f$ is the error
   * at the mean, when \f$ x = \mu \f$ .
   *
   * This factor is one of the factors that can be in a GaussianFactorGraph.
   * It may be returned from NonlinearFactor::linearize(), but is also
   * used internally to store the Hessian during Cholesky elimination.
   *
   * This can represent a quadratic factor with characteristics that cannot be
   * represented using a JacobianFactor (which has the form
   * \f$ e(x) = \Vert Ax - b \Vert^2 \f$ and stores the Jacobian \f$ A \f$
   * and error vector \f$ b \f$, i.e. is a sum-of-squares factor).  For example,
   * a HessianFactor need not be positive semidefinite, it can be indefinite or
   * even negative semidefinite.
   *
   * If a HessianFactor is indefinite or negative semi-definite, then in order
   * for solving the linear system to be possible,
   * the Hessian of the full system must be positive definite (i.e. when all
   * small Hessians are combined, the result must be positive definite).  If
   * this is not the case, an error will occur during elimination.
   *
   * This class stores G, g, and f as an augmented matrix HessianFactor::matrix_.
   * The upper-left n x n blocks of HessianFactor::matrix_ store the upper-right
   * triangle of G, the upper-right-most column of length n of HessianFactor::matrix_
   * stores g, and the lower-right entry of HessianFactor::matrix_ stores f, i.e.
   * \code
     HessianFactor::matrix_ = [ G11 G12 G13 ... g1
                                  0 G22 G23 ... g2
                                  0   0 G33 ... g3
                                  :   :   :      :
                                  0   0   0 ...  f ]
     \endcode
     Blocks can be accessed as follows:
     \code
     G11 = info(begin(), begin());
     G12 = info(begin(), begin()+1);
     G23 = info(begin()+1, begin()+2);
     g2 = linearTerm(begin()+1);
     f = constantTerm();
     .......
     \endcode
   */
  class HessianFactor : public GaussianFactor {
  protected:
    typedef Matrix InfoMatrix; ///< The full augmented Hessian
    typedef SymmetricBlockView<InfoMatrix> BlockInfo; ///< A blockwise view of the Hessian

    InfoMatrix matrix_; ///< The full augmented information matrix, s.t. the quadratic error is 0.5*[x -1]'*H*[x -1]
    BlockInfo info_;    ///< The block view of the full information matrix.

    /** Update the factor by adding the information from the JacobianFactor
     * (used internally during elimination).
     * @param update The JacobianFactor containing the new information to add
     * @param scatter A mapping from variable index to slot index in this HessianFactor
     */
    void updateATA(const JacobianFactor& update, const Scatter& scatter);

    /** Update the factor by adding the information from the HessianFactor
     * (used internally during elimination).
     * @param update The HessianFactor containing the new information to add
     * @param scatter A mapping from variable index to slot index in this HessianFactor
     */
    void updateATA(const HessianFactor& update, const Scatter& scatter);

  public:

    typedef boost::shared_ptr<HessianFactor> shared_ptr; ///< A shared_ptr to this
    typedef BlockInfo::Block Block; ///< A block from the Hessian matrix
    typedef BlockInfo::constBlock constBlock; ///< A block from the Hessian matrix (const version)
    typedef BlockInfo::Column Column; ///< A column containing the linear term h
    typedef BlockInfo::constColumn constColumn; ///< A column containing the linear term h (const version)

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

    /** Construct a unary factor, given a mean and covariance matrix.
     * error is 0.5*(x-mu)'*inv(Sigma)*(x-mu)
    */
    HessianFactor(Index j, const Vector& mu, const Matrix& Sigma);

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

    /** Convert from a JacobianFactor (computes A^T * A) or HessianFactor */
    HessianFactor(const GaussianFactor& factor);

    /** Special constructor used in EliminateCholesky which combines the given factors */
    HessianFactor(const FactorGraph<GaussianFactor>& factors,
				const std::vector<size_t>& dimensions, const Scatter& scatter);

    /** Destructor */
		virtual ~HessianFactor() {}

    /** Print the factor for debugging and testing (implementing Testable) */
    virtual void print(const std::string& s = "") const;

    /** Compare to another factor for testing (implementing Testable) */
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

    /** Evaluate the factor error f(x), see above. */
    virtual double error(const VectorValues& c) const; /** 0.5*[x -1]'*H*[x -1] (also see constructor documentation) */

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     * @param variable An iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     */
    virtual size_t getDim(const_iterator variable) const { return info_(variable-this->begin(), 0).rows(); }

    /** Return the number of columns and rows of the Hessian matrix */
    size_t rows() const { return info_.rows(); }

    /** Return a view of the block at (j1,j2) of the information matrix \f$ H \f$, no data is copied.
     * @param j1 Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @param j2 Which block column to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return A view of the requested block, not a copy.
     */
    constBlock info(const_iterator j1, const_iterator j2) const { return info_(j1-begin(), j2-begin()); }

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    double constantTerm() const;

    /** Return the part of linear term \f$ g \f$ as described above corresponding to the requested variable.
     * @param j Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return The linear term \f$ g \f$ */
    constColumn linearTerm(const_iterator j) const { return info_.column(j-begin(), size(), 0); }

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    constColumn linearTerm() const;

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
    friend class ::Constructor1bHessianFactorTest;
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

