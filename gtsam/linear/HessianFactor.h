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
 * @date    Dec 8, 2010
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
   * @brief A Gaussian factor using the canonical parameters (information form)
   *
   * HessianFactor implements a general quadratic factor of the form
   * \f[ E(x) = 0.5 x^T G x - x^T g + 0.5 f \f]
   * that stores the matrix \f$ G \f$, the vector \f$ g \f$, and the constant term \f$ f \f$.
   *
   * When \f$ G \f$ is positive semidefinite, this factor represents a Gaussian,
   * in which case \f$ G \f$ is the information matrix \f$ \Lambda \f$,
   * \f$ g \f$ is the information vector \f$ \eta \f$, and \f$ f \f$ is the residual
   * sum-square-error at the mean, when \f$ x = \mu \f$.
   *
   * Indeed, the negative log-likelihood of a Gaussian is (up to a constant)
   * @f$ E(x) = 0.5(x-\mu)^T P^{-1} (x-\mu) @f$
   * with @f$ \mu @f$ the mean and  @f$ P @f$ the covariance matrix. Expanding the product we get
   * @f[
   * E(x) = 0.5 x^T P^{-1} x - x^T P^{-1} \mu + 0.5 \mu^T P^{-1} \mu
   * @f]
   * We define the Information matrix (or Hessian) @f$ \Lambda = P^{-1} @f$
   * and the information vector @f$ \eta = P^{-1} \mu = \Lambda \mu @f$
   * to arrive at the canonical form of the Gaussian:
   * @f[
   * E(x) = 0.5 x^T \Lambda x - x^T \eta + 0.5 \mu^T \Lambda \mu
   * @f]
   *
   * This factor is one of the factors that can be in a GaussianFactorGraph.
   * It may be returned from NonlinearFactor::linearize(), but is also
   * used internally to store the Hessian during Cholesky elimination.
   *
   * This can represent a quadratic factor with characteristics that cannot be
   * represented using a JacobianFactor (which has the form
   * \f$ E(x) = \Vert Ax - b \Vert^2 \f$ and stores the Jacobian \f$ A \f$
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
    typedef GaussianFactor Base; ///< Typedef to base class

    InfoMatrix matrix_; ///< The full augmented information matrix, s.t. the quadratic error is 0.5*[x -1]'*H*[x -1]
    BlockInfo info_;    ///< The block view of the full information matrix.

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
     * term, and f the constant term.
		 * JacobianFactor error is \f[ 0.5* (Ax-b)' M (Ax-b) = 0.5*x'A'MAx - x'A'Mb + 0.5*b'Mb \f]
		 * HessianFactor  error is \f[ 0.5*(x'Gx - 2x'g + f) = 0.5*x'Gx    - x'*g   + 0.5*f    \f]
		 * So, with \f$ A = [A1 A2] \f$ and \f$ G=A*'M*A = [A1';A2']*M*[A1 A2] \f$ we have
		 \code
		  n1*n1 G11 = A1'*M*A1
		  n1*n2 G12 = A1'*M*A2
		  n2*n2 G22 = A2'*M*A2
		  n1*1   g1 = A1'*M*b
		  n2*1   g2 = A2'*M*b
		   1*1    f =  b'*M*b
		 \endcode
     */
    HessianFactor(Index j1, Index j2,
        const Matrix& G11, const Matrix& G12, const Vector& g1,
        const Matrix& G22, const Vector& g2, double f);

    /** Construct a ternary factor.  Gxx are the upper-triangle blocks of the
     * quadratic term (the Hessian matrix), gx the pieces of the linear vector
     * term, and f the constant term.
     */
    HessianFactor(Index j1, Index j2, Index j3,
        const Matrix& G11, const Matrix& G12, const Matrix& G13, const Vector& g1,
        const Matrix& G22, const Matrix& G23, const Vector& g2,
        const Matrix& G33, const Vector& g3, double f);

    /** Construct an n-way factor.  Gs contains the upper-triangle blocks of the
     * quadratic term (the Hessian matrix) provided in row-order, gs the pieces
     * of the linear vector term, and f the constant term.
     */
    HessianFactor(const std::vector<Index>& js, const std::vector<Matrix>& Gs,
        const std::vector<Vector>& gs, double f);

    /** Construct from Conditional Gaussian */
    explicit HessianFactor(const GaussianConditional& cg);

    /** Convert from a JacobianFactor (computes A^T * A) or HessianFactor */
    explicit HessianFactor(const GaussianFactor& factor);

    /** Special constructor used in EliminateCholesky which combines the given factors */
    HessianFactor(const FactorGraph<GaussianFactor>& factors,
				const std::vector<size_t>& dimensions, const Scatter& scatter);

    /** Destructor */
		virtual ~HessianFactor() {}

    /** Aassignment operator */
		HessianFactor& operator=(const HessianFactor& rhs);

		/** Clone this JacobianFactor */
		virtual GaussianFactor::shared_ptr clone() const {
		  return boost::static_pointer_cast<GaussianFactor>(
		      shared_ptr(new HessianFactor(*this)));
		}

    /** Print the factor for debugging and testing (implementing Testable) */
    virtual void print(const std::string& s = "",
    		const IndexFormatter& formatter = DefaultIndexFormatter) const;

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

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    virtual GaussianFactor::shared_ptr negate() const;

    /** Return a view of the block at (j1,j2) of the <em>upper-triangular part</em> of the
     * information matrix \f$ H \f$, no data is copied.  See HessianFactor class documentation
     * above to explain that only the upper-triangular part of the information matrix is stored
     * and returned by this function.
     * @param j1 Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @param j2 Which block column to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return A view of the requested block, not a copy.
     */
    constBlock info(const_iterator j1, const_iterator j2) const { return info_(j1-begin(), j2-begin()); }

    /** Return a view of the block at (j1,j2) of the <em>upper-triangular part</em> of the
     * information matrix \f$ H \f$, no data is copied.  See HessianFactor class documentation
     * above to explain that only the upper-triangular part of the information matrix is stored
     * and returned by this function.
     * @param j1 Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @param j2 Which block column to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return A view of the requested block, not a copy.
     */
    Block info(iterator j1, iterator j2) { return info_(j1-begin(), j2-begin()); }

    /** Return the <em>upper-triangular part</em> of the full *augmented* information matrix,
     * as described above.  See HessianFactor class documentation above to explain that only the
     * upper-triangular part of the information matrix is stored and returned by this function.
     */
    constBlock info() const { return info_.full(); }

    /** Return the <em>upper-triangular part</em> of the full *augmented* information matrix,
     * as described above.  See HessianFactor class documentation above to explain that only the
     * upper-triangular part of the information matrix is stored and returned by this function.
     */
    Block info() { return info_.full(); }

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    double constantTerm() const { return info_(this->size(), this->size())(0,0); }

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    double& constantTerm() { return info_(this->size(), this->size())(0,0); }

    /** Return the part of linear term \f$ g \f$ as described above corresponding to the requested variable.
     * @param j Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return The linear term \f$ g \f$ */
    constColumn linearTerm(const_iterator j) const { return info_.column(j-begin(), size(), 0); }

    /** Return the part of linear term \f$ g \f$ as described above corresponding to the requested variable.
     * @param j Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return The linear term \f$ g \f$ */
    Column linearTerm(iterator j) { return info_.column(j-begin(), size(), 0); }

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    constColumn linearTerm() const { return info_.rangeColumn(0, this->size(), this->size(), 0); }

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    Column linearTerm() { return info_.rangeColumn(0, this->size(), this->size(), 0); }
    
    /** Return the augmented information matrix represented by this GaussianFactor.
     * The augmented information matrix contains the information matrix with an
     * additional column holding the information vector, and an additional row
     * holding the transpose of the information vector.  The lower-right entry
     * contains the constant error term (when \f$ \delta x = 0 \f$).  The
     * augmented information matrix is described in more detail in HessianFactor,
     * which in fact stores an augmented information matrix.
     *
     * For HessianFactor, this is the same as info() except that this function
     * returns a complete symmetric matrix whereas info() returns a matrix where
     * only the upper triangle is valid, but should be interpreted as symmetric.
     * This is because info() returns only a reference to the internal
     * representation of the augmented information matrix, which stores only the
     * upper triangle.
     */
    virtual Matrix computeInformation() const;

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

    /** split partially eliminated factor */
    boost::shared_ptr<GaussianConditional> splitEliminatedFactor(size_t nrFrontals);

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

