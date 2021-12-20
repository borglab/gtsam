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

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/Scatter.h>
#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/base/FastVector.h>

#include <boost/make_shared.hpp>

namespace gtsam {

  // Forward declarations
  class Ordering;
  class JacobianFactor;
  class HessianFactor;
  class GaussianConditional;
  class GaussianBayesNet;
  class GaussianFactorGraph;

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
  class GTSAM_EXPORT HessianFactor : public GaussianFactor {
  protected:

    SymmetricBlockMatrix info_; ///< The full augmented information matrix, s.t. the quadratic error is 0.5*[x -1]'*H*[x -1]

  public:

    typedef GaussianFactor Base; ///< Typedef to base class
    typedef HessianFactor This; ///< Typedef to this class
    typedef boost::shared_ptr<This> shared_ptr; ///< A shared_ptr to this class
    typedef SymmetricBlockMatrix::Block Block; ///< A block from the Hessian matrix
    typedef SymmetricBlockMatrix::constBlock constBlock; ///< A block from the Hessian matrix (const version)


    /** default constructor for I/O */
    HessianFactor();

    /** Construct a unary factor.  G is the quadratic term (Hessian matrix), g
     * the linear term (a vector), and f the constant term.  The quadratic
     * error is:
     * 0.5*(f - 2*x'*g + x'*G*x)
     */
    HessianFactor(Key j, const Matrix& G, const Vector& g, double f);

    /** Construct a unary factor, given a mean and covariance matrix.
     * error is 0.5*(x-mu)'*inv(Sigma)*(x-mu)
    */
    HessianFactor(Key j, const Vector& mu, const Matrix& Sigma);

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
    HessianFactor(Key j1, Key j2,
        const Matrix& G11, const Matrix& G12, const Vector& g1,
        const Matrix& G22, const Vector& g2, double f);

    /** Construct a ternary factor.  Gxx are the upper-triangle blocks of the
     * quadratic term (the Hessian matrix), gx the pieces of the linear vector
     * term, and f the constant term.
     */
    HessianFactor(Key j1, Key j2, Key j3,
        const Matrix& G11, const Matrix& G12, const Matrix& G13, const Vector& g1,
        const Matrix& G22, const Matrix& G23, const Vector& g2,
        const Matrix& G33, const Vector& g3, double f);

    /** Construct an n-way factor.  Gs contains the upper-triangle blocks of the
     * quadratic term (the Hessian matrix) provided in row-order, gs the pieces
     * of the linear vector term, and f the constant term.
     */
    HessianFactor(const KeyVector& js, const std::vector<Matrix>& Gs,
        const std::vector<Vector>& gs, double f);

    /** Constructor with an arbitrary number of keys and with the augmented information matrix
    *   specified as a block matrix. */
    template<typename KEYS>
    HessianFactor(const KEYS& keys, const SymmetricBlockMatrix& augmentedInformation);

    /** Construct from a JacobianFactor (or from a GaussianConditional since it derives from it) */
    explicit HessianFactor(const JacobianFactor& cg);

    /** Attempt to construct from any GaussianFactor - currently supports JacobianFactor,
     *  HessianFactor, GaussianConditional, or any derived classes. */
    explicit HessianFactor(const GaussianFactor& factor);

    /** Combine a set of factors into a single dense HessianFactor */
    explicit HessianFactor(const GaussianFactorGraph& factors,
      const Scatter& scatter);

    /** Combine a set of factors into a single dense HessianFactor */
    explicit HessianFactor(const GaussianFactorGraph& factors)
        : HessianFactor(factors, Scatter(factors)) {}

    /** Destructor */
    ~HessianFactor() override {}

    /** Clone this HessianFactor */
    GaussianFactor::shared_ptr clone() const override {
      return boost::make_shared<HessianFactor>(*this); }

    /** Print the factor for debugging and testing (implementing Testable) */
    void print(const std::string& s = "",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override;

    /** Compare to another factor for testing (implementing Testable) */
    bool equals(const GaussianFactor& lf, double tol = 1e-9) const override;

    /** 
     * Evaluate the factor error f(x). 
     * returns 0.5*[x -1]'*H*[x -1] (also see constructor documentation)
     */
    double error(const VectorValues& c) const override;

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     * @param variable An iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     */
    DenseIndex getDim(const_iterator variable) const override {
      return info_.getDim(std::distance(begin(), variable));
    }

    /** Return the number of columns and rows of the Hessian matrix, including the information vector. */
    size_t rows() const { return info_.rows(); }

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    GaussianFactor::shared_ptr negate() const override;

    /** Check if the factor is empty.  TODO: How should this be defined? */
    bool empty() const override { return size() == 0 /*|| rows() == 0*/; }

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    double constantTerm() const {
      const auto view = info_.diagonalBlock(size());
      return view(0, 0);
    }

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    double& constantTerm() { return info_.diagonalBlock(size())(0, 0); }

    /** Return the part of linear term \f$ g \f$ as described above corresponding to the requested variable.
     * @param j Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return The linear term \f$ g \f$ */
    SymmetricBlockMatrix::constBlock linearTerm(const_iterator j) const {
      assert(!empty());
      return info_.aboveDiagonalBlock(j - begin(), size());
    }

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    SymmetricBlockMatrix::constBlock linearTerm() const {
      assert(!empty());
      // get the last column (except the bottom right block)
      return info_.aboveDiagonalRange(0, size(), size(), size() + 1);
    }

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    SymmetricBlockMatrix::Block linearTerm() {
      assert(!empty());
      return info_.aboveDiagonalRange(0, size(), size(), size() + 1);
    }

    /// Return underlying information matrix.
    const SymmetricBlockMatrix& info() const { return info_; }

    /// Return non-const information matrix.
    /// TODO(gareth): Review the sanity of having non-const access to this.
    SymmetricBlockMatrix& info() { return info_; }

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
    Matrix augmentedInformation() const override;

    /// Return self-adjoint view onto the information matrix (NOT augmented).
    Eigen::SelfAdjointView<SymmetricBlockMatrix::constBlock, Eigen::Upper> informationView() const;

    /** Return the non-augmented information matrix represented by this
     * GaussianFactor.
     */
    Matrix information() const override;

    /// Add the current diagonal to a VectorValues instance
    void hessianDiagonalAdd(VectorValues& d) const override;

    /// Using the base method
    using Base::hessianDiagonal;

    /// Raw memory access version of hessianDiagonal
    void hessianDiagonal(double* d) const override;

    /// Return the block diagonal of the Hessian for this factor
    std::map<Key,Matrix> hessianBlockDiagonal() const override;

    /// Return (dense) matrix associated with factor
    std::pair<Matrix, Vector> jacobian() const override;

    /**
     * Return (dense) matrix associated with factor
     * The returned system is an augmented matrix: [A b]
     * @param set weight to use whitening to bake in weights
     */
    Matrix augmentedJacobian() const override;

    /** Update an information matrix by adding the information corresponding to this factor
     * (used internally during elimination).
     * @param keys THe ordered vector of keys for the information matrix to be updated
     * @param info The information matrix to be updated
     */
    void updateHessian(const KeyVector& keys, SymmetricBlockMatrix* info) const override;

    /** Update another Hessian factor
     * @param other the HessianFactor to be updated
     */
    void updateHessian(HessianFactor* other) const {
      assert(other);
      updateHessian(other->keys_, &other->info_);
    }

    /** y += alpha * A'*A*x */
    void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const override;

    /// eta for Hessian
    VectorValues gradientAtZero() const override;

    /// Raw memory access version of gradientAtZero
    void gradientAtZero(double* d) const override;

    /**
     * Compute the gradient at a key:
     *      \grad f(x_i) = \sum_j G_ij*x_j - g_i
     */
    Vector gradient(Key key, const VectorValues& x) const override;

    /**
     *  In-place elimination that returns a conditional on (ordered) keys specified, and leaves
     *  this factor to be on the remaining keys (separator) only. Does dense partial Cholesky.
     */
    boost::shared_ptr<GaussianConditional> eliminateCholesky(const Ordering& keys);

      /// Solve the system A'*A delta = A'*b in-place, return delta as VectorValues
    VectorValues solve();

 private:
    /// Allocate for given scatter pattern
    void Allocate(const Scatter& scatter);

    /// Constructor with given scatter pattern, allocating but not initializing storage.
    HessianFactor(const Scatter& scatter);

    friend class NonlinearFactorGraph;
    friend class NonlinearClusterTree;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GaussianFactor);
      ar & BOOST_SERIALIZATION_NVP(info_);
    }
  };

/**
*   Densely partially eliminate with Cholesky factorization.  JacobianFactors are
*   left-multiplied with their transpose to form the Hessian using the conversion constructor
*   HessianFactor(const JacobianFactor&).
*
*   If any factors contain constrained noise models, this function will fail because our current
*   implementation cannot handle constrained noise models in Cholesky factorization.  The
*   function EliminatePreferCholesky() automatically does QR instead when this is the case.
*
*   Variables are eliminated in the order specified in \c keys.
*
*   @param factors Factors to combine and eliminate
*   @param keys The variables to eliminate and their elimination ordering
*   @return The conditional and remaining factor
*
*   \addtogroup LinearSolving */
GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditional>, boost::shared_ptr<HessianFactor> >
  EliminateCholesky(const GaussianFactorGraph& factors, const Ordering& keys);

/**
*   Densely partially eliminate with Cholesky factorization.  JacobianFactors are
*   left-multiplied with their transpose to form the Hessian using the conversion constructor
*   HessianFactor(const JacobianFactor&).
*
*   This function will fall back on QR factorization for any cliques containing JacobianFactor's
*   with constrained noise models.
*
*   Variables are eliminated in the order specified in \c keys.
*
*   @param factors Factors to combine and eliminate
*   @param keys The variables to eliminate and their elimination ordering
*   @return The conditional and remaining factor
*
*   \addtogroup LinearSolving */
GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditional>, boost::shared_ptr<GaussianFactor> >
  EliminatePreferCholesky(const GaussianFactorGraph& factors, const Ordering& keys);

/// traits
template<>
struct traits<HessianFactor> : public Testable<HessianFactor> {};

} // \ namespace gtsam


#include <gtsam/linear/HessianFactor-inl.h>
