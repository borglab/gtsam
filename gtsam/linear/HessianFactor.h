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

#include <stdlib.h>

#include <tuple>

#include <algorithm>

#include <gtsam/inference/HypoTree.h>

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
    HessianFactor(const std::vector<Key>& js, const std::vector<Matrix>& Gs,
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
      boost::optional<const Scatter&> scatter = boost::none); 

    /** Destructor */
    virtual ~HessianFactor() {}

    /** Clone this HessianFactor */
    virtual GaussianFactor::shared_ptr clone() const {
      return boost::make_shared<HessianFactor>(*this); }
    
    //[MH-A]: fit into MH format
    virtual size_t hypoSize() const {
        return 1;
    }
    //[MH-A]: fit into MH format
    const HypoList& getHypoList() const {
      std::cout << "HessianFactor::getHypoList() return empty" << std::endl;
      HypoList* empty_list = new HypoList();
      return *empty_list; //should NEVER be called
    }
    //[MH-A]: fit into MH format
    const HypoLayer* getHypoLayer() const {
      std::cout << "const HessianFactor::getHypoLayer() return NULL" << std::endl;
      return NULL;
    }
    //[MH-A]: fit into MH format
    HypoLayer* getHypoLayer() {
      std::cout << "HessianFactor::getHypoLayer() return NULL" << std::endl;
      return NULL;
    }
    
    virtual const void showType() const {
      std::cout << "show: HF" << std::endl;
    }
    
    //[MH-C]: fit into MH format
    virtual bool removeAccumulatedPruned() {
      std::cout << "HessianFactor::removeAccumulatedPruned() return false" << std::endl;
      return false;
    }

    /** Print the factor for debugging and testing (implementing Testable) */
    virtual void print(const std::string& s = "",
        const KeyFormatter& formatter = DefaultKeyFormatter) const;

    /** Compare to another factor for testing (implementing Testable) */
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

    /** Evaluate the factor error f(x), see above. */
    virtual double error(const VectorValues& c) const; /** 0.5*[x -1]'*H*[x -1] (also see constructor documentation) */

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     * @param variable An iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     */
    virtual DenseIndex getDim(const_iterator variable) const {
      return info_.getDim(std::distance(begin(), variable));
    }

    /** Return the number of columns and rows of the Hessian matrix, including the information vector. */
    size_t rows() const { return info_.rows(); }

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    virtual GaussianFactor::shared_ptr negate() const;

    /** Check if the factor is empty.  TODO: How should this be defined? */
    virtual bool empty() const { return size() == 0 /*|| rows() == 0*/; }

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
    virtual Matrix augmentedInformation() const;

    /// Return self-adjoint view onto the information matrix (NOT augmented).
    Eigen::SelfAdjointView<SymmetricBlockMatrix::constBlock, Eigen::Upper> informationView() const;

    /** Return the non-augmented information matrix represented by this
     * GaussianFactor.
     */
    virtual Matrix information() const;

    /// Return the diagonal of the Hessian for this factor
    virtual VectorValues hessianDiagonal() const;

    /// Raw memory access version of hessianDiagonal
    virtual void hessianDiagonal(double* d) const;

    /// Return the block diagonal of the Hessian for this factor
    virtual std::map<Key,Matrix> hessianBlockDiagonal() const;

    /// Return (dense) matrix associated with factor
    virtual std::pair<Matrix, Vector> jacobian() const;

    /**
     * Return (dense) matrix associated with factor
     * The returned system is an augmented matrix: [A b]
     * @param set weight to use whitening to bake in weights
     */
    virtual Matrix augmentedJacobian() const;

    /** Update an information matrix by adding the information corresponding to this factor
     * (used internally during elimination).
     * @param keys THe ordered vector of keys for the information matrix to be updated
     * @param info The information matrix to be updated
     */
    void updateHessian(const FastVector<Key>& keys, SymmetricBlockMatrix* info) const;
    
    //[MH-A]: virtual only 
    virtual void mhUpdateHessian(const FastVector<Key>& keys, HessList& hessian_list) const {
      std::cout << "ERROR: HessianFactor::mhUpdateHessian() should NEVER be called!!" << std::endl;
    }

    /** Update another Hessian factor
     * @param other the HessianFactor to be updated
     */
    void updateHessian(HessianFactor* other) const {
      assert(other);
      updateHessian(other->keys_, &other->info_);
    }

    /** y += alpha * A'*A*x */
    void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const;

    /// eta for Hessian
    VectorValues gradientAtZero() const;

    /// Raw memory access version of gradientAtZero
    virtual void gradientAtZero(double* d) const;

    /**
     * Compute the gradient at a key:
     *      \grad f(x_i) = \sum_j G_ij*x_j - g_i
     */
    Vector gradient(Key key, const VectorValues& x) const;

    /**
     *  In-place elimination that returns a conditional on (ordered) keys specified, and leaves
     *  this factor to be on the remaining keys (separator) only. Does dense partial Cholesky.
     */
    boost::shared_ptr<GaussianConditional> eliminateCholesky(const Ordering& keys);

      /// Solve the system A'*A delta = A'*b in-place, return delta as VectorValues
    VectorValues solve();


#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
    /// @name Deprecated
    /// @{
    const SymmetricBlockMatrix& matrixObject() const { return info_; }
    /// @}
#endif

  //[MH-A]: have to access this when constructing MHHessianFactor
  //private:
  public:
    /// Allocate for given scatter pattern
    void Allocate(const Scatter& scatter);
  
  private:

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



//================================ MHHessianFactor and MH-functions ===========================
  //[MH-A]:
  class GTSAM_EXPORT MHHessianFactor : public HessianFactor {
  public:

    //typedef GaussianFactor Base; ///< Typedef to base class
    typedef HessianFactor Base; ///< Typedef to base class
    
    typedef MHHessianFactor This; ///< Typedef to this class
    typedef boost::shared_ptr<This> shared_ptr; ///< A shared_ptr to this class
    typedef SymmetricBlockMatrix::Block Block; ///< A block from the Hessian matrix
    typedef SymmetricBlockMatrix::constBlock constBlock; ///< A block from the Hessian matrix (const version)

    typedef std::list<HypoNode*> HypoList;
    typedef std::list<HessianFactor> HessList;
    typedef typename HypoList::iterator HypoListIter;
    typedef typename HessList::iterator HessListIter;
    typedef typename HypoList::const_iterator HypoListCstIter;
    typedef typename HessList::const_iterator HessListCstIter;

    typedef std::vector<PruningRecord*> RecordArr;
   
    typedef std::list<std::pair<double, HypoNode*>  > ErrHypoList;
    typedef typename ErrHypoList::iterator ErrHypoListIter;
    typedef typename ErrHypoList::const_iterator ErrHypoListCstIter;
    
    typedef std::tuple<double, HypoNode*, size_t> PruneObj;
    typedef std::list<PruneObj> PruneList;
    typedef typename PruneList::iterator PruneListIter;
    typedef typename PruneList::const_iterator PruneListCstIter;

  public: //for convenience to access these two lists...

    HessList hessian_list_;

    HypoLayer* resulting_layer_;

  public:
    /** default constructor for I/O */
    MHHessianFactor();

    /** Construct a unary factor.  G is the quadratic term (Hessian matrix), g
     * the linear term (a vector), and f the constant term.  The quadratic
     * error is:
     * 0.5*(f - 2*x'*g + x'*G*x)
     */
    //HessianFactor(Key j, const Matrix& G, const Vector& g, double f);

    /** Construct a unary factor, given a mean and covariance matrix.
     * error is 0.5*(x-mu)'*inv(Sigma)*(x-mu)
    */
    //HessianFactor(Key j, const Vector& mu, const Matrix& Sigma);

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
    /*
    HessianFactor(Key j1, Key j2,
        const Matrix& G11, const Matrix& G12, const Vector& g1,
        const Matrix& G22, const Vector& g2, double f);
    // */
    /** Construct a ternary factor.  Gxx are the upper-triangle blocks of the
     * quadratic term (the Hessian matrix), gx the pieces of the linear vector
     * term, and f the constant term.
     */
    /*
    HessianFactor(Key j1, Key j2, Key j3,
        const Matrix& G11, const Matrix& G12, const Matrix& G13, const Vector& g1,
        const Matrix& G22, const Matrix& G23, const Vector& g2,
        const Matrix& G33, const Vector& g3, double f);
    // */
    /** Construct an n-way factor.  Gs contains the upper-triangle blocks of the
     * quadratic term (the Hessian matrix) provided in row-order, gs the pieces
     * of the linear vector term, and f the constant term.
     */
    /*
    HessianFactor(const std::vector<Key>& js, const std::vector<Matrix>& Gs,
        const std::vector<Vector>& gs, double f);
    // */
    /** Constructor with an arbitrary number of keys and with the augmented information matrix
    *   specified as a block matrix. */
    /*
    template<typename KEYS>
    HessianFactor(const KEYS& keys, const SymmetricBlockMatrix& augmentedInformation);
    // */
    /** Construct from a JacobianFactor (or from a GaussianConditional since it derives from it) */
    //explicit HessianFactor(const JacobianFactor& cg);

    /** Attempt to construct from any GaussianFactor - currently supports JacobianFactor,
     *  HessianFactor, GaussianConditional, or any derived classes. */
    //explicit HessianFactor(const GaussianFactor& factor);

    /** Combine a set of factors into a single dense HessianFactor */
    //[MH-A]: used in MHEliminateCholesky() in HessianFactor.h
    explicit MHHessianFactor(const GaussianFactorGraph& factors, boost::optional<const Scatter&> scatter = boost::none); 

    /** Destructor */
    virtual ~MHHessianFactor() {}

    /** Clone this HessianFactor */
    virtual GaussianFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<GaussianFactor>(
          boost::make_shared<MHHessianFactor>(*this)); //mshiao: can only be activated after all other functions are established...
    }
    
    virtual size_t hypoSize() const {
      return getHypoList().size();
    }
    const HypoList& getHypoList() const {
      return resulting_layer_->getNodeList();
    }
    const HypoLayer* getHypoLayer() const {
      return resulting_layer_;
    }
    HypoLayer* getHypoLayer() {
      return resulting_layer_;
    }
    
    virtual const void showType() const {
      std::cout << "show: MHHF" << std::endl;
    }

    /** Print the factor for debugging and testing (implementing Testable) */
    //virtual void print(const std::string& s = "", const KeyFormatter& formatter = DefaultKeyFormatter) const;
    //TODO: virtual
    //virtual void print(const std::string& s = "", const KeyFormatter& formatter = DefaultKeyFormatter) const {}

    /** Compare to another factor for testing (implementing Testable) */
    //virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;
    //TODO: virtual
    //virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const {}

    /** Evaluate the factor error f(x), see above. */
    //virtual double error(const VectorValues& c) const; /** 0.5*[x -1]'*H*[x -1] (also see constructor documentation) */
    //TODO: virtual
    //virtual double error(const VectorValues& c) const {} /** 0.5*[x -1]'*H*[x -1] (also see constructor documentation) */

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     * @param variable An iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     */
    virtual DenseIndex getDim(const_iterator variable) const {
      //MHHF
      return hessian_list_.front().info().getDim(std::distance(begin(), variable));
    }

    /** Return the number of columns and rows of the Hessian matrix, including the information vector. */
    //size_t rows() const { return info_.rows(); }

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    //virtual GaussianFactor::shared_ptr negate() const;

    /** Check if the factor is empty.  TODO: How should this be defined? */
    //virtual bool empty() const { return size() == 0 /*|| rows() == 0*/; }

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    /*
    double constantTerm() const {
      const auto view = info_.diagonalBlock(size());
      return view(0, 0);
    }
    // */

    /** Return the constant term \f$ f \f$ as described above
     * @return The constant term \f$ f \f$
     */
    //double& constantTerm() { return info_.diagonalBlock(size())(0, 0); }

    /** Return the part of linear term \f$ g \f$ as described above corresponding to the requested variable.
     * @param j Which block row to get, as an iterator pointing to the slot in this factor.  You can
     * use, for example, begin() + 2 to get the 3rd variable in this factor.
     * @return The linear term \f$ g \f$ */
    /*
    SymmetricBlockMatrix::constBlock linearTerm(const_iterator j) const {
      assert(!empty());
      return info_.aboveDiagonalBlock(j - begin(), size());
    }
    // */

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    /*
    SymmetricBlockMatrix::constBlock linearTerm() const {
      assert(!empty());
      // get the last column (except the bottom right block)
      return info_.aboveDiagonalRange(0, size(), size(), size() + 1);
    }
    // */

    /** Return the complete linear term \f$ g \f$ as described above.
     * @return The linear term \f$ g \f$ */
    /*
    SymmetricBlockMatrix::Block linearTerm() {
      assert(!empty());
      return info_.aboveDiagonalRange(0, size(), size(), size() + 1);
    }
    // */

    /// Return underlying information matrix.
    //const SymmetricBlockMatrix& info() const { return info_; }

    /// Return non-const information matrix.
    /// TODO(gareth): Review the sanity of having non-const access to this.
    //SymmetricBlockMatrix& info() { return info_; }

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
    //virtual Matrix augmentedInformation() const;
    //TODO: virtual
    //virtual Matrix augmentedInformation() const {}

    /// Return self-adjoint view onto the information matrix (NOT augmented).
    //Eigen::SelfAdjointView<SymmetricBlockMatrix::constBlock, Eigen::Upper> informationView() const;

    /** Return the non-augmented information matrix represented by this
     * GaussianFactor.
     */
    //virtual Matrix information() const;
    //TODO: virtual
    //virtual Matrix information() const {}

    /// Return the diagonal of the Hessian for this factor
    //virtual VectorValues hessianDiagonal() const;
    //TODO: virtual
    //virtual VectorValues hessianDiagonal() const {}

    /// Raw memory access version of hessianDiagonal
    //virtual void hessianDiagonal(double* d) const;
    //TODO: virtual
    //virtual void hessianDiagonal(double* d) const {}

    /// Return the block diagonal of the Hessian for this factor
    //virtual std::map<Key,Matrix> hessianBlockDiagonal() const;
    //TODO: virtual
    //virtual std::map<Key,Matrix> hessianBlockDiagonal() const {}

    /// Return (dense) matrix associated with factor
    //virtual std::pair<Matrix, Vector> jacobian() const;
    //TODO: virtual
    //virtual std::pair<Matrix, Vector> jacobian() const {}

    /**
     * Return (dense) matrix associated with factor
     * The returned system is an augmented matrix: [A b]
     * @param set weight to use whitening to bake in weights
     */
    //virtual Matrix augmentedJacobian() const;
    //TODO: virtual
    //virtual Matrix augmentedJacobian() const {}

    /** Update an information matrix by adding the information corresponding to this factor
     * (used internally during elimination).
     * @param keys THe ordered vector of keys for the information matrix to be updated
     * @param info The information matrix to be updated
     */
    //void updateHessian(const FastVector<Key>& keys, SymmetricBlockMatrix* info) const;
    //TODO: virtual
    //void updateHessian(const FastVector<Key>& keys, SymmetricBlockMatrix* info) const {}

    /** Update another Hessian factor
     * @param other the HessianFactor to be updated
     */
    /*
    void updateHessian(HessianFactor* other) const {
      assert(other);
      updateHessian(other->keys_, &other->info_);
    }
    // */

    //[MH-A]:
    void mhUpdateHessian(const FastVector<Key>& keys, HessList& hessian_list, const int& max_layer_idx) const;

    /** y += alpha * A'*A*x */
    //void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const;
    //TODO: virtual
    //void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const {}

    /// eta for Hessian
    //VectorValues gradientAtZero() const;
    //TODO: virtual
    //VectorValues gradientAtZero() const {}

    /// Raw memory access version of gradientAtZero
    //virtual void gradientAtZero(double* d) const;
    //TODO: virtual
    //virtual void gradientAtZero(double* d) const {}

    /**
     * Compute the gradient at a key:
     *      \grad f(x_i) = \sum_j G_ij*x_j - g_i
     */
    //Vector gradient(Key key, const VectorValues& x) const;
    //TODO: virtual
    //Vector gradient(Key key, const VectorValues& x) const {}

    /**
     *  In-place elimination that returns a conditional on (ordered) keys specified, and leaves
     *  this factor to be on the remaining keys (separator) only. Does dense partial Cholesky.
     */
    //[MH-A]:
    boost::shared_ptr<GaussianConditional> mhEliminateCholesky(const Ordering& keys);

      /// Solve the system A'*A delta = A'*b in-place, return delta as VectorValues
    //VectorValues solve();
    
    //[MH-C]: called in removeAccumulatedPruned()
    void eraseHessListAt(const std::vector<size_t>& idx_arr) {
      size_t count_idx = 0;
      size_t count_gv = 0;
      for (HessListIter hit = hessian_list_.begin(); hit != hessian_list_.end(); ++hit) {
        if (count_gv == idx_arr[count_idx]) {
          hit = hessian_list_.erase(hit);
          --hit;
          ++count_idx;
          if (count_idx == idx_arr.size()) {
            break;
          }
        }
        ++count_gv;
      }
    } // END eraseHessListAt()

    //[MH-C]:
    bool removeAccumulatedPruned() {
      HypoList& hypo_list = resulting_layer_->getNodeList();
      if (hessian_list_.size() == hypo_list.size()) {
        return false; //already up-to-date, no need any changes
      } else {
        RecordArr& record_arr = resulting_layer_->getRecordArr();
        
        for (size_t r = 0; r < record_arr.size(); ++r) {
        //for (size_t r = (record_arr.size() - 1); r >= 0; --r) {
        
          if (record_arr[r]->original_size_ == hessian_list_.size()) {
            for (size_t t = r; t < record_arr.size(); ++t) {
              eraseHessListAt(record_arr[t]->pruned_idx_arr_);
            }
            return true;
          }
        }
        std::cout << "ERROR: MHHF::removeAccumulatedPruned() should match exactly one original_size_" << std::endl;
        return false;
      }
    } // END removeAccumulatedPruned()

    //[MH-C]:
    double mhGet_single_err2(HessListCstIter& hit) const {
      
      auto& info_mat = ((*hit).matrixObject()).matrix_;
      
      auto n_r = info_mat.rows();
      auto n_c = info_mat.cols();

      // The bottom-right corner... 
      return std::abs( info_mat( (n_r-1), (n_c-1) ) );
    }

    //[MH-C]: 
    void printAllHypo() {
      
      //TODO: might be necessary?
      removeAccumulatedPruned();

      const size_t& common_dim = getHypoLayer()->getBelongTreePtr()->common_dim_;
      const HypoList& hypo_list = getHypoList(); 

      std::cout << "printAllHypo(): " << hypo_list.size() << std::endl;
      std::cout << "err2  acc_dim  chi2  err2/chi2" << hypo_list.size() << std::endl;
      
      HypoListCstIter it = hypo_list.begin();
      for (HessListCstIter hit = hessian_list_.begin(); hit != hessian_list_.end(); ++hit, ++it) {
        
        const size_t acc_dim = (*it)->accumulated_dim_;
        const double err2 = mhGet_single_err2(hit); 
        const double chi2 = HypoLayer::getChi2(acc_dim + common_dim);
        std::cout << err2 << " ";
        std::cout << acc_dim << " ";
        std::cout << chi2 << " ";
        std::cout << err2/chi2 << std::endl;

      }
      std::cout << std::endl;
    }
    
    //[MH-C]:
    static bool sortPruneListByDim(const PruneObj& lhs, const PruneObj& rhs) {
      return (std::get<2>(lhs) > std::get<2>(rhs)); //larger Dim is better!
    }
    
     
    //[MH-C]: Define pruning criteria here!!!!
    bool selectPruneList(PruneList& prune_list, const size_t& max_hypo_num, const size_t& ult_hypo_bound, const bool& is_strict_th, const bool& is_print_details) const { //(false: max(), false: 0.0)
     
      // Current format of prune_list: <(err2/chi2), hypo, dim>
      //TODO: Replace err2/chi2 with chi2 confidence...
      // Desired format of prune_list: <p_value, hypo, dim>

      if ( !(is_strict_th) && (hypoSize() <= max_hypo_num) ) {
        
        return false; //no need to do any pruning...
      }
      
      //[MH-E]: Create candidate_list
      //[MH-E]: Move all err > chi2 from candidate_list to prune_list
      PruneList candidate_list;
      
      const size_t common_dim = getHypoLayer()->getBelongTreePtr()->common_dim_;
      const HypoList& hypo_list = getHypoList();
      HypoListCstIter it = hypo_list.begin();
      
      if (getHypoLayer()->getLayerIdx() != getHypoLayer()->getBelongTreePtr()->getLastLayer().getLayerIdx()) {
        std::cout << "ERROR: the HypoLayer of the HessianFactor for pruning (" << getHypoLayer()->getLayerIdx() << ") != the latest HypoLayer (" << getHypoLayer()->getBelongTreePtr()->getLastLayer().getLayerIdx() << ") !!" << std::endl; 
      }

      for (HessListCstIter hit = hessian_list_.begin(); hit != hessian_list_.end(); ++hit) {
        const size_t sum_dim = (*it)->accumulated_dim_ + common_dim;
        const double chi2_th = HypoLayer::getChi2(sum_dim);
        const double err2 = mhGet_single_err2(hit);
        
        //WARNING: using err2/chi2_th to approximate boost::math::cdf(DoF, err2)
        if (err2 > chi2_th) { //should be pruned
          prune_list.push_back(std::make_tuple( err2/chi2_th, (*it), sum_dim ));
          
        } else { //should go to next round
          candidate_list.push_back(std::make_tuple( err2/chi2_th, (*it), sum_dim ));
        }

        ++it;
      }
      
      // DEBUG only
      if (is_print_details) {
        //
        std::cout << "==== prune_list (err2 > chi2_th): " << prune_list.size() << " ====" << std::endl;
        for (PruneList::iterator pit = prune_list.begin(); pit != prune_list.end(); ++pit) {
          std::cout << "[P] err2/chi2_th: " << std::get<0>(*pit);
          std::cout << ", dim: " << std::get<2>(*pit) << std::endl;
        }
        //
        std::cout << "==== candidate_list (err2 <= chi2_th): " << candidate_list.size() << " ====" << std::endl;
        for (PruneList::iterator cit = candidate_list.begin(); cit != candidate_list.end(); ++cit) {
          std::cout << "[C] err2/chi2_th: " << std::get<0>(*cit);
          std::cout << ", dim: " << std::get<2>(*cit) << std::endl;
        }
      }
      // END DEBUG
      
      if (candidate_list.empty()) { //all exceed chi2_th
        std::cout << "ERROR: All hypos exceed the chi2_th! Keep track of " << max_hypo_num << " hypos with smallest err2/chi2_th..." << std::endl;
        if (prune_list.size() > max_hypo_num) {
          
          // Sorting based on err2/chi2 again
          prune_list.sort();
          
          for (size_t i = 0; i < max_hypo_num; ++i) {
            prune_list.pop_front();
          }
          return true;
        } else {
          return false;
        }
      }
      
      if (candidate_list.size() > max_hypo_num) { //# remaining hypos > max_hypo_num
        //[MH-E]: Sort candidate_list based on dim
        
        candidate_list.sort(sortPruneListByDim);
        
        const size_t max_dim = std::get<2>(candidate_list.front());
        PruneListIter cit = candidate_list.begin();
        for (size_t i = 0; i < max_hypo_num; ++i) {
          ++cit;
        }
        // Get min_dim
        const size_t min_dim = std::get<2>(*cit);
        
        // Use min_dim to cut off rest in candidate_list
        while ((!candidate_list.empty()) && std::get<2>(candidate_list.back()) < min_dim) {
          prune_list.push_back(candidate_list.back());
          candidate_list.pop_back();
        }
        if (candidate_list.size() > ult_hypo_bound) { //too many remaining hypos
          if (max_dim == min_dim) { //all of the remaining dim of hypos are the same
            // Sorting based on err2/chi2 again
            candidate_list.sort();

            for (size_t i = 0; i < ult_hypo_bound; ++i) { //keep more possible hypos
              candidate_list.pop_front();
            }
            while (!candidate_list.empty()) {
              prune_list.push_back(candidate_list.back());
              candidate_list.pop_back();
            }            
          } else {
            while (std::get<2>(candidate_list.back()) <= min_dim) { //actually == is okay here...
              prune_list.push_back(candidate_list.back());
              candidate_list.pop_back();
            }
          }
        }
      }
      
      if (prune_list.empty()) { //all err2 <= chi2_th && all dim >= min_dim 
        return false;
      } else { //some err2 > chi2_th || some dim < min_dim
        return true;
      }

    } // END selectPruneList()

    //[MH-C]:
    HypoNode* findBestHypo(const size_t& best_type) {
      
      if (getHypoList().size() != hessian_list_.size()) {
         std::cout << "ERROR: Sizes of two lists do NOT match: " << getHypoList().size() << " " << hessian_list_.size() << std::endl;
         return NULL;
      }

      if (best_type == 0) { //Best: min err2
        double min_err2 = std::numeric_limits<double>::max();
        HypoListCstIter best_hypo_iter;
        HypoListCstIter it = getHypoList().begin();
        for (HessListCstIter hit = hessian_list_.begin(); hit != hessian_list_.end(); ++hit, ++it) {
          const double err2 = mhGet_single_err2(hit);
          if (err2 < min_err2) {
            min_err2 = err2;
            best_hypo_iter = it;
          }
        }
        if (min_err2 == std::numeric_limits<double>::max()) {
            std::cout << "ERROR: Best Hypo NOT found!!" << std::endl;
            return NULL;
        }
        return (*best_hypo_iter);

      } else {
        //TODO: other types of "Best"
        std::cout << "ERROR: Best types other than 0 are NOT implemented yet!!" << std::endl;
        return NULL;
      }
    
    } // END findBestHypo

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
    /// @name Deprecated
    /// @{
    //const SymmetricBlockMatrix& matrixObject() const { return info_; }
    /// @}
#endif

  private:

    /// Allocate for given scatter pattern
    //[MH-A]: setup keys_ here
    void mhAllocate(const Scatter& scatter);

    /// Constructor with given scatter pattern, allocating but not initializing storage.
    //HessianFactor(const Scatter& scatter);

    friend class NonlinearFactorGraph;
    friend class NonlinearClusterTree;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GaussianFactor);
      //ar & BOOST_SERIALIZATION_NVP(hypo_list_); //[D]
      ar & BOOST_SERIALIZATION_NVP(resulting_layer_);
      ar & BOOST_SERIALIZATION_NVP(hessian_list_);
    }
  }; // END MHHessianFactor

//[MH-A]: 
GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditional>, boost::shared_ptr<MHHessianFactor> >
  MHEliminateCholesky(const GaussianFactorGraph& factors, const Ordering& keys);

/// traits
template<>
struct traits<MHHessianFactor> : public Testable<MHHessianFactor> {};

//================================ END MHHessianFactor and MH-functions ===========================

} // \ namespace gtsam


#include <gtsam/linear/HessianFactor-inl.h>
