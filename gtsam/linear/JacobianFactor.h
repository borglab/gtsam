/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    JacobianFactor.h
 * @author  Richard Roberts
 * @author  Christian Potthast
 * @author  Frank Dellaert
 * @date    Dec 8, 2010
 */
#pragma once

#include <gtsam/inference/HypoTree.h>

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/global_includes.h>

#include <boost/make_shared.hpp>

namespace gtsam {

  // Forward declarations
  class HessianFactor;
  class VariableSlots;
  class GaussianFactorGraph;
  class GaussianConditional;
  class HessianFactor;
  class VectorValues;
  class Ordering;
  class JacobianFactor;

  /**
   * Multiply all factors and eliminate the given keys from the resulting factor using a QR
   * variant that handles constraints (zero sigmas). Computation happens in noiseModel::Gaussian::QR
   * Returns a conditional on those keys, and a new factor on the separator.
   */
  GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditional>, boost::shared_ptr<JacobianFactor> >
    EliminateQR(const GaussianFactorGraph& factors, const Ordering& keys);

  /**
   * A Gaussian factor in the squared-error form.
   *
   * JacobianFactor implements a
   * Gaussian, which has quadratic negative log-likelihood
   * \f[ E(x) = \frac{1}{2} (Ax-b)^T \Sigma^{-1} (Ax-b) \f]
   * where \f$ \Sigma \f$ is a \a diagonal covariance matrix.  The
   * matrix \f$ A \f$, r.h.s. vector \f$ b \f$, and diagonal noise model
   * \f$ \Sigma \f$ are stored in this class.
   *
   * This factor represents the sum-of-squares error of a \a linear
   * measurement function, and is created upon linearization of a NoiseModelFactor,
   * which in turn is a sum-of-squares factor with a nonlinear measurement function.
   *
   * Here is an example of how this factor represents a sum-of-squares error:
   *
   * Letting \f$ h(x) \f$ be a \a linear measurement prediction function, \f$ z \f$ be
   * the actual observed measurement, the residual is
   * \f[ f(x) = h(x) - z . \f]
   * If we expect noise with diagonal covariance matrix \f$ \Sigma \f$ on this
   * measurement, then the negative log-likelihood of the Gaussian induced by this
   * measurement model is
   * \f[ E(x) = \frac{1}{2} (h(x) - z)^T \Sigma^{-1} (h(x) - z) . \f]
   * Because \f$ h(x) \f$ is linear, we can write it as
   * \f[ h(x) = Ax + e \f]
   * and thus we have
   * \f[ E(x) = \frac{1}{2} (Ax-b)^T \Sigma^{-1} (Ax-b) \f]
   * where \f$ b = z - e \f$.
   *
   * This factor can involve an arbitrary number of variables, and in the
   * above example \f$ x \f$ would almost always be only be a subset of the variables
   * in the entire factor graph.  There are special constructors for 1-, 2-, and 3-
   * way JacobianFactors, and additional constructors for creating n-way JacobianFactors.
   * The Jacobian matrix \f$ A \f$ is passed to these constructors in blocks,
   * for example, for a 2-way factor, the constructor would accept \f$ A1 \f$ and \f$ A2 \f$,
   * as well as the variable indices \f$ j1 \f$ and \f$ j2 \f$
   * and the negative log-likelihood represented by this factor would be
   * \f[ E(x) = \frac{1}{2} (A_1 x_{j1} + A_2 x_{j2} - b)^T \Sigma^{-1} (A_1 x_{j1} + A_2 x_{j2} - b) . \f]
   */
  class GTSAM_EXPORT JacobianFactor : public GaussianFactor
  {
  public:

    typedef JacobianFactor This; ///< Typedef to this class
    typedef GaussianFactor Base; ///< Typedef to base class
    typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

    typedef VerticalBlockMatrix::Block ABlock;
    typedef VerticalBlockMatrix::constBlock constABlock;
    typedef ABlock::ColXpr BVector;
    typedef constABlock::ConstColXpr constBVector;

  //protected: //mhsiao: test only

    VerticalBlockMatrix Ab_;      // the block view of the full matrix

  protected: //mhsiao: test only

    noiseModel::Diagonal::shared_ptr model_; // Gaussian noise model with diagonal covariance matrix

  public:

    /** Convert from other GaussianFactor */
    explicit JacobianFactor(const GaussianFactor& gf);

    /** Copy constructor */
    JacobianFactor(const JacobianFactor& jf) : Base(jf), Ab_(jf.Ab_), model_(jf.model_) {}

    /** Conversion from HessianFactor (does Cholesky to obtain Jacobian matrix) */
    explicit JacobianFactor(const HessianFactor& hf);

    /** default constructor for I/O */
    JacobianFactor();

    /** Construct Null factor */
    explicit JacobianFactor(const Vector& b_in);

    /** Construct unary factor */
    JacobianFactor(Key i1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Construct binary factor */
    JacobianFactor(Key i1, const Matrix& A1,
        Key i2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Construct ternary factor */
    JacobianFactor(Key i1, const Matrix& A1, Key i2,
        const Matrix& A2, Key i3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Construct an n-ary factor
     * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
     *         collection of keys and matrices making up the factor. */
    template<typename TERMS>
    JacobianFactor(const TERMS& terms, const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
     *  instead of in block terms.  Note that only the active view of the provided augmented matrix
     *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
     *  factor. */
    template<typename KEYS>
    JacobianFactor(
      const KEYS& keys, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& sigmas = SharedDiagonal());

    /**
     * Build a dense joint factor from all the factors in a factor graph.  If a VariableSlots
     * structure computed for \c graph is already available, providing it will reduce the amount of
     * computation performed. */
    explicit JacobianFactor(
      const GaussianFactorGraph& graph,
      boost::optional<const Ordering&> ordering = boost::none,
      boost::optional<const VariableSlots&> p_variableSlots = boost::none);

    /** Virtual destructor */
    virtual ~JacobianFactor() {}

    /** Clone this JacobianFactor */
    virtual GaussianFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<GaussianFactor>(
          boost::make_shared<JacobianFactor>(*this));
    }
    
    //[MH-A]: fit into MH format
    virtual size_t hypoSize() const {
      return 1;
    }
    //[MH-A]: fit into MH format
    const HypoList& getHypoList() const {
      std::cout << "JacobianFactor::getHypoList() return empty" << std::endl;
      HypoList* empty_list = new HypoList();
      return *empty_list; //should NEVER be called 
    }
    //[MH-A]: fit into MH format
    const HypoLayer* getHypoLayer() const {
      std::cout << "const JacobianFactor::getHypoLayer() return NULL" << std::endl;
      return NULL;
    }
    //[MH-A]: fit into MH format
    HypoLayer* getHypoLayer() {
      std::cout << "JacobianFactor::getHypoLayer() return NULL" << std::endl;
      return NULL;
    }
    
    virtual const void showType() const {
      std::cout << "show: JF" << std::endl;
    }
    
    //[MH-C]: fit into MH format
    virtual bool removeAccumulatedPruned() {
      std::cout << "JacobianFactor::removeAccumulatedPruned() return false" << std::endl;
      return false;
    }

    // Implementing Testable interface
    virtual void print(const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;
    virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

    Vector unweighted_error(const VectorValues& c) const; /** (A*x-b) */
    Vector error_vector(const VectorValues& c) const; /** (A*x-b)/sigma */
    virtual double error(const VectorValues& c) const; /**  0.5*(A*x-b)'*D*(A*x-b) */

    /** Return the augmented information matrix represented by this GaussianFactor.
     * The augmented information matrix contains the information matrix with an
     * additional column holding the information vector, and an additional row
     * holding the transpose of the information vector.  The lower-right entry
     * contains the constant error term (when \f$ \delta x = 0 \f$).  The
     * augmented information matrix is described in more detail in HessianFactor,
     * which in fact stores an augmented information matrix.
     */
    virtual Matrix augmentedInformation() const;

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

    /**
     * @brief Returns (dense) A,b pair associated with factor, bakes in the weights
     */
    virtual std::pair<Matrix, Vector> jacobian() const;

    /**
     * @brief Returns (dense) A,b pair associated with factor, does not bake in weights
     */
    std::pair<Matrix, Vector> jacobianUnweighted() const;

    /** Return (dense) matrix associated with factor.  The returned system is an augmented matrix:
    *   [A b]
    *  weights are baked in */
    virtual Matrix augmentedJacobian() const;

    /** Return (dense) matrix associated with factor.  The returned system is an augmented matrix:
    *   [A b]
    *   weights are not baked in */
    Matrix augmentedJacobianUnweighted() const;

    /** Return the full augmented Jacobian matrix of this factor as a VerticalBlockMatrix object. */
    const VerticalBlockMatrix& matrixObject() const { return Ab_; }

    /** Mutable access to the full augmented Jacobian matrix of this factor as a VerticalBlockMatrix object. */
    VerticalBlockMatrix& matrixObject() { return Ab_; }

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    virtual GaussianFactor::shared_ptr negate() const;

    /** Check if the factor is empty.  TODO: How should this be defined? */
    virtual bool empty() const { return size() == 0 /*|| rows() == 0*/; }

    /** is noise model constrained ? */
    bool isConstrained() const {
      return model_ && model_->isConstrained();
    }

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    virtual DenseIndex getDim(const_iterator variable) const { 
      return Ab_(variable - begin()).cols(); 
    }

    /**
     * return the number of rows in the corresponding linear system
     */
    size_t rows() const { return Ab_.rows(); }

    /**
     * return the number of columns in the corresponding linear system
     */
    //[MH-A]: virtual allows same function name in MHJF
    virtual size_t cols() const { return Ab_.cols(); }

    /** get a copy of model */
    const SharedDiagonal& get_model() const { return model_;  }

    /** get a copy of model (non-const version) */
    SharedDiagonal& get_model() { return model_;  }

    /** Get a view of the r.h.s. vector b, not weighted by noise */
    const constBVector getb() const { return Ab_(size()).col(0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator */
    constABlock getA(const_iterator variable) const { return Ab_(variable - begin()); }

    /** Get a view of the A matrix, not weighted by noise */
    constABlock getA() const { return Ab_.range(0, size()); }

    /** Get a view of the r.h.s. vector b (non-const version) */
    BVector getb() { return Ab_(size()).col(0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator (non-const version) */
    ABlock getA(iterator variable) { return Ab_(variable - begin()); }

    /** Get a view of the A matrix */
    ABlock getA() { return Ab_.range(0, size()); }

    /** Update an information matrix by adding the information corresponding to this factor
     * (used internally during elimination).
     * @param scatter A mapping from variable index to slot index in this HessianFactor
     * @param info The information matrix to be updated
     */
    void updateHessian(const FastVector<Key>& keys, SymmetricBlockMatrix* info) const;

    //[MH-A]:  do nothing...
    void mhUpdateHessian(const FastVector<Key>& keys, HessList& hessian_list, const int& max_layer_idx) const {
      std::cout << "ERROR: JacobianFactor::mhUpdateHessian() should NEVER be called" << std::endl;
    }


    /** Return A*x */
    Vector operator*(const VectorValues& x) const;

    /** x += A'*e.  If x is initially missing any values, they are created and assumed to start as
     *  zero vectors. */
    void transposeMultiplyAdd(double alpha, const Vector& e, VectorValues& x) const;

    /** y += alpha * A'*A*x */
    void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const;

    /**
     * Raw memory access version of multiplyHessianAdd y += alpha * A'*A*x
     * Requires the vector accumulatedDims to tell the dimension of
     * each variable: e.g.: x0 has dim 3, x2 has dim 6, x3 has dim 2,
     * then accumulatedDims is [0 3 9 11 13]
     * NOTE: size of accumulatedDims is size of keys + 1!!
     * TODO(frank): we should probably kill this if no longer needed
     */
    void multiplyHessianAdd(double alpha, const double* x, double* y,
        const std::vector<size_t>& accumulatedDims) const;

    /// A'*b for Jacobian
    VectorValues gradientAtZero() const;

    /// A'*b for Jacobian (raw memory version)
    virtual void gradientAtZero(double* d) const;

    /// Compute the gradient wrt a key at any values
    Vector gradient(Key key, const VectorValues& x) const;

    /** Return a whitened version of the factor, i.e. with unit diagonal noise model. */
    JacobianFactor whiten() const;

    /** Eliminate the requested variables. */
    std::pair<boost::shared_ptr<GaussianConditional>, shared_ptr>
      eliminate(const Ordering& keys);

    /** set noiseModel correctly */
    void setModel(bool anyConstrained, const Vector& sigmas);

    /**
     * Densely partially eliminate with QR factorization, this is usually provided as an argument to
     * one of the factor graph elimination functions (see EliminateableFactorGraph).  HessianFactors
     * are first factored with Cholesky decomposition to produce JacobianFactors, by calling the
     * conversion constructor JacobianFactor(const HessianFactor&). Variables are eliminated in the
     * order specified in \c keys.
     * @param factors Factors to combine and eliminate
     * @param keys The variables to eliminate in the order as specified here in \c keys
     * @return The conditional and remaining factor
     *
     * \addtogroup LinearSolving */
    friend GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditional>, shared_ptr>
      EliminateQR(const GaussianFactorGraph& factors, const Ordering& keys);

    /**
     * splits a pre-factorized factor into a conditional, and changes the current
     * factor to be the remaining component. Performs same operation as eliminate(),
     * but without running QR.
     * NOTE: looks at dimension of noise model to determine how many rows to keep.
     * @param nrFrontals number of keys to eliminate
     */
    boost::shared_ptr<GaussianConditional> splitConditional(size_t nrFrontals);

  protected:

    /// Internal function to fill blocks and set dimensions
    template<typename TERMS>
    void fillTerms(const TERMS& terms, const Vector& b, const SharedDiagonal& noiseModel);

  private:

    /** Unsafe Constructor that creates an uninitialized Jacobian of right size
     *  @param keys in some order
     *  @param diemnsions of the variables in same order
     *  @param m output dimension
     *  @param model noise model (default NULL)
     */
    template<class KEYS, class DIMENSIONS>
    JacobianFactor(const KEYS& keys, const DIMENSIONS& dims, DenseIndex m,
        const SharedDiagonal& model = SharedDiagonal()) :
        Base(keys), Ab_(dims.begin(), dims.end(), m, true), model_(model) {
    }

    // be very selective on who can access these private methods:
    template<typename T> friend class ExpressionFactor;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(Ab_);
      ar & BOOST_SERIALIZATION_NVP(model_);
    }
  }; // JacobianFactor

/// traits
template<>
struct traits<JacobianFactor> : public Testable<JacobianFactor> {
};

//========================== MHJacobianFactor =============================
//[MH-A]: 
class GTSAM_EXPORT MHJacobianFactor : public JacobianFactor {
  public:

    typedef MHJacobianFactor This; ///< Typedef to this class
    
    //typedef GaussianFactor Base; ///< Typedef to base class
    typedef JacobianFactor Base; ///< Typedef to base class
    
    typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

    typedef VerticalBlockMatrix::Block ABlock;
    typedef VerticalBlockMatrix::constBlock constABlock;
    typedef ABlock::ColXpr BVector;
    typedef constABlock::ConstColXpr constBVector;

    typedef std::list<HypoNode*> HypoList;
    typedef std::list<JacobianFactor> JacobList;
    typedef typename HypoList::iterator HypoListIter;
    typedef typename JacobList::iterator JacobListIter;
    typedef typename HypoList::const_iterator HypoListCstIter;
    typedef typename JacobList::const_iterator JacobListCstIter;
    
    typedef std::vector<PruningRecord*> RecordArr;

    typedef std::list<VerticalBlockMatrix> AbList;
    typedef typename AbList::iterator AbListIter;
    typedef typename AbList::const_iterator AbListCstIter;

    typedef std::list<HessianFactor> HessList;
    typedef typename HessList::iterator HessListIter;
    typedef typename HessList::const_iterator HessListCstIter;
  
  public: //for convenience to access these two lists...
    
    JacobList jacobian_list_;

    HypoLayer* resulting_layer_;
 

  public:

    /** Convert from other GaussianFactor */
    //explicit JacobianFactor(const GaussianFactor& gf);

    /** Copy constructor */
    MHJacobianFactor(const MHJacobianFactor& jf) : Base(jf), jacobian_list_(jf.jacobian_list_), resulting_layer_(jf.resulting_layer_) {
    
      // Bridge MHJF constructor to JF
    
    } //mhsiao: NOT used so far...

    /** Conversion from HessianFactor (does Cholesky to obtain Jacobian matrix) */
    //explicit JacobianFactor(const HessianFactor& hf);

    /** default constructor for I/O */
    MHJacobianFactor() {}; //see original function in JacobianFactor.cpp

    /** Construct Null factor */
    //explicit JacobianFactor(const Vector& b_in);

    /** Construct unary factor */
    MHJacobianFactor(Key i1, const Matrix& A1, const Vector& b, const SharedDiagonal& model = SharedDiagonal()) : Base(i1, A1, b, model) {

      // Bridge MHJF constructor to JF
        
    }

    /** Construct binary factor */
    MHJacobianFactor(Key i1, const Matrix& A1, Key i2, const Matrix& A2, const Vector& b, const SharedDiagonal& model = SharedDiagonal()) : Base(i1, A1, i2, A2, b, model) {

      // Bridge MHJF constructor to JF
    
    }

    /** Construct ternary factor */
    MHJacobianFactor(Key i1, const Matrix& A1, Key i2, const Matrix& A2, Key i3, const Matrix& A3, const Vector& b, const SharedDiagonal& model = SharedDiagonal()) : Base(i1, A1, i2, A2, i3, A3, b, model) {

      // Bridge MHJF constructor to JF
    
    }

    /** Construct an n-ary factor
     * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
     *         collection of keys and matrices making up the factor. */
    //[MH-A]: used in linearization and construct GaussianConditional... 
    template<typename TERMS>
    MHJacobianFactor(const TERMS& mh_terms, const std::vector<Vector>& b_arr, HypoLayer* resultting_layer, const SharedDiagonal& model = SharedDiagonal()); //mshaio: called by MHNoiseModelFactor::linearize()

    /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
     *  instead of in block terms.  Note that only the active view of the provided augmented matrix
     *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
     *  factor. */

    //TODO: only use this function when we treat MHJF as an original JF (will this really happen?)
    template<typename KEYS>
    MHJacobianFactor(
      const KEYS& keys, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& sigmas = SharedDiagonal()) : Base(keys, augmentedMatrix, sigmas) { //mhsiao: used in original GaussianConditional as we replace "GC : public JF" with "GC : public MHJF"... 

      // Bridge MHJF constructor to JF

    }

    //[MH-A]: used in the constructor of GaussianConditional...
    template<typename KEYS>
    MHJacobianFactor(const KEYS& keys, const std::list<VerticalBlockMatrix>& augmentedMatrix_list, HypoLayer* resulting_layer, const SharedDiagonal& sigmas = SharedDiagonal()) {

      // Use the constructor of each JF
      AbListCstIter ait = augmentedMatrix_list.begin();
      const HypoList& hypo_list = resulting_layer->getNodeList();
      
      for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit, ++ait) {

        jacobian_list_.push_back(JacobianFactor(keys, (*ait), sigmas));

      }
      resulting_layer_ = resulting_layer;
      
      keys_.resize(keys.size());
      for (size_t j = 0; j < keys.size(); ++j) {
        keys_[j] = keys[j];
      }

    } // END MHJacobianFactor()


    /**
     * Build a dense joint factor from all the factors in a factor graph.  If a VariableSlots
     * structure computed for \c graph is already available, providing it will reduce the amount of
     * computation performed. */
    /*
    explicit JacobianFactor(
      const GaussianFactorGraph& graph,
      boost::optional<const Ordering&> ordering = boost::none,
      boost::optional<const VariableSlots&> p_variableSlots = boost::none);
    // */
    /** Virtual destructor */
    virtual ~MHJacobianFactor() {}

    /** Clone this JacobianFactor */
    virtual GaussianFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<GaussianFactor>(
          boost::make_shared<MHJacobianFactor>(*this)); //mshiao: can only be activated after all other functions are established...
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
      std::cout << "show: MHJF" << std::endl;
    }

    // Implementing Testable interface
    //TODO: mhsiao: some of them might only have an empty structure...
    //virtual void print(const std::string& s = "", const KeyFormatter& formatter = DefaultKeyFormatter) const;
    //virtual bool equals(const GaussianFactor& lf, double tol = 1e-9) const;

    //Vector unweighted_error(const VectorValues& c) const; /** (A*x-b) */
    //Vector error_vector(const VectorValues& c) const; /** (A*x-b)/sigma */
    //virtual double error(const VectorValues& c) const; /**  0.5*(A*x-b)'*D*(A*x-b) */

    /** Return the augmented information matrix represented by this GaussianFactor.
     * The augmented information matrix contains the information matrix with an
     * additional column holding the information vector, and an additional row
     * holding the transpose of the information vector.  The lower-right entry
     * contains the constant error term (when \f$ \delta x = 0 \f$).  The
     * augmented information matrix is described in more detail in HessianFactor,
     * which in fact stores an augmented information matrix.
     */
    //TODO: the format does not support MH... return empty for now...
    /*
    //virtual Matrix augmentedInformation() const;
    virtual Matrix augmentedInformation() const {
      Matrix empty_Ab;
      return empty_Ab.transpose() * empty_Ab;
    }
    // */

    /** Return the non-augmented information matrix represented by this
     * GaussianFactor.
     */
    //TODO: the format does not support MH... return empty for now...
    //virtual Matrix information() const;
    /*
    virtual Matrix information() const {
      Matrix empty_A;
      return empty_A.transpose() * empty_A;
    }
    // */

    /// Return the diagonal of the Hessian for this factor
    //TODO: the format does not support MH... return empty for now...
    //virtual VectorValues hessianDiagonal() const;

    /// Raw memory access version of hessianDiagonal
    //virtual void hessianDiagonal(double* d) const;

    /// Return the block diagonal of the Hessian for this factor
    //TODO: the format does not support MH... return empty for now...
    //virtual std::map<Key,Matrix> hessianBlockDiagonal() const;
    /*
    virtual std::map<Key,Matrix> hessianBlockDiagonal() const {
      std::map<Key, Matrix> blocks;
      return blocks;
    }
    // */

    /**
     * @brief Returns (dense) A,b pair associated with factor, bakes in the weights
     */
    //TODO: the format does not support MH... return empty for now...
    //virtual std::pair<Matrix, Vector> jacobian() const;
    /*
    virtual std::pair<Matrix, Vector> jacobian() const {
      Matrix empty_A;
      Vector empty_b;
      return std::make_pair(empty_A, empty_b);
    }
    ?/ */

    /**
     * @brief Returns (dense) A,b pair associated with factor, does not bake in weights
     */
    //std::pair<Matrix, Vector> jacobianUnweighted() const;

    /** Return (dense) matrix associated with factor.  The returned system is an augmented matrix:
    *   [A b]
    *  weights are baked in */
    //TODO: the format does not support MH... return empty for now...
    //virtual Matrix augmentedJacobian() const;
    /*
    virtual Matrix augmentedJacobian() const {
      Matrix empty_Ab;
      return empty_Ab;
    }
    // */

    /** Return (dense) matrix associated with factor.  The returned system is an augmented matrix:
    *   [A b]
    *   weights are not baked in */
    //Matrix augmentedJacobianUnweighted() const;

    /** Return the full augmented Jacobian matrix of this factor as a VerticalBlockMatrix object. */
    //const VerticalBlockMatrix& matrixObject() const { return Ab_; }

    /** Mutable access to the full augmented Jacobian matrix of this factor as a VerticalBlockMatrix object. */
    //VerticalBlockMatrix& matrixObject() { return Ab_; }

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    //virtual GaussianFactor::shared_ptr negate() const;

    /** Check if the factor is empty.  TODO: How should this be defined? */
    //virtual bool empty() const { return size() == 0 /*|| rows() == 0*/; }

    /** is noise model constrained ? */
    //TODO: mhsiao: not sure...
    /*
    bool isConstrained() const {
      //return model_ && model_->isConstrained();
      return jacobian_list_.front().get_model() && jacobian_list_.front().get_model()->isConstrained();
    }
    // */

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    //[MH-A]: called by Scatter() constructor... be careful about the "GaussianFactor::const_iterator variable"
    virtual DenseIndex getDim(const_iterator variable) const { 
      if (jacobian_list_.empty()) {
        // Call GC.getDim() in original ISAM2...
        return Base::getDim(variable);
      } else {
        // Same dim throughout the list
        return (jacobian_list_.front().matrixObject()(variable - begin())).cols(); 
      }
    }

    /**
     * return the number of rows in the corresponding linear system
     */
    //size_t rows() const { return Ab_.rows(); }

    /**
     * return the number of columns in the corresponding linear system
     */
    //[iMH-A]: 
    size_t cols() const { 
      if (jacobian_list_.empty()) {
        // Call GC.cols() in original ISAM2...
        return Base::cols(); 
      } else {
        // Same cols throughout the list
        return jacobian_list_.front().cols(); 
      }
    }

    /** get a copy of model */
    //const SharedDiagonal& get_model() const { return model_;  }

    /** get a copy of model (non-const version) */
    //SharedDiagonal& get_model() { return model_;  }

    /** Get a view of the r.h.s. vector b, not weighted by noise */
    //const constBVector getb() const { return Ab_(size()).col(0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator */
    //constABlock getA(const_iterator variable) const { return Ab_(variable - begin()); }

    /** Get a view of the A matrix, not weighted by noise */
    //constABlock getA() const { return Ab_.range(0, size()); }

    /** Get a view of the r.h.s. vector b (non-const version) */
    //BVector getb() { return Ab_(size()).col(0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator (non-const version) */
    //ABlock getA(iterator variable) { return Ab_(variable - begin()); }

    /** Get a view of the A matrix */
    //ABlock getA() { return Ab_.range(0, size()); }

    /** Update an information matrix by adding the information corresponding to this factor
     * (used internally during elimination).
     * @param scatter A mapping from variable index to slot index in this HessianFactor
     * @param info The information matrix to be updated
     */

    //[MH-A]:
    void mhUpdateHessian(const FastVector<Key>& keys, HessList& hessian_list, const int& max_layer_idx) const;

    /** Return A*x */
    //Vector operator*(const VectorValues& x) const;

    /** x += A'*e.  If x is initially missing any values, they are created and assumed to start as
     *  zero vectors. */
    //void transposeMultiplyAdd(double alpha, const Vector& e, VectorValues& x) const;

    /** y += alpha * A'*A*x */
    //TODO: the format does not support MH... return empty for now...
    /*
    void multiplyHessianAdd(double alpha, const VectorValues& x, VectorValues& y) const {
      // Do nothing...
    }
    // */

    /**
     * Raw memory access version of multiplyHessianAdd y += alpha * A'*A*x
     * Requires the vector accumulatedDims to tell the dimension of
     * each variable: e.g.: x0 has dim 3, x2 has dim 6, x3 has dim 2,
     * then accumulatedDims is [0 3 9 11 13]
     * NOTE: size of accumulatedDims is size of keys + 1!!
     * TODO(frank): we should probably kill this if no longer needed
     */
    //void multiplyHessianAdd(double alpha, const double* x, double* y, const std::vector<size_t>& accumulatedDims) const;

    /// A'*b for Jacobian
    //TODO: the format does not support MH... return empty for now...
    //VectorValues gradientAtZero() const;

    /// A'*b for Jacobian (raw memory version)
    //virtual void gradientAtZero(double* d) const;

    /// Compute the gradient wrt a key at any values
    //TODO: the format does not support MH... return empty for now...
    //Vector gradient(Key key, const VectorValues& x) const;
    /*
    Vector gradient(Key key, const VectorValues& x) const {
      Vector v;
      return v;
    }
    // */

    /** Return a whitened version of the factor, i.e. with unit diagonal noise model. */
    //JacobianFactor whiten() const;

    /** Eliminate the requested variables. */
    //TODO: might be used if QR-factorization is chosen (but not the default setting...)
    //std::pair<boost::shared_ptr<GaussianConditional>, shared_ptr> eliminate(const Ordering& keys);

    /** set noiseModel correctly */
    //void setModel(bool anyConstrained, const Vector& sigmas);
    
    //[MH-C]: called in removeAccumulatedPruned()
    void eraseJacobListAt(const std::vector<size_t>& idx_arr) {
      size_t count_idx = 0;
      size_t count_gv = 0;

      for (JacobListIter jit = jacobian_list_.begin(); jit != jacobian_list_.end(); ++jit) {

        if (count_gv == idx_arr[count_idx]) {

          jit = jacobian_list_.erase(jit);

          --jit;
          ++count_idx;

          if (count_idx == idx_arr.size()) {
            break;
          }
        }
        ++count_gv;
      }

    } // END eraseJacobListAt()
    
    //[MH-C]:
    bool removeAccumulatedPruned() {
      HypoList& hypo_list = resulting_layer_->getNodeList();
      if (jacobian_list_.size() == hypo_list.size()) {
        return false; //already up-to-date, no need any changes
      } else if (jacobian_list_.size() == 0) {
        return false; //NOT a useful factor but somehow added into the optimization (should DEBUG original iSAM2...)
      } else {
        RecordArr& record_arr = resulting_layer_->getRecordArr();
        
        for (size_t r = 0; r < record_arr.size(); ++r) {
        
          if (record_arr[r]->original_size_ == jacobian_list_.size()) {
            for (size_t t = r; t < record_arr.size(); ++t) {
              eraseJacobListAt(record_arr[t]->pruned_idx_arr_);
            }
            return true;
          }
        }

        // Output error msg and details 
        std::cout << "ERROR: MHJF::removeAccumulatedPruned() should match exactly one original_size_" << std::endl;
        for (size_t r = 0; r < record_arr.size(); ++r) {
          std::cout << "org_size: " << record_arr[r]->original_size_ << std::endl;          
        }
        std::cout << "jacob_list_size: " << jacobian_list_.size() << std::endl;          

        return false;
      }
    } // END removeAccumulatedPruned()

    /**
     * Densely partially eliminate with QR factorization, this is usually provided as an argument to
     * one of the factor graph elimination functions (see EliminateableFactorGraph).  HessianFactors
     * are first factored with Cholesky decomposition to produce JacobianFactors, by calling the
     * conversion constructor JacobianFactor(const HessianFactor&). Variables are eliminated in the
     * order specified in \c keys.
     * @param factors Factors to combine and eliminate
     * @param keys The variables to eliminate in the order as specified here in \c keys
     * @return The conditional and remaining factor
     *
     * \addtogroup LinearSolving */
    /*
    //TODO: again might be used if QR-factorization is chosen (but not the default setting...)
    friend GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditional>, shared_ptr> EliminateQR(const GaussianFactorGraph& factors, const Ordering& keys);
    // */
    /**
     * splits a pre-factorized factor into a conditional, and changes the current
     * factor to be the remaining component. Performs same operation as eliminate(),
     * but without running QR.
     * NOTE: looks at dimension of noise model to determine how many rows to keep.
     * @param nrFrontals number of keys to eliminate
     */
    //TODO: again might be used if QR-factorization is chosen (but not the default setting...)
    //boost::shared_ptr<GaussianConditional> splitConditional(size_t nrFrontals);

  protected:

    /// Internal function to fill blocks and set dimensions
    //[MH-A]: might be used at all...
    template<typename TERMS>
    void fillTerms(const TERMS& mh_terms, const std::vector<Vector>& b_arr, const SharedDiagonal& noiseModel);
 
  private:

    /** Unsafe Constructor that creates an uninitialized Jacobian of right size
     *  @param keys in some order
     *  @param diemnsions of the variables in same order
     *  @param m output dimension
     *  @param model noise model (default NULL)
     */
    /*
    template<class KEYS, class DIMENSIONS>
    JacobianFactor(const KEYS& keys, const DIMENSIONS& dims, DenseIndex m,
        const SharedDiagonal& model = SharedDiagonal()) :
        Base(keys), Ab_(dims.begin(), dims.end(), m, true), model_(model) {
    }
    // */

    // be very selective on who can access these private methods:
    template<typename T> friend class ExpressionFactor;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(resulting_layer_); 
      ar & BOOST_SERIALIZATION_NVP(jacobian_list_);
  }
}; // END MHJacobianFactor

/// traits
template<>
struct traits<MHJacobianFactor> : public Testable<MHJacobianFactor> {
};

//========================== END MHJacobianFactor =============================

} // \ namespace gtsam

#include <gtsam/linear/JacobianFactor-inl.h>


