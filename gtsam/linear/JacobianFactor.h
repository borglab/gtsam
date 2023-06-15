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

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/global_includes.h>
#include <gtsam/inference/VariableSlots.h>

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp>
#endif

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
  GTSAM_EXPORT std::pair<std::shared_ptr<GaussianConditional>, std::shared_ptr<JacobianFactor> >
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
    typedef std::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

    typedef VerticalBlockMatrix::Block ABlock;
    typedef VerticalBlockMatrix::constBlock constABlock;
    typedef ABlock::ColXpr BVector;
    typedef constABlock::ConstColXpr constBVector;

  protected:

    VerticalBlockMatrix Ab_;      // the block view of the full matrix
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
      const GaussianFactorGraph& graph);

    /**
     * Build a dense joint factor from all the factors in a factor graph.  If a VariableSlots
     * structure computed for \c graph is already available, providing it will reduce the amount of
     * computation performed. */
    explicit JacobianFactor(
      const GaussianFactorGraph& graph,
      const VariableSlots& p_variableSlots);
    
    /**
     * Build a dense joint factor from all the factors in a factor graph.  If a VariableSlots
     * structure computed for \c graph is already available, providing it will reduce the amount of
     * computation performed. */
    explicit JacobianFactor(
      const GaussianFactorGraph& graph,
      const Ordering& ordering);
    
    /**
     * Build a dense joint factor from all the factors in a factor graph.  If a VariableSlots
     * structure computed for \c graph is already available, providing it will reduce the amount of
     * computation performed. */
    explicit JacobianFactor(
      const GaussianFactorGraph& graph,
      const Ordering& ordering,
      const VariableSlots& p_variableSlots);

    /** Virtual destructor */
    ~JacobianFactor() override {}

    /** Clone this JacobianFactor */
    GaussianFactor::shared_ptr clone() const override {
      return std::static_pointer_cast<GaussianFactor>(
          std::make_shared<JacobianFactor>(*this));
    }

    // Implementing Testable interface
    void print(const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;
    bool equals(const GaussianFactor& lf, double tol = 1e-9) const override;

    Vector unweighted_error(const VectorValues& c) const; /** (A*x-b) */
    Vector error_vector(const VectorValues& c) const; /** (A*x-b)/sigma */

    /// HybridValues simply extracts the \class VectorValues and calls error.
    using GaussianFactor::error;

    //// 0.5*(A*x-b)'*D*(A*x-b).
    double error(const VectorValues& c) const override; 

    /** Return the augmented information matrix represented by this GaussianFactor.
     * The augmented information matrix contains the information matrix with an
     * additional column holding the information vector, and an additional row
     * holding the transpose of the information vector.  The lower-right entry
     * contains the constant error term (when \f$ \delta x = 0 \f$).  The
     * augmented information matrix is described in more detail in HessianFactor,
     * which in fact stores an augmented information matrix.
     */
    Matrix augmentedInformation() const override;

    /** Return the non-augmented information matrix represented by this
     * GaussianFactor.
     */
    Matrix information() const override;

    /// Using the base method
    using Base::hessianDiagonal;

    /// Add the current diagonal to a VectorValues instance
    void hessianDiagonalAdd(VectorValues& d) const override;

    /// Raw memory access version of hessianDiagonal
    void hessianDiagonal(double* d) const override;

    /// Return the block diagonal of the Hessian for this factor
    std::map<Key,Matrix> hessianBlockDiagonal() const override;

    /**
     * @brief Returns (dense) A,b pair associated with factor, bakes in the weights
     */
    std::pair<Matrix, Vector> jacobian() const override;

    /**
     * @brief Returns (dense) A,b pair associated with factor, does not bake in weights
     */
    std::pair<Matrix, Vector> jacobianUnweighted() const;

    /** Return (dense) matrix associated with factor.  The returned system is an augmented matrix:
    *   [A b]
    *  weights are baked in */
    Matrix augmentedJacobian() const override;

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
    GaussianFactor::shared_ptr negate() const override;

    /** is noise model constrained ? */
    bool isConstrained() const {
      return model_ && model_->isConstrained();
    }

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    DenseIndex getDim(const_iterator variable) const override { 
      return Ab_(variable - begin()).cols(); 
    }

    /**
     * return the number of rows in the corresponding linear system
     */
    size_t rows() const { return Ab_.rows(); }

    /**
     * return the number of columns in the corresponding linear system
     */
    size_t cols() const { return Ab_.cols(); }

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
    void updateHessian(const KeyVector& keys, SymmetricBlockMatrix* info) const override;

    /** Return A*x */
    Vector operator*(const VectorValues& x) const;

    /** x += alpha * A'*e.  If x is initially missing any values, they are
     * created and assumed to start as zero vectors. */
    void transposeMultiplyAdd(double alpha, const Vector& e,
                              VectorValues& x) const;

    /** y += alpha * A'*A*x */
    void multiplyHessianAdd(double alpha, const VectorValues& x,
                            VectorValues& y) const override;

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
    VectorValues gradientAtZero() const override;

    /// A'*b for Jacobian (raw memory version)
    void gradientAtZero(double* d) const override;

    /// Compute the gradient wrt a key at any values
    Vector gradient(Key key, const VectorValues& x) const override;

    /** Return a whitened version of the factor, i.e. with unit diagonal noise model. */
    JacobianFactor whiten() const;

    /** Eliminate the requested variables. */
    std::pair<std::shared_ptr<GaussianConditional>, shared_ptr>
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
     * \ingroup LinearSolving */
    friend GTSAM_EXPORT std::pair<std::shared_ptr<GaussianConditional>, shared_ptr>
      EliminateQR(const GaussianFactorGraph& factors, const Ordering& keys);

    /**
     * splits a pre-factorized factor into a conditional, and changes the current
     * factor to be the remaining component. Performs same operation as eliminate(),
     * but without running QR.
     * NOTE: looks at dimension of noise model to determine how many rows to keep.
     * @param nrFrontals number of keys to eliminate
     */
    std::shared_ptr<GaussianConditional> splitConditional(size_t nrFrontals);

  protected:

    /// Internal function to fill blocks and set dimensions
    template<typename TERMS>
    void fillTerms(const TERMS& terms, const Vector& b, const SharedDiagonal& noiseModel);

  private:

    /**
     * Helper function for public constructors:
     * Build a dense joint factor from all the factors in a factor graph.  Takes in
     * ordered variable slots */
    void JacobianFactorHelper(
      const GaussianFactorGraph& graph,
      const FastVector<VariableSlots::const_iterator>& orderedSlots);

    /** Unsafe Constructor that creates an uninitialized Jacobian of right size
     *  @param keys in some order
     *  @param diemnsions of the variables in same order
     *  @param m output dimension
     *  @param model noise model (default nullptr)
     */
    template<class KEYS, class DIMENSIONS>
    JacobianFactor(const KEYS& keys, const DIMENSIONS& dims, DenseIndex m,
        const SharedDiagonal& model = SharedDiagonal()) :
        Base(keys), Ab_(dims.begin(), dims.end(), m, true), model_(model) {
    }

    // be very selective on who can access these private methods:
    template<typename T> friend class ExpressionFactor;

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void save(ARCHIVE & ar, const unsigned int version) const {
      // TODO(fan): This is a hack for Boost < 1.66
      // We really need to introduce proper versioning in the archives
      // As otherwise this will not read objects serialized by older
      // versions of GTSAM
      ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar << BOOST_SERIALIZATION_NVP(Ab_);
      bool model_null = false;
      if(model_.get() == nullptr) {
        model_null = true;
        ar << boost::serialization::make_nvp("model_null", model_null);
      } else {
        ar << boost::serialization::make_nvp("model_null", model_null);
        ar << BOOST_SERIALIZATION_NVP(model_);
      }
    }

    template<class ARCHIVE>
    void load(ARCHIVE & ar, const unsigned int version) {
      // invoke serialization of the base class
      ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar >> BOOST_SERIALIZATION_NVP(Ab_);
      if (version < 1) {
        ar >> BOOST_SERIALIZATION_NVP(model_);
      } else {
        bool model_null;
        ar >> BOOST_SERIALIZATION_NVP(model_null);
        if (!model_null) {
          ar >> BOOST_SERIALIZATION_NVP(model_);
        }
      }
    }

    BOOST_SERIALIZATION_SPLIT_MEMBER()
#endif
  }; // JacobianFactor
/// traits
template<>
struct traits<JacobianFactor> : public Testable<JacobianFactor> {
};

} // \ namespace gtsam

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
BOOST_CLASS_VERSION(gtsam::JacobianFactor, 1)
#endif

#include <gtsam/linear/JacobianFactor-inl.h>


