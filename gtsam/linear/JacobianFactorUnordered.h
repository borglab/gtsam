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

#include <gtsam/linear/GaussianFactorUnordered.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/global_includes.h>

#include <boost/make_shared.hpp>

namespace gtsam {

  // Forward declarations
  //class HessianFactor;
  class VariableSlots;
  class GaussianFactorGraphUnordered;
  class GaussianConditionalUnordered;
  class VectorValuesUnordered;
  class OrderingUnordered;

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
  class GTSAM_EXPORT JacobianFactorUnordered : public GaussianFactorUnordered
  {
  public:
    typedef JacobianFactorUnordered This; ///< Typedef to this class
    typedef GaussianFactorUnordered Base; ///< Typedef to base class
    typedef boost::shared_ptr<This> shared_ptr; ///< shared_ptr to this class

  protected:
    VerticalBlockMatrix Ab_;      // the block view of the full matrix
    noiseModel::Diagonal::shared_ptr model_; // Gaussian noise model with diagonal covariance matrix

  public:
    typedef VerticalBlockMatrix::Block ABlock;
    typedef VerticalBlockMatrix::constBlock constABlock;
    typedef VerticalBlockMatrix::Column BVector;
    typedef VerticalBlockMatrix::constColumn constBVector;

    /** Convert from other GaussianFactor */
    explicit JacobianFactorUnordered(const GaussianFactorUnordered& gf);

    /** default constructor for I/O */
    JacobianFactorUnordered();

    /** Construct Null factor */
    explicit JacobianFactorUnordered(const Vector& b_in);

    /** Construct unary factor */
    JacobianFactorUnordered(Key i1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Construct binary factor */
    JacobianFactorUnordered(Key i1, const Matrix& A1,
        Key i2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Construct ternary factor */
    JacobianFactorUnordered(Key i1, const Matrix& A1, Key i2,
        const Matrix& A2, Key i3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Construct an n-ary factor
     * @tparam TERMS A container whose value type is std::pair<Key, Matrix>, specifying the
     *         collection of keys and matrices making up the factor. */
    template<typename TERMS>
    JacobianFactorUnordered(const TERMS& terms, const Vector& b, const SharedDiagonal& model = SharedDiagonal());

    /** Constructor with arbitrary number keys, and where the augmented matrix is given all together
     *  instead of in block terms.  Note that only the active view of the provided augmented matrix
     *  is used, and that the matrix data is copied into a newly-allocated matrix in the constructed
     *  factor. */
    template<typename KEYS>
    JacobianFactorUnordered(
      const KEYS& keys, const VerticalBlockMatrix& augmentedMatrix, const SharedDiagonal& sigmas = SharedDiagonal());
    
    /** Convert from a HessianFactor (does Cholesky) */
    //JacobianFactorUnordered(const HessianFactor& factor);

    /**
     * Build a dense joint factor from all the factors in a factor graph.  If a VariableSlots
     * structure computed for \c graph is already available, providing it will reduce the amount of
     * computation performed. */
    explicit JacobianFactorUnordered(
      const GaussianFactorGraphUnordered& graph,
      boost::optional<const OrderingUnordered&> ordering = boost::none,
      boost::optional<const VariableSlots&> variableSlots = boost::none);

    /** Virtual destructor */
    virtual ~JacobianFactorUnordered() {}

    /** Clone this JacobianFactorUnordered */
    virtual GaussianFactorUnordered::shared_ptr clone() const {
      return boost::static_pointer_cast<GaussianFactorUnordered>(
          boost::make_shared<JacobianFactorUnordered>(*this));
    }

    // Implementing Testable interface
    virtual void print(const std::string& s = "",
      const KeyFormatter& formatter = DefaultKeyFormatter) const;
    virtual bool equals(const GaussianFactorUnordered& lf, double tol = 1e-9) const;

    Vector unweighted_error(const VectorValuesUnordered& c) const; /** (A*x-b) */
    Vector error_vector(const VectorValuesUnordered& c) const; /** (A*x-b)/sigma */
    virtual double error(const VectorValuesUnordered& c) const; /**  0.5*(A*x-b)'*D*(A*x-b) */

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
    
    /**
     * Return (dense) matrix associated with factor
     * @param ordering of variables needed for matrix column order
     * @param set weight to true to bake in the weights
     */
    virtual std::pair<Matrix, Vector> jacobian(bool weight = true) const;

    /**
     * Return (dense) matrix associated with factor
     * The returned system is an augmented matrix: [A b]
     * @param set weight to use whitening to bake in weights
     */
    virtual Matrix augmentedJacobian(bool weight = true) const;

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    //virtual GaussianFactorUnordered::shared_ptr negate() const;

    /** Check if the factor contains no information, i.e. zero rows.  This does
     * not necessarily mean that the factor involves no variables (to check for
     * involving no variables use keys().empty()).
     */
    virtual bool empty() const { return size() == 0 /*|| rows() == 0*/; }

    /** is noise model constrained ? */
    bool isConstrained() const { return model_->isConstrained(); }

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    virtual size_t getDim(const_iterator variable) const { return Ab_(variable - begin()).cols(); }

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

    /** Get a view of the r.h.s. vector b */
    const constBVector getb() const { return Ab_(size()).col(0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator */
    constABlock getA(const_iterator variable) const { return Ab_(variable - begin()); }

    /** Get a view of the A matrix */
    constABlock getA() const { return Ab_.range(0, size()); }

    /** Get a view of the r.h.s. vector b (non-const version) */
    BVector getb() { return Ab_(size()).col(0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator (non-const version) */
    ABlock getA(iterator variable) { return Ab_(variable - begin()); }

    /** Get a view of the A matrix */
    ABlock getA() { return Ab_.range(0, size()); }

    /** Return A*x */
    Vector operator*(const VectorValuesUnordered& x) const;

    /** x += A'*e.  If x is initially missing any values, they are created and assumed to start as
     *  zero vectors. */
    void transposeMultiplyAdd(double alpha, const Vector& e, VectorValuesUnordered& x) const;

    /** Return a whitened version of the factor, i.e. with unit diagonal noise model. */
    JacobianFactorUnordered whiten() const;

    /** Eliminate the requested variables. */
    std::pair<boost::shared_ptr<GaussianConditionalUnordered>, boost::shared_ptr<JacobianFactorUnordered> >
      eliminate(const OrderingUnordered& keys);

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
    friend GTSAM_EXPORT std::pair<boost::shared_ptr<GaussianConditionalUnordered>, boost::shared_ptr<JacobianFactorUnordered> >
      EliminateQRUnordered(const GaussianFactorGraphUnordered& factors, const OrderingUnordered& keys);

  private:

    /// Internal function to fill blocks and set dimensions
    template<typename TERMS>
    void fillTerms(const TERMS& terms, const Vector& b, const SharedDiagonal& noiseModel);
    
    /**
     * splits a pre-factorized factor into a conditional, and changes the current
     * factor to be the remaining component. Performs same operation as eliminate(),
     * but without running QR.
     */
    boost::shared_ptr<GaussianConditionalUnordered> splitConditional(size_t nrFrontals = 1);

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(Ab_);
      ar & BOOST_SERIALIZATION_NVP(model_);
    }
  }; // JacobianFactor

} // gtsam

#include <gtsam/linear/JacobianFactorUnordered-inl.h>


