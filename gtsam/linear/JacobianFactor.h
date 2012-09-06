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
 * @date    Dec 8, 2010
 */
#pragma once

#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/base/blockMatrices.h>
#include <gtsam/base/types.h>

#include <boost/tuple/tuple.hpp>

// Forward declarations of friend unit tests
class Combine2JacobianFactorTest;
class eliminateFrontalsJacobianFactorTest;
class constructor2JacobianFactorTest;

namespace gtsam {

  // Forward declarations
  class HessianFactor;
  class VariableSlots;
  template<class C> class BayesNet;

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
  class JacobianFactor : public GaussianFactor {
  protected:
		typedef Matrix AbMatrix;
		typedef VerticalBlockView<AbMatrix> BlockAb;

    noiseModel::Diagonal::shared_ptr model_; // Gaussian noise model with diagonal covariance matrix
    AbMatrix matrix_; // the full matrix corresponding to the factor
    BlockAb Ab_;      // the block view of the full matrix
    typedef GaussianFactor Base; // typedef to base

  public:
    typedef boost::shared_ptr<JacobianFactor> shared_ptr;
    typedef BlockAb::Block ABlock;
    typedef BlockAb::constBlock constABlock;
    typedef BlockAb::Column BVector;
    typedef BlockAb::constColumn constBVector;

    /** Copy constructor */
    JacobianFactor(const JacobianFactor& gf);

    /** Convert from other GaussianFactor */
    JacobianFactor(const GaussianFactor& gf);

    /** default constructor for I/O */
    JacobianFactor();

    /** Construct Null factor */
    JacobianFactor(const Vector& b_in);

    /** Construct unary factor */
    JacobianFactor(Index i1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model);

    /** Construct binary factor */
    JacobianFactor(Index i1, const Matrix& A1,
        Index i2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model);

    /** Construct ternary factor */
    JacobianFactor(Index i1, const Matrix& A1, Index i2,
        const Matrix& A2, Index i3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model);

    /** Construct an n-ary factor */
    JacobianFactor(const std::vector<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model);

    JacobianFactor(const std::list<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model);

    /** Construct from Conditional Gaussian */
    JacobianFactor(const GaussianConditional& cg);

    /** Convert from a HessianFactor (does Cholesky) */
    JacobianFactor(const HessianFactor& factor);

    /** Virtual destructor */
    virtual ~JacobianFactor() {}

    /** Aassignment operator */
    JacobianFactor& operator=(const JacobianFactor& rhs);

    /** Clone this JacobianFactor */
    virtual GaussianFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<GaussianFactor>(
          shared_ptr(new JacobianFactor(*this)));
    }

    // Implementing Testable interface
    virtual void print(const std::string& s = "",
    		const IndexFormatter& formatter = DefaultIndexFormatter) const;
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
    virtual Matrix computeInformation() const;

    /**
     * Construct the corresponding anti-factor to negate information
     * stored stored in this factor.
     * @return a HessianFactor with negated Hessian matrices
     */
    virtual GaussianFactor::shared_ptr negate() const;

    /** Check if the factor contains no information, i.e. zero rows.  This does
     * not necessarily mean that the factor involves no variables (to check for
     * involving no variables use keys().empty()).
     */
    bool empty() const { return Ab_.rows() == 0;}

    /** is noise model constrained ? */
    bool isConstrained() const { return model_->isConstrained();}

    /** Return the dimension of the variable pointed to by the given key iterator
     * todo: Remove this in favor of keeping track of dimensions with variables?
     */
    virtual size_t getDim(const_iterator variable) const { return Ab_(variable - begin()).cols(); }

    /**
     * return the number of rows in the corresponding linear system
     */
    size_t rows() const { return Ab_.rows(); }

    /**
     * return the number of rows in the corresponding linear system
     */
    size_t numberOfRows() const { return rows(); }

    /**
     * return the number of columns in the corresponding linear system
     */
    size_t cols() const { return Ab_.cols(); }

    /** get a copy of model */
    const SharedDiagonal& get_model() const { return model_;  }

    /** get a copy of model (non-const version) */
    SharedDiagonal& get_model() { return model_;  }

    /** Get a view of the r.h.s. vector b */
    const constBVector getb() const { return Ab_.column(size(), 0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator */
    constABlock getA(const_iterator variable) const { return Ab_(variable - begin()); }

    /** Get a view of the A matrix */
    constABlock getA() const { return Ab_.range(0, size()); }

    /** Get a view of the r.h.s. vector b (non-const version) */
    BVector getb() { return Ab_.column(size(), 0); }

    /** Get a view of the A matrix for the variable pointed to by the given key iterator (non-const version) */
    ABlock getA(iterator variable) { return Ab_(variable - begin()); }

    /** Get a view of the A matrix */
    ABlock getA() { return Ab_.range(0, size()); }

    /** Return A*x */
    Vector operator*(const VectorValues& x) const;

    /** x += A'*e */
    void transposeMultiplyAdd(double alpha, const Vector& e, VectorValues& x) const;

    /**
     * Return (dense) matrix associated with factor
     * @param ordering of variables needed for matrix column order
     * @param set weight to true to bake in the weights
     */
    std::pair<Matrix, Vector> matrix(bool weight = true) const;

    /**
     * Return (dense) matrix associated with factor
     * The returned system is an augmented matrix: [A b]
     * @param set weight to use whitening to bake in weights
     */
    Matrix matrix_augmented(bool weight = true) const;

    /**
     * Return vector of i, j, and s to generate an m-by-n sparse matrix
     * such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse.
     * As above, the standard deviations are baked into A and b
     * @param columnIndices First column index for each variable.
     */
    std::vector<boost::tuple<size_t, size_t, double> >
    sparse(const std::vector<size_t>& columnIndices) const;

    /**
     * Return a whitened version of the factor, i.e. with unit diagonal noise
     * model. */
    JacobianFactor whiten() const;

    /**
     * eliminate the first variable
     */
    boost::shared_ptr<GaussianConditional> eliminateFirst();

    /** return a multi-frontal conditional. It's actually a chordal Bayesnet */
    boost::shared_ptr<GaussianConditional> eliminate(size_t nrFrontals = 1);

    /**
     * splits a pre-factorized factor into a conditional, and changes the current
     * factor to be the remaining component. Performs same operation as eliminate(),
     * but without running QR.
     */
    boost::shared_ptr<GaussianConditional> splitConditional(size_t nrFrontals = 1);

    // following methods all used in CombineJacobians:
    // Many imperative, perhaps all need to be combined in constructor

    /** allocate space */
    void allocate(const VariableSlots& variableSlots,
				std::vector<size_t>& varDims, size_t m);

    /** set noiseModel correctly */
  	void setModel(bool anyConstrained, const Vector& sigmas);

    /** Assert invariants after elimination */
    void assertInvariants() const;

    /** An exception indicating that the noise model dimension passed into the
     * JacobianFactor has a different dimensionality than the factor. */
    class InvalidNoiseModel : std::exception {
    public:
      const size_t factorDims; ///< The dimensionality of the factor
      const size_t noiseModelDims; ///< The dimensionality of the noise model

      InvalidNoiseModel(size_t factorDims, size_t noiseModelDims) :
          factorDims(factorDims), noiseModelDims(noiseModelDims) {}
      virtual ~InvalidNoiseModel() throw() {}

      virtual const char* what() const throw();

    private:
      mutable std::string description_;
    };

  private:

    // Friend HessianFactor to facilitate conversion constructors
    friend class HessianFactor;

    // Friend unit tests (see also forward declarations above)
    friend class ::Combine2JacobianFactorTest;
    friend class ::eliminateFrontalsJacobianFactorTest;
    friend class ::constructor2JacobianFactorTest;

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
    	ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(GaussianFactor);
    	ar & BOOST_SERIALIZATION_NVP(Ab_);
    	ar & BOOST_SERIALIZATION_NVP(model_);
    	ar & BOOST_SERIALIZATION_NVP(matrix_);
    }
  }; // JacobianFactor

} // gtsam

