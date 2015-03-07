/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GaussianFactorGraph.h
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 * @author  Alireza Fathi
 * @author  Richard Roberts
 * @author  Frank Dellaert
 */ 

#pragma once

#include <boost/tuple/tuple.hpp>

#include <gtsam/base/FastSet.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>

namespace gtsam {

  /**
   * A Linear Factor Graph is a factor graph where all factors are Gaussian, i.e.
   *   Factor == GaussianFactor
   *   VectorValues = A values structure of vectors
   * Most of the time, linear factor graphs arise by linearizing a non-linear factor graph.
   */
  class GaussianFactorGraph : public FactorGraph<GaussianFactor> {
  public:

    typedef boost::shared_ptr<GaussianFactorGraph> shared_ptr;
    typedef FactorGraph<GaussianFactor> Base;

    /**
     * Default constructor 
     */
    GaussianFactorGraph() {}

    /**
     * Constructor that receives a Chordal Bayes Net and returns a GaussianFactorGraph
     */
    GaussianFactorGraph(const GaussianBayesNet& CBN);

    /**
     * Constructor that receives a BayesTree and returns a GaussianFactorGraph
     */
    template<class CLIQUE>
    GaussianFactorGraph(const BayesTree<GaussianConditional,CLIQUE>& gbt) : Base(gbt) {}

    /** Constructor from a factor graph of GaussianFactor or a derived type */
    template<class DERIVEDFACTOR>
    GaussianFactorGraph(const FactorGraph<DERIVEDFACTOR>& fg) {
      push_back(fg);
    }

		/** Add a factor by value - makes a copy */
		void add(const GaussianFactor& factor) {
			factors_.push_back(factor.clone());
		}

		/** Add a factor by pointer - stores pointer without copying the factor */
		void add(const sharedFactor& factor) {
			factors_.push_back(factor);
		}

    /** Add a null factor */
    void add(const Vector& b) {
    	add(JacobianFactor(b));
    }

    /** Add a unary factor */
    void add(Index key1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model) {
    	add(JacobianFactor(key1,A1,b,model));
    }

    /** Add a binary factor */
    void add(Index key1, const Matrix& A1,
        Index key2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model) {
    	add(JacobianFactor(key1,A1,key2,A2,b,model));
    }

    /** Add a ternary factor */
    void add(Index key1, const Matrix& A1,
        Index key2, const Matrix& A2,
        Index key3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model) {
    	add(JacobianFactor(key1,A1,key2,A2,key3,A3,b,model));
    }

    /** Add an n-ary factor */
    void add(const std::vector<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model) {
    	add(JacobianFactor(terms,b,model));
    }

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    typedef FastSet<Index> Keys;
    Keys keys() const;

				
		/** Eliminate the first \c n frontal variables, returning the resulting
		 * conditional and remaining factor graph - this is very inefficient for
		 * eliminating all variables, to do that use EliminationTree or
		 * JunctionTree.  Note that this version simply calls
		 * FactorGraph<GaussianFactor>::eliminateFrontals with EliminateQR as the
		 * eliminate function argument.
		 */
		std::pair<sharedConditional, GaussianFactorGraph> eliminateFrontals(size_t nFrontals) const;

    /** Permute the variables in the factors */
    void permuteWithInverse(const Permutation& inversePermutation);

    /** unnormalized error */
    double error(const VectorValues& x) const {
      double total_error = 0.;
  		BOOST_FOREACH(const sharedFactor& factor, *this)
        total_error += factor->error(x);
      return total_error;
    }

    /** Unnormalized probability. O(n) */
    double probPrime(const VectorValues& c) const {
      return exp(-0.5 * error(c));
    }

    /**
     * Static function that combines two factor graphs.
     * @param lfg1 Linear factor graph
     * @param lfg2 Linear factor graph
     * @return a new combined factor graph
     */
    static GaussianFactorGraph combine2(const GaussianFactorGraph& lfg1,
        const GaussianFactorGraph& lfg2);

    /**
     * combine two factor graphs
     * @param *lfg Linear factor graph
     */
    void combine(const GaussianFactorGraph &lfg);

    /**
     * Return vector of i, j, and s to generate an m-by-n sparse Jacobian matrix,
		 * where i(k) and j(k) are the base 0 row and column indices, s(k) a double.
		 * The standard deviations are baked into A and b
		 */
		std::vector<boost::tuple<size_t, size_t, double> > sparseJacobian() const;

		/**
		 * Matrix version of sparseJacobian: generates a 3*m matrix with [i,j,s] entries
		 * such that S(i(k),j(k)) = s(k), which can be given to MATLAB's sparse.
		 * The standard deviations are baked into A and b
		 */
		Matrix sparseJacobian_() const;

    /**
     * Return a dense \f$ [ \;A\;b\; ] \in \mathbb{R}^{m \times n+1} \f$
		 * Jacobian matrix, augmented with b with the noise models baked
		 * into A and b.  The negative log-likelihood is
		 * \f$ \frac{1}{2} \Vert Ax-b \Vert^2 \f$.  See also
		 * GaussianFactorGraph::jacobian and GaussianFactorGraph::sparseJacobian.
     */
    Matrix augmentedJacobian() const;

		/**
		 * Return the dense Jacobian \f$ A \f$ and right-hand-side \f$ b \f$,
		 * with the noise models baked into A and b. The negative log-likelihood
		 * is \f$ \frac{1}{2} \Vert Ax-b \Vert^2 \f$.  See also
		 * GaussianFactorGraph::augmentedJacobian and
		 * GaussianFactorGraph::sparseJacobian.
		 */
		std::pair<Matrix,Vector> jacobian() const;

    /**
     * Return a dense \f$ \Lambda \in \mathbb{R}^{n+1 \times n+1} \f$ Hessian
		 * matrix, augmented with the information vector \f$ \eta \f$.  The
		 * augmented Hessian is
		 \f[ \left[ \begin{array}{ccc}
		 \Lambda & \eta \\
		 \eta^T & c
		 \end{array} \right] \f]
		 and the negative log-likelihood is
		 \f$ \frac{1}{2} x^T \Lambda x + \eta^T x + c \f$.
     */
    Matrix augmentedHessian() const;

		/**
		 * Return the dense Hessian \f$ \Lambda \f$ and information vector
		 * \f$ \eta \f$, with the noise models baked in. The negative log-likelihood
		 * is \frac{1}{2} x^T \Lambda x + \eta^T x + c.  See also
		 * GaussianFactorGraph::augmentedHessian.
		 */
		std::pair<Matrix,Vector> hessian() const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
    	ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    }

  };

  /**
   * Combine and eliminate several factors.
   * \addtogroup LinearSolving
   */
	JacobianFactor::shared_ptr CombineJacobians(
			const FactorGraph<JacobianFactor>& factors,
			const VariableSlots& variableSlots);

	/**
	 * Evaluates whether linear factors have any constrained noise models
	 * @return true if any factor is constrained.
	 */
	bool hasConstraints(const FactorGraph<GaussianFactor>& factors);

	/**
	 * Densely combine and partially eliminate JacobianFactors to produce a
	 * single conditional with nrFrontals frontal variables and a remaining
	 * factor.
	 * Variables are eliminated in the natural order of the variable indices of in the factors.
	 * @param factors Factors to combine and eliminate
	 * @param nrFrontals Number of frontal variables to eliminate.
	 * @return The conditional and remaining factor

   * \addtogroup LinearSolving
	 */
	GaussianFactorGraph::EliminationResult EliminateJacobians(const FactorGraph<
			JacobianFactor>& factors, size_t nrFrontals = 1);

	/**
	 * Densely partially eliminate with QR factorization.  HessianFactors are
	 * first factored with Cholesky decomposition to produce JacobianFactors,
	 * by calling the conversion constructor JacobianFactor(const HessianFactor&).
	 * Variables are eliminated in the natural order of the variable indices of in
	 * the factors.
	 * @param factors Factors to combine and eliminate
	 * @param nrFrontals Number of frontal variables to eliminate.
	 * @return The conditional and remaining factor

   * \addtogroup LinearSolving
	 */
  GaussianFactorGraph::EliminationResult EliminateQR(const FactorGraph<
			GaussianFactor>& factors, size_t nrFrontals = 1);

  /**
   * Densely partially eliminate with Cholesky factorization.  JacobianFactors
   * are left-multiplied with their transpose to form the Hessian using the
   * conversion constructor HessianFactor(const JacobianFactor&).
   *
   * If any factors contain constrained noise models (any sigmas equal to
   * zero), QR factorization will be performed instead, because our current
   * implementation cannot handle constrained noise models in Cholesky
   * factorization.  EliminateCholesky(), on the other hand, will fail if any
   * factors contain constrained noise models.
   *
   * Variables are eliminated in the natural order of the variable indices of in
   * the factors.
   * @param factors Factors to combine and eliminate
   * @param nrFrontals Number of frontal variables to eliminate.
   * @return The conditional and remaining factor

   * \addtogroup LinearSolving
   */
  GaussianFactorGraph::EliminationResult EliminatePreferCholesky(const FactorGraph<
			GaussianFactor>& factors, size_t nrFrontals = 1);

  /**
   * Densely partially eliminate with Cholesky factorization.  JacobianFactors
   * are left-multiplied with their transpose to form the Hessian using the
   * conversion constructor HessianFactor(const JacobianFactor&).
   *
   * If any factors contain constrained noise models, this function will fail
   * because our current implementation cannot handle constrained noise models
   * in Cholesky factorization.  The function EliminatePreferCholesky()
   * automatically does QR instead when this is the case.
   *
   * Variables are eliminated in the natural order of the variable indices of in
   * the factors.
   * @param factors Factors to combine and eliminate
   * @param nrFrontals Number of frontal variables to eliminate.
   * @return The conditional and remaining factor

   * \addtogroup LinearSolving
   */
  GaussianFactorGraph::EliminationResult EliminateCholesky(const FactorGraph<
			GaussianFactor>& factors, size_t nrFrontals = 1);

  /****** Linear Algebra Opeations ******/

  /** return A*x */
  Errors operator*(const GaussianFactorGraph& fg, const VectorValues& x);

  /** In-place version e <- A*x that overwrites e. */
  void multiplyInPlace(const GaussianFactorGraph& fg, const VectorValues& x, Errors& e);

  /** In-place version e <- A*x that takes an iterator. */
  void multiplyInPlace(const GaussianFactorGraph& fg, const VectorValues& x, const Errors::iterator& e);

  /** x += alpha*A'*e */
  void transposeMultiplyAdd(const GaussianFactorGraph& fg, double alpha, const Errors& e, VectorValues& x);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=x_0} \left\Vert \Sigma^{-1} A x - b \right\Vert^2 \f$,
   * centered around \f$ x = x_0 \f$.
   * The gradient is \f$ A^T(Ax-b) \f$.
   * @param fg The Jacobian factor graph $(A,b)$
   * @param x0 The center about which to compute the gradient
   * @return The gradient as a VectorValues
   */
  VectorValues gradient(const GaussianFactorGraph& fg, const VectorValues& x0);

  /**
   * Compute the gradient of the energy function,
   * \f$ \nabla_{x=0} \left\Vert \Sigma^{-1} A x - b \right\Vert^2 \f$,
   * centered around zero.
   * The gradient is \f$ A^T(Ax-b) \f$.
   * @param fg The Jacobian factor graph $(A,b)$
   * @param [output] g A VectorValues to store the gradient, which must be preallocated, see allocateVectorValues
   * @return The gradient as a VectorValues
   */
  void gradientAtZero(const GaussianFactorGraph& fg, VectorValues& g);

  /* matrix-vector operations */
  void residual(const GaussianFactorGraph& fg, const VectorValues &x, VectorValues &r);
  void multiply(const GaussianFactorGraph& fg, const VectorValues &x, VectorValues &r);
  void transposeMultiply(const GaussianFactorGraph& fg, const VectorValues &r, VectorValues &x);

  /** shared pointer version
   * \todo Make this a member function - affects SubgraphPreconditioner */
  boost::shared_ptr<Errors> gaussianErrors_(const GaussianFactorGraph& fg, const VectorValues& x);

	/** return A*x-b
   * \todo Make this a member function - affects SubgraphPreconditioner */
  inline Errors gaussianErrors(const GaussianFactorGraph& fg, const VectorValues& x) {
    return *gaussianErrors_(fg, x); }

} // namespace gtsam
