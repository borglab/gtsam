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
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianBayesNet.h>

namespace gtsam {

	struct SharedDiagonal;

  /** return A*x-b
   * \todo Make this a member function - affects SubgraphPreconditioner */
  template<class FACTOR>
  Errors gaussianErrors(const FactorGraph<FACTOR>& fg, const VectorValues& x) {
    return *gaussianErrors_(fg, x);
  }

  /** shared pointer version
   * \todo Make this a member function - affects SubgraphPreconditioner */
  template<class FACTOR>
  boost::shared_ptr<Errors> gaussianErrors_(const FactorGraph<FACTOR>& fg, const VectorValues& x) {
    boost::shared_ptr<Errors> e(new Errors);
    BOOST_FOREACH(const typename FACTOR::shared_ptr& factor, fg) {
      e->push_back(factor->error_vector(x));
    }
    return e;
  }

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

    /** Constructor from a factor graph of GaussianFactor or a derived type */
    template<class DERIVEDFACTOR>
    GaussianFactorGraph(const FactorGraph<DERIVEDFACTOR>& fg) {
      push_back(fg);
    }

    /** Add a null factor */
    void add(const Vector& b) {
      push_back(sharedFactor(new JacobianFactor(b)));
    }

    /** Add a unary factor */
    void add(Index key1, const Matrix& A1,
        const Vector& b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(key1,A1,b,model)));
    }

    /** Add a binary factor */
    void add(Index key1, const Matrix& A1,
        Index key2, const Matrix& A2,
        const Vector& b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(key1,A1,key2,A2,b,model)));
    }

    /** Add a ternary factor */
    void add(Index key1, const Matrix& A1,
        Index key2, const Matrix& A2,
        Index key3, const Matrix& A3,
        const Vector& b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(key1,A1,key2,A2,key3,A3,b,model)));
    }

    /** Add an n-ary factor */
    void add(const std::vector<std::pair<Index, Matrix> > &terms,
        const Vector &b, const SharedDiagonal& model) {
      push_back(sharedFactor(new JacobianFactor(terms,b,model)));
    }

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    typedef FastSet<Index> Keys;
    Keys keys() const;

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
     * static function that combines two factor graphs
     * @param const &lfg1 Linear factor graph
     * @param const &lfg2 Linear factor graph
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
		Matrix sparse() const;

    /**
     * Return a dense \f$ m \times n \f$ Jacobian matrix, augmented with b
     * with standard deviations are baked into A and b
     */
    Matrix denseJacobian() const;

    /**
     * Return a dense \f$ n \times n \f$ Hessian matrix, augmented with \f$ A^T b \f$
     */
    Matrix denseHessian() const;

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
   * \ingroup LinearSolving
   */
	JacobianFactor::shared_ptr CombineJacobians(
			const FactorGraph<JacobianFactor>& factors,
			const VariableSlots& variableSlots);

	/**
	 * Densely combine and partially eliminate JacobianFactors to produce a
	 * single conditional with nrFrontals frontal variables and a remaining
	 * factor.
	 * Variables are eliminated in the natural order of the variable indices of in the factors.
	 * @param factors Factors to combine and eliminate
	 * @param nrFrontals Number of frontal variables to eliminate.
	 * @return The conditional and remaining factor

   * \ingroup LinearSolving
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

   * \ingroup LinearSolving
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

   * \ingroup LinearSolving
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

   * \ingroup LinearSolving
   */
  GaussianFactorGraph::EliminationResult EliminateCholesky(const FactorGraph<
			GaussianFactor>& factors, size_t nrFrontals = 1);

  /**
   * Densely partially eliminate with LDL factorization.  JacobianFactors
   * are left-multiplied with their transpose to form the Hessian using the
   * conversion constructor HessianFactor(const JacobianFactor&).
   *
   * If any factors contain constrained noise models (any sigmas equal to
   * zero), QR factorization will be performed instead, because our current
   * implementation cannot handle constrained noise models in LDL
   * factorization.  EliminateLDL(), on the other hand, will fail if any
   * factors contain constrained noise models.
   *
   * Variables are eliminated in the natural order of the variable indices of in
   * the factors.
   * @param factors Factors to combine and eliminate
   * @param nrFrontals Number of frontal variables to eliminate.
   * @return The conditional and remaining factor

   * \ingroup LinearSolving
   */
  GaussianFactorGraph::EliminationResult EliminatePreferLDL(const FactorGraph<
          GaussianFactor>& factors, size_t nrFrontals = 1);

  /**
   * Densely partially eliminate with LDL factorization.  JacobianFactors
   * are left-multiplied with their transpose to form the Hessian using the
   * conversion constructor HessianFactor(const JacobianFactor&).
   *
   * If any factors contain constrained noise models, this function will fail
   * because our current implementation cannot handle constrained noise models
   * in LDL factorization.  The function EliminatePreferLDL()
   * automatically does QR instead when this is the case.
   *
   * Variables are eliminated in the natural order of the variable indices of in
   * the factors.
   * @param factors Factors to combine and eliminate
   * @param nrFrontals Number of frontal variables to eliminate.
   * @return The conditional and remaining factor

   * \ingroup LinearSolving
   */
  GaussianFactorGraph::EliminationResult EliminateLDL(const FactorGraph<
      GaussianFactor>& factors, size_t nrFrontals = 1);

} // namespace gtsam
