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
 */ 
 
#pragma once

#include <boost/shared_ptr.hpp>

#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/Errors.h>
#include <gtsam/linear/GaussianFactor.h>
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

	/* dummy constructor, to be compatible with conjugate gradient solver */
    template<class DERIVEDFACTOR>
    GaussianFactorGraph(const FactorGraph<DERIVEDFACTOR>& fg, const VectorValues &x0) {
      push_back(fg);
    }

  	/** Add a null factor */
    void add(const Vector& b) {
    	push_back(sharedFactor(new GaussianFactor(b)));
  	}

  	/** Add a unary factor */
    void add(Index key1, const Matrix& A1,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,b,model)));
  	}

  	/** Add a binary factor */
    void add(Index key1, const Matrix& A1,
  			Index key2, const Matrix& A2,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,b,model)));
  	}

  	/** Add a ternary factor */
    void add(Index key1, const Matrix& A1,
  			Index key2, const Matrix& A2,
  			Index key3, const Matrix& A3,
  			const Vector& b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,key3,A3,b,model)));
  	}

  	/** Add an n-ary factor */
    void add(const std::vector<std::pair<Index, Matrix> > &terms,
  	    const Vector &b, const SharedDiagonal& model) {
    	push_back(sharedFactor(new GaussianFactor(terms,b,model)));
  	}

    /**
     * Return the set of variables involved in the factors (computes a set
     * union).
     */
    typedef std::set<Index, std::less<Index>, boost::fast_pool_allocator<Index> > Keys;
    Keys keys() const;

    /** Permute the variables in the factors */
    void permuteWithInverse(const Permutation& inversePermutation);

		/** return A*x-b */
		Errors errors(const VectorValues& x) const;

		/** shared pointer version */
		boost::shared_ptr<Errors> errors_(const VectorValues& x) const;

			/** unnormalized error */
		double error(const VectorValues& x) const;

		/** return A*x */
		Errors operator*(const VectorValues& x) const;

		/* In-place version e <- A*x that overwrites e. */
		void multiplyInPlace(const VectorValues& x, Errors& e) const;

		/* In-place version e <- A*x that takes an iterator. */
		void multiplyInPlace(const VectorValues& x, const Errors::iterator& e) const;

		/** x += alpha*A'*e */
		void transposeMultiplyAdd(double alpha, const Errors& e, VectorValues& x) const;

		/**
		 * Calculate Gradient of A^(A*x-b) for a given config
		 * @param x: VectorValues specifying where to calculate gradient
		 * @return gradient, as a VectorValues as well
		 */
		VectorValues gradient(const VectorValues& x) const;

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
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */
    GaussianFactorGraph add_priors(double sigma, const std::vector<size_t>& dimensions) const;

	/**
	 * Split a Gaussian factor graph into two, according to M
	 * M keeps the vertex indices of edges of A1. The others belong to A2.
	 */
	bool split(const std::map<Index, Index> &M, GaussianFactorGraph &A1, GaussianFactorGraph &A2) const ;

	// allocate a vectorvalues of b's structure
	VectorValues allocateVectorValuesb() const ;

	/* get the diagonal of A^ A, used to build jacobi preconditioner */
	bool getDiagonalOfHessian(VectorValues &values) const ;

	void residual(const VectorValues &x, VectorValues &r) const ;
	void multiply(const VectorValues &x, VectorValues &r) const ;
	void transposeMultiply(const VectorValues &r, VectorValues &x) const ;
	void getb(VectorValues &b) const ;
	VectorValues getb() const ;
  };

} // namespace gtsam
