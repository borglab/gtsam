/**
 * @file    GaussianFactorGraph.h
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 * @author  Alireza Fathi
 */ 

// $Id: GaussianFactorGraph.h,v 1.24 2009/08/14 20:48:51 acunning Exp $

// \callgraph
 
#pragma once

#include <boost/shared_ptr.hpp>
#include "FactorGraph.h"
#include "Errors.h"
#include "GaussianFactor.h"
#include "GaussianBayesNet.h" // needed for MATLAB toolbox !!

namespace gtsam {

	class Ordering;

  /**
   * A Linear Factor Graph is a factor graph where all factors are Gaussian, i.e.
   *   Factor == GaussianFactor
   *   VectorConfig = A configuration of vectors
   * Most of the time, linear factor graphs arise by linearizing a non-linear factor graph.
   */
  class GaussianFactorGraph : public FactorGraph<GaussianFactor> {
  public:

    /**
     * Default constructor 
     */
    GaussianFactorGraph() {}

    /**
     * Constructor that receives a Chordal Bayes Net and returns a GaussianFactorGraph
     */
    GaussianFactorGraph(const GaussianBayesNet& CBN);

  	/** Add a null factor */
    inline void add(const Vector& b) {
    	push_back(sharedFactor(new GaussianFactor(b)));
  	}

  	/** Add a unary factor */
    inline void add(const std::string& key1, const Matrix& A1,
  			const Vector& b, double sigma) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,b,sigma)));
  	}

  	/** Add a binary factor */
    inline void add(const std::string& key1, const Matrix& A1,
  			const std::string& key2, const Matrix& A2,
  			const Vector& b, double sigma) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,b,sigma)));
  	}

  	/** Add a ternary factor */
    inline void add(const std::string& key1, const Matrix& A1,
  			const std::string& key2, const Matrix& A2,
  			const std::string& key3, const Matrix& A3,
  			const Vector& b, double sigma) {
    	push_back(sharedFactor(new GaussianFactor(key1,A1,key2,A2,key3,A3,b,sigma)));
  	}

  	/** Add an n-ary factor */
    inline void add(const std::vector<std::pair<std::string, Matrix> > &terms,
  	    const Vector &b, double sigma) {
    	push_back(sharedFactor(new GaussianFactor(terms,b,sigma)));
  	}

		/** return A*x-b */
		Errors errors(const VectorConfig& x) const;

			/** unnormalized error */
		double error(const VectorConfig& x) const;

		/** return A*x */
		Errors operator*(const VectorConfig& x) const;

		/** return A^x */
		VectorConfig operator^(const Errors& e) const;

  	/**
  	 * Calculate Gradient of A^(A*x-b) for a given config
  	 * @param x: VectorConfig specifying where to calculate gradient
  	 * @return gradient, as a VectorConfig as well
  	 */
  	VectorConfig gradient(const VectorConfig& x) const;

		/** Unnormalized probability. O(n) */
		double probPrime(const VectorConfig& c) const {
			return exp(-0.5 * error(c));
		}

    /**
     * find the separator, i.e. all the nodes that have at least one
     * common factor with the given node. FD: not used AFAIK.
     */
    std::set<std::string> find_separator(const std::string& key) const;

  	/**
     * Eliminate a single node yielding a conditional Gaussian
     * Eliminates the factors from the factor graph through findAndRemoveFactors
     * and adds a new factor on the separator to the factor graph
     */
    GaussianConditional::shared_ptr eliminateOne(const std::string& key);

    /**
     * eliminate factor graph in place(!) in the given order, yielding
     * a chordal Bayes net. Allows for passing an incomplete ordering
     * that does not completely eliminate the graph
     */
    GaussianBayesNet eliminate(const Ordering& ordering);
		
    /**
     * optimize a linear factor graph
     * @param ordering fg in order
     */
    VectorConfig optimize(const Ordering& ordering);

    /**
     * shared pointer versions for MATLAB
     */
    boost::shared_ptr<GaussianBayesNet> eliminate_(const Ordering&);
    boost::shared_ptr<VectorConfig> optimize_(const Ordering&);

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
     * Find all variables and their dimensions
     * @return The set of all variable/dimension pairs
     */
    Dimensions dimensions() const;

    /**
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */
    GaussianFactorGraph add_priors(double sigma) const;

    /**
     * Return RHS (b./sigmas) as Errors class
     */
    Errors rhs() const;

    /**
     * Return (dense) matrix associated with factor graph
     * @param ordering of variables needed for matrix column order
     */
    std::pair<Matrix,Vector> matrix (const Ordering& ordering) const;

    /**
     * get the starting column indices for all variables
     * @param ordering of variables needed for matrix column order
     * @return The set of all variable/index pairs
     */
    Dimensions columnIndices(const Ordering& ordering) const;

  	/**
  	 * Return 3*nzmax matrix where the rows correspond to the vectors i, j, and s
  	 * to generate an m-by-n sparse matrix, which can be given to MATLAB's sparse function.
  	 * The standard deviations are baked into A and b
  	 * @param ordering of variables needed for matrix column order
  	 */
  	Matrix sparse(const Ordering& ordering) const;

  	/**
  	 * Take an optimal step in direction d by calculating optimal step-size
  	 * @param x: starting point for search
  	 * @param d: search direction
  	 */
  	VectorConfig optimalUpdate(const VectorConfig& x0, const VectorConfig& d) const;

  	/**
		 * Find solution using gradient descent
		 * @param x0: VectorConfig specifying initial estimate
		 * @return solution
		 */
		VectorConfig steepestDescent(const VectorConfig& x0, bool verbose = false,
				double epsilon = 1e-3, size_t maxIterations = 0) const;

		/**
		 * shared pointer versions for MATLAB
		 */
		boost::shared_ptr<VectorConfig>
		steepestDescent_(const VectorConfig& x0, bool verbose = false,
				double epsilon = 1e-3, size_t maxIterations = 0) const;

		/**
		 * Find solution using conjugate gradient descent
		 * @param x0: VectorConfig specifying initial estimate
		 * @return solution
		 */
		VectorConfig conjugateGradientDescent(const VectorConfig& x0, bool verbose =
				false, double epsilon = 1e-3, size_t maxIterations = 0) const;

		/**
		 * shared pointer versions for MATLAB
		 */
		boost::shared_ptr<VectorConfig> conjugateGradientDescent_(
				const VectorConfig& x0, bool verbose = false, double epsilon = 1e-3,
				size_t maxIterations = 0) const;
  };

}
