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

		/** unnormalized error */
		double error(const VectorConfig& c) const {
			double total_error = 0.;
			// iterate over all the factors_ to accumulate the log probabilities
			for (const_iterator factor = factors_.begin(); factor != factors_.end(); factor++)
				total_error += (*factor)->error(c);

			return total_error;
		}

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
     * Return (dense) matrix associated with factor graph
     * @param ordering of variables needed for matrix column order
     */
    std::pair<Matrix,Vector> matrix (const Ordering& ordering) const;

  	/**
  	 * Return 3*nzmax matrix where the rows correspond to the vectors i, j, and s
  	 * to generate an m-by-n sparse matrix, which can be given to MATLAB's sparse function.
  	 * The standard deviations are baked into A and b
  	 * @param ordering of variables needed for matrix column order
  	 */
  	Matrix sparse(const Ordering& ordering) const;
  };

}
