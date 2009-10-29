/**
 * @file    LinearFactorGraph.h
 * @brief   Linear Factor Graph where all factors are Gaussians
 * @author  Kai Ni
 * @author  Christian Potthast
 * @author  Alireza Fathi
 */ 

// $Id: LinearFactorGraph.h,v 1.24 2009/08/14 20:48:51 acunning Exp $

// \callgraph
 
#pragma once

#include <boost/shared_ptr.hpp>

#include "LinearFactor.h"
#include "VectorConfig.h"
#include "FactorGraph.h"
#include "ChordalBayesNet.h"

namespace gtsam {

  /**
   * A Linear Factor Graph is a factor graph where all factors are Gaussian, i.e.
   *   Factor == LinearFactor
   *   VectorConfig = A configuration of vectors
   * Most of the time, linear factor graphs arise by linearizing a non-linear factor graph.
   */
  class LinearFactorGraph : public FactorGraph<LinearFactor> {
  public:

    /**
     * Default constructor 
     */
    LinearFactorGraph() {}

    /**
     * Constructor that receives a Chordal Bayes Net and returns a LinearFactorGraph
     */
    LinearFactorGraph(const ChordalBayesNet& CBN);

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
     * given a chordal bayes net, sets the linear factor graph identical to that CBN
     * FD: imperative !!
     */
    void setCBN(const ChordalBayesNet& CBN);

    /**
     * find the separator, i.e. all the nodes that have at least one
     * common factor with the given node. FD: not used AFAIK.
     */
    std::set<std::string> find_separator(const std::string& key) const;

    /**
     * eliminate factor graph in place(!) in the given order, yielding
     * a chordal Bayes net
     */
    boost::shared_ptr<ChordalBayesNet> eliminate(const Ordering& ordering);
		
    /**
     * Same as eliminate but allows for passing an incomplete ordering
     * that does not completely eliminate the graph
     */
    boost::shared_ptr<ChordalBayesNet> eliminate_partially(const Ordering& ordering);
		
    /**
     * optimize a linear factor graph
     * @param ordering fg in order
     */
    VectorConfig optimize(const Ordering& ordering);

    /**
     * static function that combines two factor graphs
     * @param const &lfg1 Linear factor graph
     * @param const &lfg2 Linear factor graph
     * @return a new combined factor graph
     */
    static LinearFactorGraph combine2(const LinearFactorGraph& lfg1,
				const LinearFactorGraph& lfg2);
		
    /**
     * combine two factor graphs
     * @param *lfg Linear factor graph
     */
    void combine(const LinearFactorGraph &lfg);

    /**
     * Find all variables and their dimensions
     * @return The set of all variable/dimension pairs
     */
    VariableSet variables() const;

    /**
     * Add zero-mean i.i.d. Gaussian prior terms to each variable
     * @param sigma Standard deviation of Gaussian
     */
    LinearFactorGraph add_priors(double sigma) const;

    /**
     * Return (dense) matrix associated with factor graph
     * @param ordering of variables needed for matrix column order
     */
    std::pair<Matrix,Vector> matrix (const Ordering& ordering) const;
  };

}
