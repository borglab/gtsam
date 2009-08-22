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
#include "FactorGraph.h"
#include "ChordalBayesNet.h"

namespace gtsam {

  /** Linear Factor Graph where all factors are Gaussians */
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

    /**
     * given a chordal bayes net, sets the linear factor graph identical to that CBN
     * FD: imperative !!
     */
    void setCBN(const ChordalBayesNet& CBN);

    /**
     * This function returns the best ordering for this linear factor
     * graph, computed using colamd
     */
    Ordering getOrdering() const;

    /**
     * find the separator, i.e. all the nodes that have at least one
     * common factor with the given node. FD: not used AFAIK.
     */
    std::set<std::string> find_separator(const std::string& key) const;

    /**
     * find all the factors that involve the given node and remove them
     * from the factor graph
     * @param key the key for the given node
     */
    LinearFactorSet find_factors_and_remove(const std::string& key);

    /**
     * extract and combine all the factors that involve a given node
     * @param key the key for the given node
     * @return the combined linear factor
     */
    boost::shared_ptr<MutableLinearFactor> 
      combine_factors(const std::string& key);

    /**
     * eliminate one node yielding a ConditionalGaussian
     * Eliminates the factors from the factor graph through find_factors_and_remove
     * and adds a new factor to the factor graph
     */
    ConditionalGaussian::shared_ptr eliminate_one(const std::string& key);
	 
    /**
     * eliminate factor graph in place(!) in the given order, yielding
     * a chordal Bayes net
     */
    ChordalBayesNet::shared_ptr eliminate(const Ordering& ordering);
		
    /**
     * Same as eliminate but allows for passing an incomplete ordering
     * that does not completely eliminate the graph
     */
    ChordalBayesNet::shared_ptr eliminate_partially(const Ordering& ordering);
		
    /**
     * optimize a linear factor graph
     * @param ordering fg in order
     */
    FGConfig optimize(const Ordering& ordering);

    /**
     * combine two factor graphs
     * @param const &lfg1 Linear factor graph
     * @param const &lfg2 Linear factor graph
     * @return a new combined factor graph
     */
    static const LinearFactorGraph combine2(const LinearFactorGraph& lfg1,
				     const LinearFactorGraph& lfg2 ) ;
		
    /**
     * combine two factor graphs
     * @param *lfg Linear factor graph
     */
    void combine(LinearFactorGraph &lfg);

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
