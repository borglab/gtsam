/*
 * ConstrainedLinearFactorGraph.h
 *
 *  Created on: Aug 10, 2009
 *      Author: alexgc
 */

#ifndef CONSTRAINEDLINEARFACTORGRAPH_H_
#define CONSTRAINEDLINEARFACTORGRAPH_H_

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include "LinearFactorGraph.h"
#include "EqualityFactor.h"
#include "ConstrainedChordalBayesNet.h"

namespace gtsam {

class ConstrainedLinearFactorGraph: public LinearFactorGraph {

protected:
    std::vector<EqualityFactor::shared_ptr> eq_factors; /// collection of equality factors

public:
    // iterators for equality constraints - same interface as linear factors
    typedef std::vector<EqualityFactor::shared_ptr>::const_iterator eq_const_iterator;
    typedef std::vector<EqualityFactor::shared_ptr>::iterator eq_iterator;

public:
    /**
     * Default constructor
     */
	ConstrainedLinearFactorGraph();

	/**
	 * Copy from linear factor graph
	 */
	ConstrainedLinearFactorGraph(const LinearFactorGraph& lfg);

	virtual ~ConstrainedLinearFactorGraph();

	void push_back_eq(EqualityFactor::shared_ptr factor);

	// Additional STL-like functions for Equality Factors

	EqualityFactor::shared_ptr eq_at(const size_t i) const {return eq_factors.at(i);}

    /** return the iterator pointing to the first equality factor */
    eq_const_iterator eq_begin() const {
      return eq_factors.begin();
    }

    /** return the iterator pointing to the last factor */
    eq_const_iterator eq_end() const {
      return eq_factors.end();
    }

    /** clear the factor graph - re-implemented to include equality factors */
    void clear(){
      factors_.clear();
      eq_factors.clear();
    }

    /** size - reimplemented to include the equality factors_ */
    inline size_t size() const { return factors_.size() + eq_factors.size(); }

    /** Check equality - checks equality constraints as well*/
    bool equals(const LinearFactorGraph& fg, double tol=1e-9) const;

    /**
     * eliminate factor graph in place(!) in the given order, yielding
     * a chordal Bayes net
     */
    ConstrainedChordalBayesNet::shared_ptr eliminate(const Ordering& ordering);

    /**
     * optimize a linear factor graph
     * @param ordering fg in order
     */
    FGConfig optimize(const Ordering& ordering);

    /**
     * eliminate one node yielding a DeltaFunction
     * Eliminates the factors from the factor graph through find_factors_and_remove
     * and adds a new factor to the factor graph
     *
     * Only eliminates nodes *with* equality factors
     */
    DeltaFunction::shared_ptr eliminate_one_eq(const std::string& key);

    /**
     * Determines if a node has any equality factors connected to it
     */
    bool involves_equality(const std::string& key) const;

    /**
     * Prints the contents of the factor graph with optional name string
     */
    void print(const std::string& s="") const;

    /**
     * Finds a matching equality constraint by key - assumed present
     * Performs in-place removal of the equality constraint
     */
    EqualityFactor::shared_ptr extract_eq(const std::string& key);

    /**
     * Combines an equality factor with a joined linear factor
     * Executes in place, and will add new factors back to the graph
     */
    void eq_combine_and_eliminate(const EqualityFactor& eqf, const MutableLinearFactor& joint_factor);

    /**
     * This function returns the best ordering for this linear factor
     * graph, computed using colamd for the linear factors with all
     * of the equality factors eliminated first
     */
    Ordering getOrdering() const;

    /**
     * Converts the graph into a linear factor graph
     * Removes all equality constraints
     */
    LinearFactorGraph convert() const;

};

}

#endif /* CONSTRAINEDLINEARFACTORGRAPH_H_ */
