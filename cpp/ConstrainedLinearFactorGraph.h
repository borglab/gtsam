/**
 * @file ConstrainedLinearFactorGraph.h
 * @brief A modified version of LinearFactorGraph that can handle
 * linear constraints.
 * @author Alex Cunningham
 */

#ifndef CONSTRAINEDLINEARFACTORGRAPH_H_
#define CONSTRAINEDLINEARFACTORGRAPH_H_

#include "LinearFactorGraph.h"
#include "ChordalBayesNet.h"
#include "LinearConstraint.h"

namespace gtsam {

class ConstrainedLinearFactorGraph: public LinearFactorGraph {

protected:
    std::vector<LinearConstraint::shared_ptr> constraints_; /// collection of equality factors

public:
    // iterators for equality constraints - same interface as linear factors
    typedef std::vector<LinearConstraint::shared_ptr>::const_iterator constraint_const_iterator;
    typedef std::vector<LinearConstraint::shared_ptr>::iterator constraint_iterator;

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

	/**
	 * Add a constraint to the graph
	 * @param constraint is a shared pointer to a linear constraint between nodes in the graph
	 */
	void push_back_constraint(LinearConstraint::shared_ptr constraint);

	// Additional STL-like functions for Equality Factors
	LinearConstraint::shared_ptr constraint_at(const size_t i) const {return constraints_.at(i);}

    /** return the iterator pointing to the first equality factor */
    constraint_const_iterator constraint_begin() const {
      return constraints_.begin();
    }

    /** return the iterator pointing to the last factor */
    constraint_const_iterator constraint_end() const {
      return constraints_.end();
    }

    /** clear the factor graph - re-implemented to include equality factors */
    void clear(){
      factors_.clear();
      constraints_.clear();
    }

    /** size - reimplemented to include the equality factors_ */
    inline size_t size() const { return factors_.size() + constraints_.size(); }

    /** Check equality - checks equality constraints as well*/
    bool equals(const LinearFactorGraph& fg, double tol=1e-9) const;

    /**
     * eliminate factor graph in place(!) in the given order, yielding
     * a chordal Bayes net.  Note that even with constraints,
     * a constrained factor graph can produce a CBN, because
     * constrained conditional gaussian is a subclass of conditional
     * gaussian, with a different solving procedure.
     * @param ordering is the order to eliminate the variables
     */
    ChordalBayesNet::shared_ptr eliminate(const Ordering& ordering);

    /**
     * Eliminates a node with a constraint on it
     * Other factors have a change of variables performed via Schur complement to remove the
     * eliminated node.
     * FIXME: currently will not handle multiple constraints on the same node
     */
    ConstrainedConditionalGaussian::shared_ptr eliminate_constraint(const std::string& key);

    /**
     * optimize a linear factor graph
     * @param ordering fg in order
     */
    FGConfig optimize(const Ordering& ordering);

    /**
     * Determines if a node has any constraints attached to it
     */
    bool is_constrained(const std::string& key) const;

    /**
     * Prints the contents of the factor graph with optional name string
     */
    void print(const std::string& s="") const;

    /**
     * Finds a matching equality constraint by key - assumed present
     * Performs in-place removal of the constraint
     */
    LinearConstraint::shared_ptr extract_constraint(const std::string& key);

    /**
     * This function returns the best ordering for this linear factor
     * graph, computed using colamd for the linear factors with all
     * of the equality factors eliminated first
     */
    Ordering getOrdering() const;
};

}

#endif /* CONSTRAINEDLINEARFACTORGRAPH_H_ */
