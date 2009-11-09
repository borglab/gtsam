/**
 * @file ConstrainedLinearFactorGraph.h
 * @brief A modified version of LinearFactorGraph that can handle
 * linear constraints.
 * @author Alex Cunningham
 */

#ifndef CONSTRAINEDLINEARFACTORGRAPH_H_
#define CONSTRAINEDLINEARFACTORGRAPH_H_

#include "LinearFactorGraph.h"
#include "GaussianBayesNet.h"
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

	/**
	 * STL-like indexing into the constraint vector
	 * @param i index of the target constraint
	 * @return the constraint to be returned
	 */
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
    GaussianBayesNet eliminate(const Ordering& ordering);

    /**
     * Picks one of the contraints in a set of constraints to eliminate
     * Currently just picks the first one - should probably be optimized
     * @param constraints is a set of constraints of which one will be eliminated
     * @return one of the constraints to use
     */
	LinearConstraint::shared_ptr pick_constraint(
			const std::vector<LinearConstraint::shared_ptr>& constraints) const;

	/**
	 * Eliminates the specified constraint for the selected variable,
	 * and then performs a change of variables on all the constraints in the separator
	 * and reinserts them into the graph.
	 * @param key is the identifier for the node to eliminate
	 * @param separator is the set of constraints to transform and reinsert
	 * @param constraint is the primary constraint to eliminate
	 */
	void update_constraints(const std::string& key,
			const std::vector<LinearConstraint::shared_ptr>& separator,
			const LinearConstraint::shared_ptr& constraint);

    /**
     * Eliminates a node with a constraint on it
     * Other factors have a change of variables performed via Schur complement to remove the
     * eliminated node.
     */
    ConstrainedConditionalGaussian::shared_ptr eliminate_constraint(const std::string& key);

    /**
     * optimize a linear factor graph
     * @param ordering fg in order
     */
    VectorConfig optimize(const Ordering& ordering);

    /**
     * Determines if a node has any constraints attached to it
     */
    bool is_constrained(const std::string& key) const;

    /**
     * Prints the contents of the factor graph with optional name string
     */
    void print(const std::string& s="") const;

    /**
     * Pulls out all constraints attached to a particular node
     * Note: this removes the constraints in place
     * @param key of the node to pull constraints on
     * @return a set of constraints
     */
    std::vector<LinearConstraint::shared_ptr> find_constraints_and_remove(const std::string& key);

    /**
     * This function returns the best ordering for this linear factor
     * graph, computed using colamd for the linear factors with all
     * of the equality factors eliminated first
     */
    Ordering getOrdering() const;
};

}

#endif /* CONSTRAINEDLINEARFACTORGRAPH_H_ */
