/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicSequentialSolver.h
 * @author  Richard Roberts
 * @date    Oct 21, 2010
 */
#pragma once

#include <gtsam/inference/GenericSequentialSolver.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>

namespace gtsam {

  class SymbolicSequentialSolver : GenericSequentialSolver<IndexFactorOrdered> {

  protected:
    typedef GenericSequentialSolver<IndexFactorOrdered> Base;

  public:
    /**
     * Construct the solver for a factor graph.  This builds the junction
     * tree, which does the symbolic elimination, identifies the cliques,
     * and distributes all the factors to the right cliques.
     */
    SymbolicSequentialSolver(const SymbolicFactorGraphOrdered& factorGraph) : Base(factorGraph) {};

    /**
     * Construct the solver with a shared pointer to a factor graph and to a
     * VariableIndex.  The solver will store these pointers, so this constructor
     * is the fastest.
     */
    SymbolicSequentialSolver(const SymbolicFactorGraphOrdered::shared_ptr& factorGraph,
        const VariableIndexOrdered::shared_ptr& variableIndex) : Base(factorGraph, variableIndex) {};

    /** Print to cout */
    void print(const std::string& name = "SymbolicSequentialSolver: ") const { Base::print(name); };

    /** Test whether is equal to another */
    bool equals(const SymbolicSequentialSolver& other, double tol = 1e-9) const { return Base::equals(other, tol); };

    /**
     * Eliminate the factor graph sequentially.  Uses a column elimination tree
     * to recursively eliminate.
     */
    SymbolicBayesNetOrdered::shared_ptr eliminate() const
    { return Base::eliminate(&EliminateSymbolic); };

    /**
     * Compute a conditional density P(F|S) while marginalizing out variables J
     * P(F|S) is obtained by P(J,F,S)=P(J|F,S)P(F|S)P(S) and dropping P(S)
     * Returns the result as a Bayes net.
     */
    SymbolicBayesNetOrdered::shared_ptr conditionalBayesNet(const std::vector<Index>& js, size_t nrFrontals) const
    { return Base::conditionalBayesNet(js, nrFrontals, &EliminateSymbolic); };

    /**
     * Compute the marginal joint over a set of variables, by integrating out
     * all of the other variables.  Returns the result as a Bayes net.
     */
    SymbolicBayesNetOrdered::shared_ptr jointBayesNet(const std::vector<Index>& js) const
    { return Base::jointBayesNet(js, &EliminateSymbolic); };

    /**
     * Compute the marginal joint over a set of variables, by integrating out
     * all of the other variables.  Returns the result as a factor graph.
     */
    SymbolicFactorGraphOrdered::shared_ptr jointFactorGraph(const std::vector<Index>& js) const
    { return Base::jointFactorGraph(js, &EliminateSymbolic); };

    /**
     * Compute the marginal Gaussian density over a variable, by integrating out
     * all of the other variables.  This function returns the result as a factor.
     */
    IndexFactorOrdered::shared_ptr marginalFactor(Index j) const { return Base::marginalFactor(j, &EliminateSymbolic); };
  };

}

