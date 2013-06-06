/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesTree.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/inference/BayesTreeUnordered.h>
#include <gtsam/inference/BayesTreeCliqueDefault.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  class GTSAM_EXPORT SymbolicBayesTreeCliqueUnordered :
    public BayesTreeCliqueBaseUnordered<SymbolicBayesTreeCliqueUnordered, SymbolicFactorGraphUnordered, SymbolicBayesNetUnordered>
  {
  public:
    typedef SymbolicBayesTreeCliqueUnordered This;
    typedef BayesTreeCliqueBaseUnordered<SymbolicBayesTreeCliqueUnordered, SymbolicFactorGraphUnordered, SymbolicBayesNetUnordered> Base;
    typedef boost::shared_ptr<This> shared_ptr;
    typedef boost::weak_ptr<This> weak_ptr;
    SymbolicBayesTreeCliqueUnordered() {}
    SymbolicBayesTreeCliqueUnordered(const SymbolicConditionalUnordered::shared_ptr& conditional) : Base(conditional) {}
  };

  /* ************************************************************************* */
  class GTSAM_EXPORT SymbolicBayesTreeUnordered :
    public BayesTreeUnordered<SymbolicBayesTreeCliqueUnordered>
  {

  public:
    /** Insert a new conditional */
    //void insert(const sharedConditional& conditional);

  protected:
    
    /**
     * Add a conditional to the front of a clique, i.e. a conditional whose
     * parents are already in the clique or its separators.  This function does
     * not check for this condition, it just updates the data structures.
     */
    //void addToCliqueFront(const sharedConditional& conditional, const sharedClique& clique);

  private:

    // Dummy method to export class
    void noop() const;

  };

}
