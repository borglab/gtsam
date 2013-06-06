/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicEliminationTreeUnordered.h
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/inference/EliminationTreeUnordered.h>

namespace gtsam {

  class SymbolicEliminationTreeUnordered :
    public EliminationTreeUnordered<SymbolicBayesNetUnordered, SymbolicFactorGraphUnordered> {
  public:
    typedef EliminationTreeUnordered<SymbolicBayesNetUnordered, SymbolicFactorGraphUnordered> Base; ///< Base class
    typedef SymbolicEliminationTreeUnordered This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
    
    /**
    * Build the elimination tree of a factor graph using pre-computed column structure.
    * @param factorGraph The factor graph for which to build the elimination tree
    * @param structure The set of factors involving each variable.  If this is not
    * precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
    * named constructor instead.
    * @return The elimination tree
    */
    SymbolicEliminationTreeUnordered(const SymbolicFactorGraphUnordered& factorGraph,
      const VariableIndexUnordered& structure, const std::vector<Key>& order) :
    Base(factorGraph, structure, order) {}

    /** Build the elimination tree of a factor graph.  Note that this has to compute the column
    * structure as a VariableIndex, so if you already have this precomputed, use the other
    * constructor instead.
    * @param factorGraph The factor graph for which to build the elimination tree
    */
    SymbolicEliminationTreeUnordered(const SymbolicFactorGraphUnordered& factorGraph, const std::vector<Key>& order) :
      Base(factorGraph, order) {}

    /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    SymbolicEliminationTreeUnordered(const This& other) : Base(other) {}

    /** Assignment operator - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    This& operator=(const This& other) { (void) Base::operator=(other); return *this; }

  private:

    /// Private default constructor
    SymbolicEliminationTreeUnordered() {}

    friend class ::EliminationTreeUnorderedTester;

  };

}
