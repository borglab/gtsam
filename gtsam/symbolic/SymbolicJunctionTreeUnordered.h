/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicJunctionTreeUnordered.h
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/symbolic/SymbolicBayesTreeUnordered.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>
#include <gtsam/inference/JunctionTreeUnordered.h>

namespace gtsam {

  class GTSAM_EXPORT SymbolicJunctionTreeUnordered :
    public JunctionTreeUnordered<SymbolicBayesTreeUnordered, SymbolicFactorGraphUnordered> {
  public:
    typedef JunctionTreeUnordered<SymbolicBayesTreeUnordered, SymbolicFactorGraphUnordered> Base; ///< Base class
    typedef SymbolicJunctionTreeUnordered This; ///< This class
    typedef boost::shared_ptr<This> shared_ptr; ///< Shared pointer to this class
    
    /**
    * Build the elimination tree of a factor graph using pre-computed column structure.
    * @param factorGraph The factor graph for which to build the elimination tree
    * @param structure The set of factors involving each variable.  If this is not
    * precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
    * named constructor instead.
    * @return The elimination tree
    */
    SymbolicJunctionTreeUnordered(const SymbolicEliminationTreeUnordered& eliminationTree) :
      Base(Base::FromEliminationTree(eliminationTree)) {}

    /** Copy constructor - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    SymbolicJunctionTreeUnordered(const This& other) : Base(other) {}

    /** Assignment operator - makes a deep copy of the tree structure, but only pointers to factors are
     *  copied, factors are not cloned. */
    This& operator=(const This& other) { (void) Base::operator=(other); return *this; }

  private:

    // Dummy method to export class type
    void noop() const;
  };

}
