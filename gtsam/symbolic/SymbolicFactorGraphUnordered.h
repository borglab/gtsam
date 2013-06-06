/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicFactorGraph.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/inference/FactorGraphUnordered.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/symbolic/SymbolicFactorUnordered.h>

namespace gtsam { class SymbolicConditionalUnordered; }
namespace gtsam { class SymbolicBayesNetUnordered; }
namespace gtsam { class SymbolicEliminationTreeUnordered; }
namespace gtsam { class SymbolicBayesTreeUnordered; }
namespace gtsam { class SymbolicJunctionTreeUnordered; }

namespace gtsam {

  /** Symbolic Factor Graph
   *  \nosubgrouping
   */
  class GTSAM_EXPORT SymbolicFactorGraphUnordered:
    public FactorGraphUnordered<SymbolicFactorUnordered>,
    public EliminateableFactorGraph<
    SymbolicFactorUnordered, SymbolicFactorGraphUnordered, SymbolicConditionalUnordered, SymbolicBayesNetUnordered,
    SymbolicEliminationTreeUnordered, SymbolicBayesTreeUnordered, SymbolicJunctionTreeUnordered>
  {
  public:

    typedef SymbolicFactorGraphUnordered This;
    typedef FactorGraphUnordered<SymbolicFactorUnordered> Base;
    typedef EliminateableFactorGraph<
      SymbolicFactorUnordered, SymbolicFactorGraphUnordered, SymbolicConditionalUnordered, SymbolicBayesNetUnordered,
      SymbolicEliminationTreeUnordered, SymbolicBayesTreeUnordered, SymbolicJunctionTreeUnordered> BaseEliminateable;
    typedef BaseEliminateable::Eliminate Eliminate;

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicFactorGraphUnordered() {}

    /** Construct from any factor graph with factors derived from SymbolicFactor. */
    template<class DERIVEDFACTOR>
    SymbolicFactorGraphUnordered(const FactorGraphUnordered<DERIVEDFACTOR>& graph) : Base(graph.begin(), graph.end()) {}

    /** Constructor from iterator over factors */
    template<typename ITERATOR>
    SymbolicFactorGraphUnordered(ITERATOR firstFactor, ITERATOR lastFactor) : Base(firstFactor, lastFactor) {}

    /// @}
    /// @name Standard Interface
    /// @{

    /** Push back unary factor */
    void push_factor(Key key);

    /** Push back binary factor */
    void push_factor(Key key1, Key key2);

    /** Push back ternary factor */
    void push_factor(Key key1, Key key2, Key key3);

    /** Push back 4-way factor */
    void push_factor(Key key1, Key key2, Key key3, Key key4);

    /// @}
  };

} // namespace gtsam
