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
#include <gtsam/symbolic/SymbolicFactorUnordered.h>

namespace gtsam {

  /** Symbolic Factor Graph
   *  \nosubgrouping
   */
  class SymbolicFactorGraphUnordered: public FactorGraphUnordered<SymbolicFactorUnordered> {

  public:

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicFactorGraphUnordered() {}

    /// @}
    /// @name Standard Interface
    /// @{

    /** Push back unary factor */
    GTSAM_EXPORT void push_factor(Key key);

    /** Push back binary factor */
    GTSAM_EXPORT void push_factor(Key key1, Key key2);

    /** Push back ternary factor */
    GTSAM_EXPORT void push_factor(Key key1, Key key2, Key key3);

    /** Push back 4-way factor */
    GTSAM_EXPORT void push_factor(Key key1, Key key2, Key key3, Key key4);

    /// @}
  };

} // namespace gtsam
