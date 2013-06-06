/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesNet.h
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/base/types.h>
#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>

namespace gtsam {

  /** Symbolic Bayes Net
   *  \nosubgrouping
   */
  class SymbolicBayesNetUnordered: public SymbolicFactorGraphUnordered {

  public:

    typedef SymbolicFactorGraphUnordered Base;
    typedef SymbolicConditionalUnordered ConditionalType;

    /// @name Standard Constructors
    /// @{

    /** Construct empty factor graph */
    SymbolicBayesNetUnordered() {}
    
    /// @}
    /// @name Standard Interface
    /// @{
  };

} // namespace gtsam
