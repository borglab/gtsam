/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicJunctionTreeUnordered.cpp
 * @date Mar 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/inference/JunctionTreeUnordered-inst.h>
#include <gtsam/symbolic/SymbolicJunctionTreeUnordered.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>

namespace gtsam {

  /* ************************************************************************* */
  SymbolicJunctionTreeUnordered::SymbolicJunctionTreeUnordered(
    const SymbolicEliminationTreeUnordered& eliminationTree) :
  Base(Base::FromEliminationTree(eliminationTree)) {}

  /* ************************************************************************* */
  SymbolicJunctionTreeUnordered::SymbolicJunctionTreeUnordered(const This& other) :
    Base(other) {}

  /* ************************************************************************* */
  SymbolicJunctionTreeUnordered& SymbolicJunctionTreeUnordered::operator=(const This& other)
  {
    (void) Base::operator=(other);
    return *this;
  }

}
