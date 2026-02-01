/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicISAM.cpp
 * @date July 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/symbolic/SymbolicISAM.h>
#include <gtsam/inference/ISAM-inst.h>

namespace gtsam {

  // Instantiate base class
  template class ISAM<SymbolicBayesTree>;

  /* ************************************************************************* */
  SymbolicISAM::SymbolicISAM() {}

  /* ************************************************************************* */
  SymbolicISAM::SymbolicISAM(const SymbolicBayesTree& bayesTree) :
    Base(bayesTree) {}

}
