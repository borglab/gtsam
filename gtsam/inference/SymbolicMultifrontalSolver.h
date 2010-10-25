/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicMultifrontalSolver.h
 * @brief   
 * @author  Richard Roberts
 * @created Oct 22, 2010
 */

#pragma once

#include <gtsam/inference/GenericMultifrontalSolver.h>
#include <gtsam/inference/SymbolicFactorGraph.h>
#include <gtsam/inference/JunctionTree.h>

namespace gtsam {

// The base class provides all of the needed functionality
typedef GenericMultifrontalSolver<IndexFactor, JunctionTree<FactorGraph<IndexFactor> > > SymbolicMultifrontalSolver;

}
