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
#include <gtsam/inference/SymbolicFactorGraph.h>

namespace gtsam {

// The base class provides all of the needed functionality
typedef GenericSequentialSolver<IndexFactor> SymbolicSequentialSolver;

}

