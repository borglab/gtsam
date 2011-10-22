/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicSequentialSolver.cpp
 * @author  Richard Roberts
 * @date    Oct 21, 2010
 */

#include <gtsam/inference/SymbolicSequentialSolver.h>
#include <gtsam/inference/GenericSequentialSolver-inl.h>

namespace gtsam {

// An explicit instantiation to be compiled into the library
template class GenericSequentialSolver<IndexFactor>;

}
