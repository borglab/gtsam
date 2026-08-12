/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SymbolicConditional.cpp
 * @author  Richard Roberts
 * @date    Oct 17, 2010
 */

#include <gtsam/symbolic/SymbolicConditional.h>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
void SymbolicConditional::print(const std::string& str,
                                const KeyFormatter& keyFormatter) const {
  BaseConditional::print(str, keyFormatter);
}

/* ************************************************************************* */
bool SymbolicConditional::equals(const This& c, double tol) const {
  return BaseFactor::equals(c) && BaseConditional::equals(c);
}

/* ************************************************************************* */
double SymbolicConditional::logProbability(const HybridValues& c) const {
  throw std::runtime_error("SymbolicConditional::logProbability is not implemented");
}

/* ************************************************************************* */
double SymbolicConditional::evaluate(const HybridValues& c) const {
  throw std::runtime_error("SymbolicConditional::evaluate is not implemented");
}


}  // namespace gtsam
