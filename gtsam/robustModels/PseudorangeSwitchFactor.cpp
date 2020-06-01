/**
 *  @file   PseudorangeSwitchFactor.cpp
 *  @author Ryan Waton and Jason Gross
 *  @brief  Implementation file for pseudorange switchable factor
 **/

#include "PseudorangeSwitchFactor.h"

using namespace std;
using namespace vertigo; 

namespace gtsam {

//***************************************************************************
  Vector PseudorangeSwitchFactor::evaluateError(const gnssStateVec& q,
				   const SwitchVariableLinear& s,
                                   boost::optional<Matrix&> H1,
				   boost::optional<Matrix&> H2) const {

    double error = (h_.transpose()*q)-measured_;
    error *= s.value();
    if (H1) { (*H1) = (Matrix(1,5) << h_.transpose() * s.value() ).finished(); }
    if (H2) { (*H2) = (Vector(1) << error).finished(); }
    return (Vector(1) << error).finished();
  }
} // namespace
