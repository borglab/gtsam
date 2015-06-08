/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file   ConjugateGradientSolver.cpp
 *  @brief  Implementation of Conjugate Gradient solver for a linear system
 *  @author Yong-Dian Jian
 *  @author Sungtae An
 *  @date   Nov 6, 2014
 */

#include <gtsam/linear/ConjugateGradientSolver.h>
#include <boost/algorithm/string.hpp>
#include <iostream>

using namespace std;

namespace gtsam {

/*****************************************************************************/
void ConjugateGradientParameters::print(ostream &os) const {
   Base::print(os);
   cout << "ConjugateGradientParameters" << endl
        << "minIter:       " << minIterations_ << endl
        << "maxIter:       " << maxIterations_ << endl
        << "resetIter:     " << reset_ << endl
        << "eps_rel:       " << epsilon_rel_ << endl
        << "eps_abs:       " << epsilon_abs_ << endl;
}

/*****************************************************************************/
std::string ConjugateGradientParameters::blasTranslator(const BLASKernel value) {
  std::string s;
  switch (value) {
  case ConjugateGradientParameters::GTSAM:      s = "GTSAM" ;      break;
  default:                                      s = "UNDEFINED" ;  break;
  }
  return s;
}

/*****************************************************************************/
ConjugateGradientParameters::BLASKernel ConjugateGradientParameters::blasTranslator(
    const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "GTSAM")  return ConjugateGradientParameters::GTSAM;

  /* default is SBM */
  return ConjugateGradientParameters::GTSAM;
}

/*****************************************************************************/

}


