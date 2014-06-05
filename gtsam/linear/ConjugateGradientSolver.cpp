/*
 * ConjugateGradientSolver.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: ydjian
 */

#include <gtsam/linear/ConjugateGradientSolver.h>
#include <iostream>

namespace gtsam {

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
ConjugateGradientParameters::BLASKernel ConjugateGradientParameters::blasTranslator(const std::string &src) {
  std::string s = src;  boost::algorithm::to_upper(s);
  if (s == "GTSAM")  return ConjugateGradientParameters::GTSAM;

  /* default is SBM */
  return ConjugateGradientParameters::GTSAM;
}

/*****************************************************************************/

/*****************************************************************************/
void ConjugateGradientParameters::print() const {
   Base::print();
   std::cout << "ConjugateGradientParameters" << std::endl
             << "minIter:       " << minIterations_ << std::endl
             << "maxIter:       " << maxIterations_ << std::endl
             << "resetIter:     " << reset_ << std::endl
             << "eps_rel:       " << epsilon_rel_ << std::endl
             << "eps_abs:       " << epsilon_abs_ << std::endl;
 }

}


