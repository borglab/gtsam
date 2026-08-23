/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   IterativeSolver.cpp
 * @brief  Some support classes for iterative solvers
 * @date   Sep 3, 2012
 * @author Yong-Dian Jian
 */

#include <gtsam/linear/IterativeSolver.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <iostream>

using namespace std;

namespace gtsam {

/*****************************************************************************/
string IterativeOptimizationParameters::getVerbosity() const {
  return verbosityTranslator(verbosity_);
}

/*****************************************************************************/
void IterativeOptimizationParameters::setVerbosity(const string &src) {
  verbosity_ = verbosityTranslator(src);
}

/*****************************************************************************/
void IterativeOptimizationParameters::print() const {
  print(cout);
}

/*****************************************************************************/
void IterativeOptimizationParameters::print(ostream &os) const {
  os << "IterativeOptimizationParameters:" << endl << "verbosity:     "
      << verbosityTranslator(verbosity_) << endl;
}

/*****************************************************************************/
bool IterativeOptimizationParameters::equals(
    const IterativeOptimizationParameters &other, double tol) const {
  return verbosity_ == other.verbosity();
}

/*****************************************************************************/
ostream& operator<<(ostream &os, const IterativeOptimizationParameters &p) {
  p.print(os);
  return os;
}

/*****************************************************************************/
IterativeOptimizationParameters::Verbosity IterativeOptimizationParameters::verbosityTranslator(
    const string &src) {
  string s = src;
  // Convert to upper case
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "SILENT")
    return IterativeOptimizationParameters::SILENT;
  else if (s == "COMPLEXITY")
    return IterativeOptimizationParameters::COMPLEXITY;
  else if (s == "ERROR")
    return IterativeOptimizationParameters::ERROR;
  /* default is default */
  else
    return IterativeOptimizationParameters::SILENT;
}

/*****************************************************************************/
string IterativeOptimizationParameters::verbosityTranslator(
    IterativeOptimizationParameters::Verbosity verbosity) {
  if (verbosity == SILENT)
    return "SILENT";
  else if (verbosity == COMPLEXITY)
    return "COMPLEXITY";
  else if (verbosity == ERROR)
    return "ERROR";
  else
    return "UNKNOWN";
}

/*****************************************************************************/
VectorValues IterativeSolver::optimize(const GaussianFactorGraph &gfg,
    const KeyInfo* keyInfo, const std::map<Key, Vector>* lambda) {
  return optimize(gfg, keyInfo ? *keyInfo : KeyInfo(gfg),
      lambda ? *lambda : std::map<Key, Vector>());
}

/*****************************************************************************/
VectorValues IterativeSolver::optimize(const GaussianFactorGraph &gfg,
    const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda) {
  return optimize(gfg, keyInfo, lambda, keyInfo.x0());
}

}
