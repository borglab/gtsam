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

/****************************************************************************/
KeyInfo::KeyInfo(const GaussianFactorGraph &fg, const Ordering &ordering) :
    ordering_(ordering) {
  initialize(fg);
}

/****************************************************************************/
KeyInfo::KeyInfo(const GaussianFactorGraph &fg) :
    ordering_(Ordering::Natural(fg)) {
  initialize(fg);
}

/****************************************************************************/
void KeyInfo::initialize(const GaussianFactorGraph &fg) {
  const map<Key, size_t> colspec = fg.getKeyDimMap();
  const size_t n = ordering_.size();
  size_t start = 0;

  for (size_t i = 0; i < n; ++i) {
    const Key key = ordering_[i];
    const auto it_key = colspec.find(key);
    if (it_key==colspec.end())
      throw std::runtime_error("KeyInfo: Inconsistency in key-dim map");
    const size_t dim = it_key->second;
    this->emplace(key, KeyInfoEntry(i, dim, start));
    start += dim;
  }
  numCols_ = start;
}

/****************************************************************************/
vector<size_t> KeyInfo::colSpec() const {
  std::vector<size_t> result(size(), 0);
  for ( const auto &item: *this ) {
    result[item.second.index] = item.second.dim;
  }
  return result;
}

/****************************************************************************/
VectorValues KeyInfo::x0() const {
  VectorValues result;
  for ( const auto &item: *this ) {
    result.emplace(item.first, Vector::Zero(item.second.dim));
  }
  return result;
}

/****************************************************************************/
Vector KeyInfo::x0vector() const {
  return Vector::Zero(numCols_);
}

}

