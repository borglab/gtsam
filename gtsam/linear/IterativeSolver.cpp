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
#include <gtsam/linear/LinearSolver.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>

#include <boost/algorithm/string.hpp>

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
  boost::algorithm::to_upper(s);
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
    boost::optional<const KeyInfo&> keyInfo,
    boost::optional<const std::map<Key, Vector>&> lambda) const {
  return optimize(gfg, keyInfo ? *keyInfo : KeyInfo(gfg),
      lambda ? *lambda : std::map<Key, Vector>());
}

/*****************************************************************************/
VectorValues IterativeSolver::optimize(const GaussianFactorGraph &gfg,
    const KeyInfo &keyInfo, const std::map<Key, Vector> &lambda) const {
  return optimize(gfg, keyInfo, lambda, keyInfo.x0());
}

/*****************************************************************************/
boost::shared_ptr<LinearSolver> IterativeSolver::CreateFromParameters(
    const LinearSolverParams &params) {
  if (!params.iterativeParams) {
    throw std::runtime_error(
        "NonlinearOptimizer::solve: cg parameter has to be assigned ...");
  } else if (auto pcg = boost::dynamic_pointer_cast<PCGSolverParameters>(
                 params.iterativeParams)) {
    return boost::make_shared<PCGSolver>(*pcg);
  } else if (auto spcg = boost::dynamic_pointer_cast<SubgraphSolverParameters>(
                 params.iterativeParams)) {
    if (!params.ordering)
      throw std::runtime_error("SubgraphSolver needs an ordering");
    return boost::make_shared<SubgraphSolverWrapper>(*spcg, *params.ordering);
  } else {
    throw std::runtime_error(
        "NonlinearOptimizer::solve: special cg parameter type is not handled "
        "in LM solver ...");
  }
};

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
