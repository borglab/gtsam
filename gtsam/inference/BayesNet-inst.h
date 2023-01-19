/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file    BayesNet.h
 * @brief   Bayes network
 * @author  Frank Dellaert
 * @author  Richard Roberts
 */

#pragma once

#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/FactorGraph-inst.h>

#include <boost/range/adaptor/reversed.hpp>
#include <fstream>
#include <string>

namespace gtsam {

/* ************************************************************************* */
template <class CONDITIONAL>
void BayesNet<CONDITIONAL>::print(const std::string& s,
                                  const KeyFormatter& formatter) const {
  std::cout << (s.empty() ? "" : s + " ") << std::endl;
  std::cout << "size: " << this->size() << std::endl;
  for (size_t i = 0; i < this->size(); i++) {
    const auto& conditional = this->at(i);
    std::stringstream ss;
    ss << "conditional " << i << ": ";
    if (conditional) conditional->print(ss.str(), formatter);
  }
}

/* ************************************************************************* */
template <class CONDITIONAL>
void BayesNet<CONDITIONAL>::dot(std::ostream& os,
                                const KeyFormatter& keyFormatter,
                                const DotWriter& writer) const {
  writer.digraphPreamble(&os);

  // Create nodes for each variable in the graph
  for (Key key : this->keys()) {
    auto position = writer.variablePos(key);
    writer.drawVariable(key, keyFormatter, position, &os);
  }
  os << "\n";

  // Reverse order as typically Bayes nets stored in reverse topological sort.
  for (auto it = std::make_reverse_iterator(this->end()); 
      it != std::make_reverse_iterator(this->begin()); ++it) {
    const auto& conditional = *it;
    auto frontals = conditional->frontals();
    const Key me = frontals.front();
    auto parents = conditional->parents();
    for (const Key& p : parents) {
      os << "  var" << p << "->var" << me << "\n";
    }
  }

  os << "}";
  std::flush(os);
}

/* ************************************************************************* */
template <class CONDITIONAL>
std::string BayesNet<CONDITIONAL>::dot(const KeyFormatter& keyFormatter,
                                       const DotWriter& writer) const {
  std::stringstream ss;
  dot(ss, keyFormatter, writer);
  return ss.str();
}

/* ************************************************************************* */
template <class CONDITIONAL>
void BayesNet<CONDITIONAL>::saveGraph(const std::string& filename,
                                      const KeyFormatter& keyFormatter,
                                      const DotWriter& writer) const {
  std::ofstream of(filename.c_str());
  dot(of, keyFormatter, writer);
  of.close();
}

/* ************************************************************************* */
template <class CONDITIONAL>
double BayesNet<CONDITIONAL>::logProbability(const HybridValues& x) const {
  double sum = 0.;
  for (const auto& gc : *this) {
    if (gc) sum += gc->logProbability(x);
  }
  return sum;
}

/* ************************************************************************* */
template <class CONDITIONAL>
double BayesNet<CONDITIONAL>::evaluate(const HybridValues& x) const {
  return exp(-logProbability(x));
}

/* ************************************************************************* */

}  // namespace gtsam
