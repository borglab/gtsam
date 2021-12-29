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

#include <gtsam/inference/FactorGraph-inst.h>
#include <gtsam/inference/BayesNet.h>

#include <boost/range/adaptor/reversed.hpp>
#include <fstream>

namespace gtsam {

/* ************************************************************************* */
template <class CONDITIONAL>
void BayesNet<CONDITIONAL>::print(
    const std::string& s, const KeyFormatter& formatter) const {
  Base::print(s, formatter);
}

/* ************************************************************************* */
template <class CONDITIONAL>
void BayesNet<CONDITIONAL>::dot(std::ostream& os,
                                const KeyFormatter& keyFormatter) const {
  os << "digraph G{\n";

  for (auto conditional : *this) {
    auto frontals = conditional->frontals();
    const Key me = frontals.front();
    auto parents = conditional->parents();
    for (const Key& p : parents)
      os << keyFormatter(p) << "->" << keyFormatter(me) << "\n";
  }

  os << "}";
  std::flush(os);
}

/* ************************************************************************* */
template <class CONDITIONAL>
std::string BayesNet<CONDITIONAL>::dot(const KeyFormatter& keyFormatter) const {
  std::stringstream ss;
  dot(ss, keyFormatter);
  return ss.str();
}

/* ************************************************************************* */
template <class CONDITIONAL>
void BayesNet<CONDITIONAL>::saveGraph(const std::string& filename,
                                      const KeyFormatter& keyFormatter) const {
  std::ofstream of(filename.c_str());
  dot(of, keyFormatter);
  of.close();
}

/* ************************************************************************* */

}  // namespace gtsam
