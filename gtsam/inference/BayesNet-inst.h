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
void BayesNet<CONDITIONAL>::saveGraph(const std::string& s,
                                      const KeyFormatter& keyFormatter) const {
  std::ofstream of(s.c_str());
  of << "digraph G{\n";

  for (auto conditional : boost::adaptors::reverse(*this)) {
    typename CONDITIONAL::Frontals frontals = conditional->frontals();
    Key me = frontals.front();
    typename CONDITIONAL::Parents parents = conditional->parents();
    for (Key p : parents)
      of << keyFormatter(p) << "->" << keyFormatter(me) << std::endl;
  }

  of << "}";
  of.close();
}

}  // namespace gtsam
