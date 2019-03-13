/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTree.cpp
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 * @author  Michael Kaess
 * @author  Viorela Ila
 * @author  Richard Roberts
 */

#include <gtsam/inference/BayesTree.h>

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
void BayesTreeCliqueStats::print(const std::string& s) const {
  std::cout << s
    << "avg Conditional Size: " << avgConditionalSize << std::endl
    << "max Conditional Size: " << maxConditionalSize << std::endl
    << "avg Separator Size: "   << avgSeparatorSize   << std::endl
    << "max Separator Size: "   << maxSeparatorSize   << std::endl;
}

/* ************************************************************************* */
BayesTreeCliqueStats BayesTreeCliqueData::getStats() const
{
  BayesTreeCliqueStats stats;

  double sum = 0.0;
  size_t max = 0;
  for(size_t s: conditionalSizes) {
    sum += (double)s;
    if(s > max) max = s;
  }
  stats.avgConditionalSize = sum / (double)conditionalSizes.size();
  stats.maxConditionalSize = max;

  sum = 0.0;
  max = 1;
  for(size_t s: separatorSizes) {
    sum += (double)s;
    if(s > max) max = s;
  }
  stats.avgSeparatorSize = sum / (double)separatorSizes.size();
  stats.maxSeparatorSize = max;

  return stats;
}

}
