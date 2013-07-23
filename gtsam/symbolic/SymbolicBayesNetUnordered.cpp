/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicBayesNet.cpp
 * @date Oct 29, 2009
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#include <gtsam/symbolic/SymbolicBayesNetUnordered.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>
#include <gtsam/inference/FactorGraphUnordered-inst.h>

#include <fstream>

namespace gtsam {

  /* ************************************************************************* */
  bool SymbolicBayesNetUnordered::equals(const This& bn, double tol) const
  {
    return Base::equals(bn, tol);
  }

  /* ************************************************************************* */
  void SymbolicBayesNetUnordered::saveGraph(const std::string &s, const KeyFormatter& keyFormatter) const
  {
    std::ofstream of(s.c_str());
    of << "digraph G{\n";

    BOOST_REVERSE_FOREACH(const sharedConditional& conditional, *this) {
      SymbolicConditionalUnordered::Frontals frontals = conditional->frontals();
      Key me = frontals.front();
      SymbolicConditionalUnordered::Parents parents = conditional->parents();
      BOOST_FOREACH(Key p, parents)
        of << p << "->" << me << std::endl;
    }

    of << "}";
    of.close();
  }


}
