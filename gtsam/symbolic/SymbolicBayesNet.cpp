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

#include <gtsam/symbolic/SymbolicBayesNet.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/inference/FactorGraph-inst.h>

#include <boost/range/adaptor/reversed.hpp>
#include <fstream>

namespace gtsam {

  // Instantiate base class
  template class FactorGraph<SymbolicConditional>;

  /* ************************************************************************* */
  bool SymbolicBayesNet::equals(const This& bn, double tol) const
  {
    return Base::equals(bn, tol);
  }

  /* ************************************************************************* */
  void SymbolicBayesNet::saveGraph(const std::string &s, const KeyFormatter& keyFormatter) const
  {
    std::ofstream of(s.c_str());
    of << "digraph G{\n";

    for (auto conditional: boost::adaptors::reverse(*this)) {
      SymbolicConditional::Frontals frontals = conditional->frontals();
      Key me = frontals.front();
      SymbolicConditional::Parents parents = conditional->parents();
      for(Key p: parents)
        of << p << "->" << me << std::endl;
    }

    of << "}";
    of.close();
  }


}
