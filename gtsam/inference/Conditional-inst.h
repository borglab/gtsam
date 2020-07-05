/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Conditional.h
 * @brief   Base class for conditional densities
 * @author  Frank Dellaert
 */

// \callgraph
#pragma once

#include <iostream>

#include <gtsam/inference/Conditional.h>

namespace gtsam {

  /* ************************************************************************* */
  template<class FACTOR, class DERIVEDFACTOR>
  void Conditional<FACTOR,DERIVEDFACTOR>::print(const std::string& s, const KeyFormatter& formatter) const {
    std::cout << s << " P(";
    for(Key key: frontals())
      std::cout << " " << formatter(key);
    if (nrParents() > 0)
      std::cout << " |";
    for(Key parent: parents())
      std::cout << " " << formatter(parent);
    std::cout << ")" << std::endl;
  }

  /* ************************************************************************* */
  template<class FACTOR, class DERIVEDFACTOR>
  bool Conditional<FACTOR,DERIVEDFACTOR>::equals(const This& c, double tol) const
  {
    return nrFrontals_ == c.nrFrontals_;
  }

}
