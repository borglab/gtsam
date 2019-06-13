/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file SymbolicISAM.h
 * @date July 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/symbolic/SymbolicBayesTree.h>
#include <gtsam/inference/ISAM.h>

namespace gtsam {

  class GTSAM_EXPORT SymbolicISAM : public ISAM<SymbolicBayesTree>
  {
  public:
    typedef ISAM<SymbolicBayesTree> Base;
    typedef SymbolicISAM This;
    typedef boost::shared_ptr<This> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** Create an empty Bayes Tree */
    SymbolicISAM();

    /** Copy constructor */
    SymbolicISAM(const SymbolicBayesTree& bayesTree);

    /// @}

  };

}
