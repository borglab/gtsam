/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file GaussianISAM.h
 * @date July 29, 2013
 * @author Frank Dellaert
 * @author Richard Roberts
 */

#pragma once

#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/inference/ISAM.h>
#include <gtsam/base/Testable.h>

namespace gtsam {

  class GTSAM_EXPORT GaussianISAM : public ISAM<GaussianBayesTree>
  {
  public:
    typedef ISAM<GaussianBayesTree> Base;
    typedef GaussianISAM This;
    typedef std::shared_ptr<This> shared_ptr;

    /// @name Standard Constructors
    /// @{

    /** Create an empty Bayes Tree */
    GaussianISAM();

    /** Copy constructor */
    GaussianISAM(const GaussianBayesTree& bayesTree);

    /// @}

  };

  /// traits
  template <>
  struct traits<GaussianISAM> : public Testable<GaussianISAM> {};

}
