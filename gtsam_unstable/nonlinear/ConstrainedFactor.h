/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */
/**
 * @file 	 ConstrainedFactor.h
 * @brief  
 * @author Duy-Nguyen Ta
 * @date 	 Sep 30, 2013
 */

#pragma once

#include <gtsam/base/types.h>
#include <boost/shared_ptr.hpp>

namespace gtsam {

/**
 * A base class for all ConstrainedFactor factors,
 * containing additional information for the constraint,
 * e.g., a unique key for the dual variable associated with it,
 * and special treatments for nonlinear constraint linearization in SQP.
 *
 * Derived classes of ConstrainedFactor should also inherit from
 * NoiseModelFactorX to reuse the normal linearization procedure in NonlinearFactor
 */
class ConstrainedFactor {

protected:
  Key dualKey_; //!< Unique key for the dual variable associated with this constraint

public:
  typedef boost::shared_ptr<ConstrainedFactor> shared_ptr;

public:
  /// Construct with dual key
  ConstrainedFactor(Key dualKey) : dualKey_(dualKey) {}

  /// Return the dual key
  Key dualKey() const { return dualKey_; }

};


} /* namespace gtsam */
