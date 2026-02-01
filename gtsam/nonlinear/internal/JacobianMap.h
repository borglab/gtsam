/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file JacobianMap.h
 * @date May 11, 2015
 * @author Frank Dellaert
 * @brief JacobianMap for returning derivatives from expressions
 */

#pragma once

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastVector.h>
#include <gtsam/base/VerticalBlockMatrix.h>

namespace gtsam {
namespace internal {

// A JacobianMap is the primary mechanism by which derivatives are returned.
// Expressions are designed to write their derivatives into an already allocated
// Jacobian of the correct size, of type VerticalBlockMatrix.
// The JacobianMap provides a mapping from keys to the underlying blocks.
class JacobianMap {
private:
  const KeyVector& keys_;
  VerticalBlockMatrix& Ab_;

public:
  /// Construct a JacobianMap for writing into a VerticalBlockMatrix Ab
  JacobianMap(const KeyVector& keys, VerticalBlockMatrix& Ab) :
      keys_(keys), Ab_(Ab) {
  }

  /// Access blocks of via key
  VerticalBlockMatrix::Block operator()(Key key) {
    KeyVector::const_iterator it = std::find(keys_.begin(), keys_.end(), key);
    DenseIndex block = it - keys_.begin();
    return Ab_(block);
  }
};

} // namespace internal
} // namespace gtsam

