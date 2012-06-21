/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieMatrix.cpp
 * @brief A wrapper around Matrix providing Lie compatibility
 * @author Richard Roberts and Alex Cunningham
 */

#include <gtsam/base/LieMatrix.h>

namespace gtsam {

  /* ************************************************************************* */
  LieMatrix::LieMatrix(size_t m, size_t n, ...)
    : Matrix(m,n) {
    va_list ap;
    va_start(ap, n);
    for(size_t i = 0; i < m; ++i) {
      for(size_t j = 0; j < n; ++j) {
        double value = va_arg(ap, double);
        (*this)(i,j) = value;
      }
    }
    va_end(ap);
  }

}