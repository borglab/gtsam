/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LieVector.cpp
 * @brief Implementations for LieVector functions
 * @author Alex Cunningham
 */

#include <gtsam/base/deprecated/LieVector_Deprecated.h>
#include <cstdarg>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
LieVector::LieVector(size_t m, const double* const data)
: Vector(m)
{
  for(size_t i = 0; i < m; i++)
    (*this)(i) = data[i];
}

/* ************************************************************************* */
void LieVector::print(const std::string& name) const {
  gtsam::print(vector(), name);
}

// Does not compile because LieVector is not fixed size.
// GTSAM_CONCEPT_LIE_INST(LieVector)
} // \namespace gtsam
