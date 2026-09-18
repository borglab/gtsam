/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Dummy.h
 * @brief Dummy class for testing MATLAB memory allocation
 * @author Andrew Melim
 * @author Frank Dellaert
 * @date June 14, 2012
 */

#pragma once

#include <gtsam/config.h>

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43

#include <gtsam/global_includes.h>
#include <gtsam_unstable/dllexport.h>
#include <string>

namespace gtsam {

  /**
   * Dummy class for testing MATLAB memory allocation.
   * @deprecated This experimental utility has no maintained replacement.
   */
  struct GTSAM_UNSTABLE_EXPORT Dummy {
    size_t id;
    Dummy();
    ~Dummy();
    Dummy(const Dummy& other) = default;
    Dummy& operator=(const Dummy& other) = default;
    void print(const std::string& s="") const ;
    unsigned char dummyTwoVar(unsigned char a) const ;
  };

} // namespace gtsam

#endif  // GTSAM_ALLOW_DEPRECATED_SINCE_V43
