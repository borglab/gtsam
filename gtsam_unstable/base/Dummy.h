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

#include <gtsam/global_includes.h>
#include <gtsam_unstable/dllexport.h>
#include <string>

namespace gtsam {

  struct GTSAM_UNSTABLE_EXPORT Dummy {
    size_t id;
    Dummy();
    ~Dummy();
    void print(const std::string& s="") const ;
    unsigned char dummyTwoVar(unsigned char a) const ;
  };

} // namespace gtsam

