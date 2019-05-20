/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Class.h
 * @brief describe the C++ class that is being wrapped
 * @author Frank Dellaert
 * @author Andrew Melim
 * @author Richard Roberts
 **/

#pragma once

#include <string>

namespace wrap {

  class Class;

  struct ForwardDeclaration {
    Class cls;
    bool isVirtual;
    ForwardDeclaration() : isVirtual(false) {}
    explicit ForwardDeclaration(const std::string& s) : cls(s), isVirtual(false) {}
    std::string name() const { return cls.qualifiedName("::"); }
  };

}
