/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     print_examples.h
 * @brief    Example library file
 * @author   Richard Roberts
 */

#pragma once

#include <string>

namespace example {

class PrintExamples {
public:
  /// Print a greeting
  void sayHello() const;

  /// Print a farewell
  void sayGoodbye() const;
};

namespace internal {

std::string getHelloString();

std::string getGoodbyeString();

}  // namespace internal

}  // namespace example
