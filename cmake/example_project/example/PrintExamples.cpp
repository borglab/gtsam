/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     print_examples.cpp
 * @brief    Example library file
 * @author   Richard Roberts
 */

#include <iostream>

#include <example/PrintExamples.h>

namespace example {

void PrintExamples::sayHello() const {
  std::cout << internal::getHelloString() << std::endl;
}

void PrintExamples::sayGoodbye() const {
  std::cout << internal::getGoodbyeString() << std::endl;
}

namespace internal {

std::string getHelloString() {
  return "Hello!";
}

std::string getGoodbyeString() {
  return "See you soon!";
}

}  // namespace internal

}  // namespace example
