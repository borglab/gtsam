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

#include <gtsam_unstable/base/Dummy.h>
#include <iostream>

namespace gtsam {

static size_t gDummyCount = 0;

Dummy::Dummy():id(++gDummyCount) {
  std::cout << "Dummy constructor " << id << std::endl;
}

Dummy::~Dummy() {
  std::cout << "Dummy destructor " << id << std::endl;
}

void Dummy::print(const std::string& s) const {
  std::cout << s << "Dummy " << id << std::endl;
}

unsigned char Dummy::dummyTwoVar(unsigned char a) const {
  return a;
}

}
