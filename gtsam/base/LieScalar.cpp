/*
 * LieScalar.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: thduynguyen
 */




#include <gtsam/base/deprecated/LieScalar_Deprecated.h>

namespace gtsam {
  void LieScalar::print(const std::string& name) const {
    std::cout << name << ": " << d_ << std::endl;
  }
}
