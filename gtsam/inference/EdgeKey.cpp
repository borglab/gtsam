/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file EdgeKey.cpp
 * @date Oct 24, 2024
 * @author: Frank Dellaert
 * @author: Akshay Krishnan
 */

#include <gtsam/inference/EdgeKey.h>

namespace gtsam {

// Output stream operator implementation
GTSAM_EXPORT std::ostream& operator<<(std::ostream& os, const EdgeKey& key) {
  os << "{" << key.i() << ", " << key.j() << "}";
  return os;
}

void EdgeKey::print(const std::string& s) const {
  std::cout << s << *this << std::endl;
}

}  // namespace gtsam
