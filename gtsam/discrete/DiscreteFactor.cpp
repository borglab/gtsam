/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteFactor.cpp
 * @brief discrete factor
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteFactor.h>

#include <sstream>

using namespace std;

namespace gtsam {

string DiscreteFactor::Translate(const Names& names, Key key, size_t index) {
  if (names.empty()) {
    stringstream ss;
    ss << index;
    return ss.str();
  } else {
    return names.at(key)[index];
  }
}

}  // namespace gtsam
