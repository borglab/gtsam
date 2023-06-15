/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteKey.h
 * @brief specialized key for discrete variables
 * @author Frank Dellaert
 * @date Feb 28, 2011
 */

#include <iostream>
#include "DiscreteKey.h"

namespace gtsam {

  using namespace std;

  DiscreteKeys::DiscreteKeys(const vector<int>& cs) {
    for (size_t i = 0; i < cs.size(); i++) {
      string name = "v" + std::to_string(i);
      push_back(DiscreteKey(i, cs[i]));
    }
  }

  KeyVector DiscreteKeys::indices() const {
    KeyVector js;
    for (const DiscreteKey& key : *this) js.push_back(key.first);
    return js;
  }

  map<Key, size_t> DiscreteKeys::cardinalities() const {
    map<Key, size_t> cs;
    cs.insert(begin(), end());
    return cs;
  }

  DiscreteKeys operator&(const DiscreteKey& key1, const DiscreteKey& key2) {
    DiscreteKeys keys(key1);
    return keys & key2;
  }

  void DiscreteKeys::print(const std::string& s,
                           const KeyFormatter& keyFormatter) const {
    for (auto&& dkey : *this) {
      std::cout << DefaultKeyFormatter(dkey.first) << " " << dkey.second
                << std::endl;
    }
  }

  bool DiscreteKeys::equals(const DiscreteKeys& other, double tol) const {
    if (this->size() != other.size()) {
      return false;
    }

    for (size_t i = 0; i < this->size(); i++) {
      if (this->at(i).first != other.at(i).first ||
          this->at(i).second != other.at(i).second) {
        return false;
      }
    }
    return true;
  }
}
