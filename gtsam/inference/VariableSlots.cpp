/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    VariableSlots.cpp
 * @author  Richard Roberts
 * @date    Oct 5, 2010
 */

#include <gtsam/inference/VariableSlots.h>

#include <iostream>
#include <limits>

using namespace std;

namespace gtsam {

const size_t VariableSlots::Empty = numeric_limits<size_t>::max();

/** print */
void VariableSlots::print(const std::string& str) const {
  if(this->empty())
    cout << "empty" << endl;
  else {
    cout << str << "\n";
    cout << "Var:\t";
    for(const value_type& slot: *this) { cout << slot.first << "\t"; }
    cout << "\n";

    for(size_t i=0; i<this->begin()->second.size(); ++i) {
      cout << "    \t";
      for(const value_type& slot: *this) {
        if(slot.second[i] == Empty)
          cout << "x" << "\t";
        else
          cout << slot.second[i] << "\t";
      }
      cout << "\n";
    }
  }
}

/** equals */
bool VariableSlots::equals(const VariableSlots& rhs, double tol) const {
  return *this == rhs;
}

}
