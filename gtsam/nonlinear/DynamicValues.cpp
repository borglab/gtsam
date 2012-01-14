/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DynamicValues.h
 * @author Richard Roberts
 *
 * @brief A non-templated config holding any types of Manifold-group elements
 *
 *  Detailed story:
 *  A values structure is a map from keys to values. It is used to specify the value of a bunch
 *  of variables in a factor graph. A Values is a values structure which can hold variables that
 *  are elements on manifolds, not just vectors. It then, as a whole, implements a aggregate type
 *  which is also a manifold element, and hence supports operations dim, retract, and localCoordinates.
 */

#include <gtsam/nonlinear/DynamicValues.h>

#include <boost/foreach.hpp>

using namespace std;

namespace gtsam {

  void DynamicValues::print(const string& str) const {
    cout << str << "DynamicValues with " << size() << " values:\n" << endl;
    BOOST_FOREACH(const KeyValueMap::value_type& key_value, values_) {
      cout << "  " << (string)key_value.first << ": ";
      key_value.second->print("");
    }
  }

  bool DynamicValues::equals(const DynamicValues& other, double tol) const {
    if(this->size() != other.size())
      return false;
    for(const_iterator it1=this->begin(), it2=other.begin(); it1!=this->end(); ++it1, ++it2)
  }

}
