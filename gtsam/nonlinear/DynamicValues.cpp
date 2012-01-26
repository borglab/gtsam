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

#include <list>

#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/transform_iterator.hpp>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  DynamicValues::DynamicValues(const DynamicValues& other) {
    this->insert(other);
  }

  /* ************************************************************************* */
  void DynamicValues::print(const string& str) const {
    cout << str << "DynamicValues with " << size() << " values:\n" << endl;
    BOOST_FOREACH(const KeyValueMap::value_type& key_value, values_) {
      cout << "  " << (string)key_value.first << ": ";
      key_value.second->print("");
    }
  }

  /* ************************************************************************* */
  bool DynamicValues::equals(const DynamicValues& other, double tol) const {
    if(this->size() != other.size())
      return false;
    for(const_iterator it1=this->begin(), it2=other.begin(); it1!=this->end(); ++it1, ++it2) {
      if(typeid(*it1->second) != typeid(*it2->second))
        return false;
      if(it1->first != it2->first)
        return false;
      if(!it1->second->equals_(*it2->second, tol))
        return false;
    }
    return true; // We return false earlier if we find anything that does not match
  }

  /* ************************************************************************* */
  bool DynamicValues::exists(const Symbol& j) const {
    return values_.find(j) != values_.end();
  }

  /* ************************************************************************* */
  VectorValues DynamicValues::zeroVectors(const Ordering& ordering) const {
    return VectorValues::Zero(this->dims(ordering));
  }

  /* ************************************************************************* */
  DynamicValues DynamicValues::retract(const VectorValues& delta, const Ordering& ordering) const {
    DynamicValues result;

    BOOST_FOREACH(const KeyValuePair& key_value, values_) {
      const SubVector& singleDelta = delta[ordering[key_value.first]]; // Delta for this value
      ValuePtr retractedValue(key_value.second->retract_(singleDelta)); // Retract
      result.values_[key_value.first] = retractedValue; // Add retracted result directly to result values
    }

    return result;
  }

  /* ************************************************************************* */
  VectorValues DynamicValues::localCoordinates(const DynamicValues& cp, const Ordering& ordering) const {
    VectorValues result(this->dims(ordering));
    localCoordinates(cp, ordering, result);
    return result;
  }

  /* ************************************************************************* */
  void DynamicValues::localCoordinates(const DynamicValues& cp, const Ordering& ordering, VectorValues& result) const {
    if(this->size() != cp.size())
      throw DynamicValuesMismatched();
    for(const_iterator it1=this->begin(), it2=cp.begin(); it1!=this->end(); ++it1, ++it2) {
      if(it1->first != it2->first)
        throw DynamicValuesMismatched(); // If keys do not match
      result[ordering[it1->first]] = it1->second->localCoordinates_(*it2->second); // Will throw a dynamic_cast exception if types do not match
    }
  }

  /* ************************************************************************* */
  void DynamicValues::insert(const DynamicValues& values) {
    BOOST_FOREACH(const KeyValuePair& key_value, values.values_) {
      values_.insert(make_pair(key_value.first, key_value.second->clone_()));
    }
  }

  /* ************************************************************************* */
  void DynamicValues::update(const DynamicValues& values) {
    BOOST_FOREACH(const KeyValuePair& key_value, values) {
      this->update(key_value.first, *key_value.second);
    }
  }

  /* ************************************************************************* */
  void DynamicValues::erase(const Symbol& j) {
    iterator item = values_.find(j);
    if(item == values_.end())
      throw DynamicValuesKeyDoesNotExist("erase", j);
    values_.erase(item);
  }

  /* ************************************************************************* */
  FastList<Symbol> DynamicValues::keys() const {
    return FastList<Symbol>(
        boost::make_transform_iterator(values_.begin(), boost::bind(&KeyValuePair::first, _1)),
        boost::make_transform_iterator(values_.end(), boost::bind(&KeyValuePair::first, _1)));
  }

  /* ************************************************************************* */
  DynamicValues& DynamicValues::operator=(const DynamicValues& rhs) {
    this->clear();
    this->insert(rhs);
    return *this;
  }

  /* ************************************************************************* */
  vector<size_t> DynamicValues::dims(const Ordering& ordering) const {
    vector<size_t> result(values_.size());
    // Transform with Value::dim(auto_ptr::get(KeyValuePair::second))
    result.assign(
        boost::make_transform_iterator(values_.begin(),
            boost::bind(&Value::dim, boost::bind(&ValuePtr::get, boost::bind(&KeyValuePair::second, _1)))),
        boost::make_transform_iterator(values_.begin(),
            boost::bind(&Value::dim, boost::bind(&ValuePtr::get, boost::bind(&KeyValuePair::second, _1)))));
    return result;
  }

  /* ************************************************************************* */
  Ordering::shared_ptr DynamicValues::orderingArbitrary(Index firstVar) const {
    Ordering::shared_ptr ordering(new Ordering);
    BOOST_FOREACH(const KeyValuePair& key_value, values_) {
      ordering->insert(key_value.first, firstVar++);
    }
    return ordering;
  }
}
