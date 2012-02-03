/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Values.h
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

#include <gtsam/nonlinear/Values.h>

#include <list>

#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <boost/iterator/transform_iterator.hpp>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  Values::Values(const Values& other) {
    this->insert(other);
  }

  /* ************************************************************************* */
  void Values::print(const string& str) const {
    cout << str << "Values with " << size() << " values:\n" << endl;
    for(const_iterator key_value = begin(); key_value != end(); ++key_value) {
      cout << "  " << (string)key_value->first << ": ";
      key_value->second.print("");
    }
  }

  /* ************************************************************************* */
  bool Values::equals(const Values& other, double tol) const {
    if(this->size() != other.size())
      return false;
    for(const_iterator it1=this->begin(), it2=other.begin(); it1!=this->end(); ++it1, ++it2) {
      if(typeid(it1->second) != typeid(it2->second))
        return false;
      if(it1->first != it2->first)
        return false;
      if(!it1->second.equals_(it2->second, tol))
        return false;
    }
    return true; // We return false earlier if we find anything that does not match
  }

  /* ************************************************************************* */
  bool Values::exists(const Symbol& j) const {
    return values_.find(j) != values_.end();
  }

  /* ************************************************************************* */
  VectorValues Values::zeroVectors(const Ordering& ordering) const {
    return VectorValues::Zero(this->dims(ordering));
  }

  /* ************************************************************************* */
  Values Values::retract(const VectorValues& delta, const Ordering& ordering) const {
    Values result;

    for(const_iterator key_value = begin(); key_value != end(); ++key_value) {
      const SubVector& singleDelta = delta[ordering[key_value->first]]; // Delta for this value
      Symbol key = key_value->first;  // Non-const duplicate to deal with non-const insert argument
      Value* retractedValue(key_value->second.retract_(singleDelta)); // Retract
      result.values_.insert(key, retractedValue); // Add retracted result directly to result values
    }

    return result;
  }

  /* ************************************************************************* */
  VectorValues Values::localCoordinates(const Values& cp, const Ordering& ordering) const {
    VectorValues result(this->dims(ordering));
    localCoordinates(cp, ordering, result);
    return result;
  }

  /* ************************************************************************* */
  void Values::localCoordinates(const Values& cp, const Ordering& ordering, VectorValues& result) const {
    if(this->size() != cp.size())
      throw DynamicValuesMismatched();
    for(const_iterator it1=this->begin(), it2=cp.begin(); it1!=this->end(); ++it1, ++it2) {
      if(it1->first != it2->first)
        throw DynamicValuesMismatched(); // If keys do not match
      // Will throw a dynamic_cast exception if types do not match
      result.insert(ordering[it1->first], it1->second.localCoordinates_(it2->second));
    }
  }

  /* ************************************************************************* */
  void Values::insert(const Symbol& j, const Value& val) {
  	Symbol key = j; // Non-const duplicate to deal with non-const insert argument
  	std::pair<iterator,bool> insertResult = values_.insert(key, val.clone_());
  	if(!insertResult.second)
  		throw ValuesKeyAlreadyExists(j);
  }

  /* ************************************************************************* */
  void Values::insert(const Values& values) {
    for(const_iterator key_value = values.begin(); key_value != values.end(); ++key_value) {
      Symbol key = key_value->first; // Non-const duplicate to deal with non-const insert argument
      insert(key, key_value->second);
    }
  }

  /* ************************************************************************* */
  void Values::update(const Symbol& j, const Value& val) {
  	// Find the value to update
  	KeyValueMap::iterator item = values_.find(j);
  	if(item == values_.end())
  		throw ValuesKeyDoesNotExist("update", j);

  	// Cast to the derived type
  	if(typeid(*item->second) != typeid(val))
  		throw ValuesIncorrectType(j, typeid(*item->second), typeid(val));

  	values_.replace(item, val.clone_());
  }

  /* ************************************************************************* */
  void Values::update(const Values& values) {
    for(const_iterator key_value = values.begin(); key_value != values.end(); ++key_value) {
      this->update(key_value->first, key_value->second);
    }
  }

  /* ************************************************************************* */
  void Values::erase(const Symbol& j) {
    KeyValueMap::iterator item = values_.find(j);
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("erase", j);
    values_.erase(item);
  }

  /* ************************************************************************* */
  FastList<Symbol> Values::keys() const {
    FastList<Symbol> result;
    for(const_iterator key_value = begin(); key_value != end(); ++key_value)
      result.push_back(key_value->first);
    return result;
  }

  /* ************************************************************************* */
  Values& Values::operator=(const Values& rhs) {
    this->clear();
    this->insert(rhs);
    return *this;
  }

  /* ************************************************************************* */
  vector<size_t> Values::dims(const Ordering& ordering) const {
    vector<size_t> result(values_.size());
    BOOST_FOREACH(const ConstKeyValuePair& key_value, *this) {
      result[ordering[key_value.first]] = key_value.second.dim();
    }
    return result;
  }

  /* ************************************************************************* */
  Ordering::shared_ptr Values::orderingArbitrary(Index firstVar) const {
    Ordering::shared_ptr ordering(new Ordering);
    for(const_iterator key_value = begin(); key_value != end(); ++key_value) {
      ordering->insert(key_value->first, firstVar++);
    }
    return ordering;
  }

  /* ************************************************************************* */
  const char* ValuesKeyAlreadyExists::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to add a key-value pair with key \"" + (std::string)key_ + "\", key already exists.";
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* ValuesKeyDoesNotExist::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to " + std::string(operation_) + " the key \"" +
          (std::string)key_ + "\", which does not exist in the Values.";
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* ValuesIncorrectType::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to retrieve value with key \"" + (std::string)key_ + "\", type stored in Values is " +
          std::string(storedTypeId_.name()) + " but requested type was " + std::string(requestedTypeId_.name());
    return message_.c_str();
  }

}
