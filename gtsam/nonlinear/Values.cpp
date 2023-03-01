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
#include <gtsam/linear/VectorValues.h>

#include <list>
#include <memory>
#include <sstream>

using namespace std;

namespace gtsam {

  /* ************************************************************************* */
  Values::Values(const Values& other) {
    this->insert(other);
  }

  /* ************************************************************************* */
  Values::Values(Values&& other) : values_(std::move(other.values_)) {
  }

  /* ************************************************************************* */
  Values::Values(std::initializer_list<ConstKeyValuePair> init) {
    for (const auto &kv : init)
      insert(kv.key, kv.value);
  }

  /* ************************************************************************* */
  Values::Values(const Values& other, const VectorValues& delta) {
    for (const auto& [key,value] : other.values_) {
      VectorValues::const_iterator it = delta.find(key);
      if (it != delta.end()) {
        const Vector& v = it->second;
        Value* retractedValue(value->retract_(v));  // Retract
        values_.emplace(key, retractedValue);  // Add retracted result directly to result values
      } else {
        values_.emplace(key, value->clone_());  // Add original version to result values
      }
    }
  }

  /* ************************************************************************* */
  void Values::print(const string& str, const KeyFormatter& keyFormatter) const {
    cout << str << (str.empty() ? "" : "\n");
    cout << "Values with " << size() << " values:\n";
    for (const auto& [key,value] : values_) {
      cout << "Value " << keyFormatter(key) << ": ";
      value->print("");
      cout << "\n";
    }
  }

  /* ************************************************************************* */
  bool Values::equals(const Values& other, double tol) const {
    if (this->size() != other.size())
      return false;
    for (auto it1 = values_.begin(), it2 = other.values_.begin();
         it1 != values_.end(); ++it1, ++it2) {
      const Value* value1 = it1->second.get();
      const Value* value2 = it2->second.get();
      if (typeid(*value1) != typeid(*value2) || it1->first != it2->first
          || !value1->equals_(*value2, tol)) {
        return false;
      }
    }
    return true; // We return false earlier if we find anything that does not match
}

  /* ************************************************************************* */
  bool Values::exists(Key j) const {
    return values_.find(j) != values_.end();
  }

  /* ************************************************************************* */
  Values Values::retract(const VectorValues& delta) const {
    return Values(*this, delta);
  }

  /* ************************************************************************* */
  void Values::retractMasked(const VectorValues& delta, const KeySet& mask) {
    gttic(retractMasked);
    assert(this->size() == delta.size());
    auto key_value = values_.begin();
    VectorValues::const_iterator key_delta;
#ifdef GTSAM_USE_TBB
    for (; key_value != values_.end(); ++key_value) {
      key_delta = delta.find(key_value->first);
#else
    for (key_delta = delta.begin(); key_value != values_.end();
         ++key_value, ++key_delta) {
      assert(key_value->first == key_delta->first);
#endif
      Key var = key_value->first;
      assert(static_cast<size_t>(delta[var].size()) == key_value->second->dim());
      assert(delta[var].allFinite());
      if (mask.exists(var)) {
        Value* retracted = key_value->second->retract_(delta[var]);
        // TODO(dellaert): can we use std::move here?
        *(key_value->second) = *retracted;
        retracted->deallocate_();
      }
    }
  }

  /* ************************************************************************* */
  VectorValues Values::localCoordinates(const Values& cp) const {
    if(this->size() != cp.size())
      throw DynamicValuesMismatched();
    VectorValues result;
    for (auto it1 = values_.begin(), it2 = cp.values_.begin();
         it1 != values_.end(); ++it1, ++it2) {
      if(it1->first != it2->first)
        throw DynamicValuesMismatched(); // If keys do not match
      // Will throw a dynamic_cast exception if types do not match
      // NOTE: this is separate from localCoordinates(cp, ordering, result) due to at() vs. insert
      result.insert(it1->first, it1->second->localCoordinates_(*it2->second));
    }
    return result;
  }

  /* ************************************************************************* */
  const Value& Values::at(Key j) const {
    KeyValueMap::const_iterator it = values_.find(j);

    // Throw exception if it does not exist
    if(it == values_.end())
      throw ValuesKeyDoesNotExist("retrieve", j);
    return *it->second;
  }

  /* ************************************************************************* */
  void Values::insert(Key j, const Value& val) {
    auto insertResult = values_.emplace(j, val.clone_());
    if(!insertResult.second)
      throw ValuesKeyAlreadyExists(j);
  }

  /* ************************************************************************* */
  void Values::insert(const Values& other) {
    for (const auto& [key, value] : other.values_) {
      insert(key, *(value));
    }
  }

  /* ************************************************************************* */
  void Values::update(Key j, const Value& val) {
    // Find the value to update
    KeyValueMap::iterator it = values_.find(j);
    if (it == values_.end())
      throw ValuesKeyDoesNotExist("update", j);

    // Cast to the derived type
    const Value& old_value = *it->second;
    if (typeid(old_value) != typeid(val))
      throw ValuesIncorrectType(j, typeid(old_value), typeid(val));

    values_.erase(j);
    values_.emplace(j, val.clone_());
  }

  /* ************************************************************************* */
  void Values::update(const Values& other) {
    for (auto& [key, value] : other.values_) {
      this->update(key, *(value));
    }
  }

  /* ************************************************************************ */
  void Values::insert_or_assign(Key j, const Value& val) {
    if (this->exists(j)) {
      // If key already exists, perform an update.
      this->update(j, val);
    } else {
      // If key does not exist, perform an insert.
      this->insert(j, val);
    }
  }

  /* ************************************************************************ */
  void Values::insert_or_assign(const Values& other) {
    for (auto& [key, value] : other.values_) {
      this->insert_or_assign(key, *(value));
    }
  }

  /* ************************************************************************* */
  void Values::erase(Key j) {
    KeyValueMap::iterator it = values_.find(j);
    if(it == values_.end())
      throw ValuesKeyDoesNotExist("erase", j);
    values_.erase(it);
  }

  /* ************************************************************************* */
  KeyVector Values::keys() const {
    KeyVector result;
    result.reserve(size());
    for(const auto& [key,value]: values_)
      result.push_back(key);
    return result;
  }

  /* ************************************************************************* */
  KeySet Values::keySet() const {
    KeySet result;
    for(const auto& [key,value]: values_)
      result.insert(key);
    return result;
  }

  /* ************************************************************************* */
  Values& Values::operator=(const Values& rhs) {
    this->clear();
    this->insert(rhs);
    return *this;
  }

  /* ************************************************************************* */
  size_t Values::dim() const {
    size_t result = 0;
    for (const auto& [key,value] : values_) {
      result += value->dim();
    }
    return result;
  }

  /* ************************************************************************* */
  std::map<Key,size_t> Values::dims() const {
    std::map<Key,size_t> result;
    for (const auto& [key,value] : values_) {
      result.emplace(key, value->dim());
    }
    return result;
  }

  /* ************************************************************************* */
  VectorValues Values::zeroVectors() const {
    VectorValues result;
    for (const auto& [key,value] : values_)
      result.insert(key, Vector::Zero(value->dim()));
    return result;
  }

  /* ************************************************************************* */
  const char* ValuesKeyAlreadyExists::what() const noexcept {
    if(message_.empty())
      message_ =
          "Attempting to add a key-value pair with key \"" + DefaultKeyFormatter(key_) + "\", key already exists.";
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* ValuesKeyDoesNotExist::what() const noexcept {
    if(message_.empty())
      message_ =
          "Attempting to " + std::string(operation_) + " the key \"" +
          DefaultKeyFormatter(key_) + "\", which does not exist in the Values.";
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* ValuesIncorrectType::what() const noexcept {
    if(message_.empty()) {
      std::string storedTypeName = demangle(storedTypeId_.name());
      std::string requestedTypeName = demangle(requestedTypeId_.name());

      if (storedTypeName == requestedTypeName) {
        message_ = "WARNING: Detected types with same name but different `typeid`. \
          This is usually caused by incorrect linking/inlining settings when compiling libraries using GTSAM. \
          If you are a user, please report to the author of the library using GTSAM. \
          If you are a package maintainer, please consult `cmake/GtsamPybindWrap.cmake`, line 74 for details.";
      } else {
        message_ =
        "Attempting to retrieve value with key \"" + DefaultKeyFormatter(key_) + "\", type stored in Values is " +
         storedTypeName + " but requested type was " + requestedTypeName;
      }
    }
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* NoMatchFoundForFixed::what() const noexcept {
    if(message_.empty()) {
      ostringstream oss;
    oss
        << "Attempting to retrieve fixed-size matrix with dimensions " //
        << M1_ << "x" << N1_
        << ", but found dynamic Matrix with mismatched dimensions " //
        << M2_ << "x" << N2_;
      message_ = oss.str();
    }
    return message_.c_str();
  }

}
