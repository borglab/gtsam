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

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif
#include <boost/bind.hpp>
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
#include <boost/iterator/transform_iterator.hpp>

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
  Values::Values(const Values& other, const VectorValues& delta) {
    for (const_iterator key_value = other.begin(); key_value != other.end(); ++key_value) {
      VectorValues::const_iterator it = delta.find(key_value->key);
      Key key = key_value->key;  // Non-const duplicate to deal with non-const insert argument
      if (it != delta.end()) {
        const Vector& v = it->second;

        Value* retractedValue(key_value->value.retract_(v));  // Retract
        values_.insert(key, retractedValue);  // Add retracted result directly to result values
      } else {
        values_.insert(key, key_value->value.clone_());  // Add original version to result values
      }
    }
  }
//======================= Values(is_mh) for mhRetract() =======================
  //[MH-A]: actually same as the original Values(other, delta), but can be separated for later functionalities...
  Values::Values(Values& other, const VectorValues& delta, const bool& is_mh) { //is_mh should always be true
    for (iterator key_value = other.begin(); key_value != other.end(); ++key_value) {
      VectorValues::const_iterator it = delta.find(key_value->key); //see if this Key required to be updated
      Key key = key_value->key;  // Non-const duplicate to deal with non-const insert argument
      if (it != delta.end()) { //found
        const Vector& v = it->second;
        
        //[MH-A]: MHGV::retract_()
        Value* retractedValue(key_value->value.retract_(v));  // Retract

        values_.insert(key, retractedValue);  // Add retracted result directly to result values
        
      } else {
        values_.insert(key, key_value->value.clone_());  // Add original version to result values
      }

    }
  }
//======================= END Values(is_mh) for mhRetract() =======================
  void Values::mhRetractInPlace(const VectorValues& delta) {
    for (iterator key_value = begin(); key_value != end(); ++key_value) {
      VectorValues::const_iterator it = delta.find(key_value->key); //see if this Key required to be updated
      if (it != delta.end()) { //found
        const Vector& v = it->second;
        
        //[MH-A]: MHGV::retract_()
        key_value->value.retractInPlace_(v);  // Retract

      } // END if
    } // END for
  } // END mhRetractInPlace()

  /* ************************************************************************* */
  void Values::print(const string& str, const KeyFormatter& keyFormatter) const {
    cout << str << "Values with " << size() << " values:" << endl;
    for(const_iterator key_value = begin(); key_value != end(); ++key_value) {
      cout << "Value " << keyFormatter(key_value->key) << ": ";
      key_value->value.print("");
      cout << "\n";
    }
  }

  /* ************************************************************************* */
  bool Values::equals(const Values& other, double tol) const {
    if (this->size() != other.size())
      return false;
    for (const_iterator it1 = this->begin(), it2 = other.begin();
        it1 != this->end(); ++it1, ++it2) {
      const Value& value1 = it1->value;
      const Value& value2 = it2->value;
      if (typeid(value1) != typeid(value2) || it1->key != it2->key
          || !value1.equals_(value2, tol)) {
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
//========================= mhRetract() ======================  
  //[MH-A]:
  Values Values::mhRetract(const VectorValues& delta) {
    return Values(*this, delta, true);
  }
//========================= END mhRetract() ======================  

  /* ************************************************************************* */
  VectorValues Values::localCoordinates(const Values& cp) const {
    if(this->size() != cp.size())
      throw DynamicValuesMismatched();
    VectorValues result;
    for(const_iterator it1=this->begin(), it2=cp.begin(); it1!=this->end(); ++it1, ++it2) {
      if(it1->key != it2->key)
        throw DynamicValuesMismatched(); // If keys do not match
      // Will throw a dynamic_cast exception if types do not match
      // NOTE: this is separate from localCoordinates(cp, ordering, result) due to at() vs. insert
      result.insert(it1->key, it1->value.localCoordinates_(it2->value));
    }
    return result;
  }

  /* ************************************************************************* */
  const Value& Values::at(Key j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    // Throw exception if it does not exist
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("retrieve", j);
    return *item->second;
  }

//====================== non-const at() ========================
  //[MH-A]: MH has to modify the values
  Value& Values::at(Key j) {
    // Find the item
    KeyValueMap::iterator item = values_.find(j);

    // Throw exception if it does not exist
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("retrieve", j);
    return *item->second;
  }
//====================== END non-const at() ========================

  /* ************************************************************************* */
  void Values::insert(Key j, const Value& val) {
    std::pair<iterator,bool> insertResult = tryInsert(j, val);
    if(!insertResult.second)
      throw ValuesKeyAlreadyExists(j);
  }
  
  //=========================== MH ============================
  //[MH-A]:
  void Values::mhInsert(Key j, const Value& val) {
    std::pair<iterator,bool> insertResult = tryInsert(j, val);
    if(!insertResult.second)
      throw ValuesKeyAlreadyExists(j);

    //[MH-A]: link from HypoTree
    
    const HypoList& hypo_list = val.getHypoList();
    
    const GenericList& generic_list = val.getGenericList(); 
  
    if (generic_list.size() != hypo_list.size()) {
        std::cout << "ERROR: generic_list.size() != hypo_list.size()... Make sure to call mhCalculateBestEstimate() after pruning if the MHGVs are used later!!" << std::endl;
    }
    
    GenericListCstIter git = generic_list.begin();
    
    for (HypoListCstIter hit = hypo_list.begin(); hit != hypo_list.end(); ++hit) {
     
      // Setup Key-Value* map in each HypoNode
      (*hit)->addKeyValuePair(j, (*git));
      
      ++git; 
    }
  }
  //=========================== END MH ============================

  /* ************************************************************************* */
  void Values::insert(const Values& values) {
    for(const_iterator key_value = values.begin(); key_value != values.end(); ++key_value) {
      Key key = key_value->key; // Non-const duplicate to deal with non-const insert argument
      insert(key, key_value->value); //NOT calling insert(Key, T)
    }
  }

//========================= MH ================================
  //[MH-A]:
  void Values::mhInsert(const Values& values) {
    for(const_iterator key_value = values.begin(); key_value != values.end(); ++key_value) {
      Key key = key_value->key; // Non-const duplicate to deal with non-const insert argument
      mhInsert(key, key_value->value); //NOT calling insert(Key, T)
      
      (key_value->value.getHypoLayer())->setKeyMap(key);

    }
  }


//========================= END MH ================================

  /* ************************************************************************* */
  std::pair<Values::iterator, bool> Values::tryInsert(Key j, const Value& value) {
    std::pair<KeyValueMap::iterator, bool> result = values_.insert(j, value.clone_());
    return std::make_pair(boost::make_transform_iterator(result.first, &make_deref_pair), result.second);
  }

  /* ************************************************************************* */
  void Values::update(Key j, const Value& val) {
    // Find the value to update
    KeyValueMap::iterator item = values_.find(j);
    if (item == values_.end())
      throw ValuesKeyDoesNotExist("update", j);

    // Cast to the derived type
    const Value& old_value = *item->second;
    if (typeid(old_value) != typeid(val))
      throw ValuesIncorrectType(j, typeid(old_value), typeid(val));

    values_.replace(item, val.clone_());
  }

  /* ************************************************************************* */
  void Values::update(const Values& values) {
    for(const_iterator key_value = values.begin(); key_value != values.end(); ++key_value) {
      this->update(key_value->key, key_value->value);
    }
  }

  /* ************************************************************************* */
  void Values::erase(Key j) {
    KeyValueMap::iterator item = values_.find(j);
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("erase", j);
    values_.erase(item);
  }

  /* ************************************************************************* */
  KeyVector Values::keys() const {
    KeyVector result;
    result.reserve(size());
    for(const_iterator key_value = begin(); key_value != end(); ++key_value)
      result.push_back(key_value->key);
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
    for(const ConstKeyValuePair& key_value: *this) {
      result += key_value.value.dim();
    }
    return result;
  }

  /* ************************************************************************* */
  VectorValues Values::zeroVectors() const {
    VectorValues result;
    for(const ConstKeyValuePair& key_value: *this)
      result.insert(key_value.key, Vector::Zero(key_value.value.hypoNum()*key_value.value.dim())); //the hypoNum() for original Value is 1
    return result;
  }

  /* ************************************************************************* */
  const char* ValuesKeyAlreadyExists::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to add a key-value pair with key \"" + DefaultKeyFormatter(key_) + "\", key already exists.";
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* ValuesKeyDoesNotExist::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to " + std::string(operation_) + " the key \"" +
          DefaultKeyFormatter(key_) + "\", which does not exist in the Values.";
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* ValuesIncorrectType::what() const throw() {
    if(message_.empty())
      message_ =
          "Attempting to retrieve value with key \"" + DefaultKeyFormatter(key_) + "\", type stored in Values is " +
          std::string(storedTypeId_.name()) + " but requested type was " + std::string(requestedTypeId_.name());
    return message_.c_str();
  }

  /* ************************************************************************* */
  const char* NoMatchFoundForFixed::what() const throw() {
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
