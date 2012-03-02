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

#pragma once

#include <utility>

#include <gtsam/base/DerivedValue.h>
#include <gtsam/nonlinear/Values.h> // Only so Eclipse finds class definition

namespace gtsam {

  /* ************************************************************************* */
  class ValueCloneAllocator {
  public:
    static Value* allocate_clone(const Value& a) { return a.clone_(); }
    static void deallocate_clone(const Value* a) { a->deallocate_(); }
  private:
    ValueCloneAllocator() {}
  };

#if 0
  /* ************************************************************************* */
  class ValueAutomaticCasting {
    Key key_;
    const Value& value_;

  public:
    ValueAutomaticCasting(Key key, const Value& value) : key_(key), value_(value) {}

    template<class ValueType>
    class ConvertibleToValue : public ValueType {
    };

    template<class ValueType>
    operator const ConvertibleToValue<ValueType>& () const {
      // Check the type and throw exception if incorrect
      if(typeid(ValueType) != typeid(value_))
        throw ValuesIncorrectType(key_, typeid(ValueType), typeid(value_));

      // We have already checked the type, so do a "blind" static_cast, not dynamic_cast
      return static_cast<const ConvertibleToValue<ValueType>&>(value_);
    }
  };
#endif

  /* ************************************************************************* */
  template<typename ValueType>
  const ValueType& Values::at(Key j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    // Throw exception if it does not exist
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("retrieve", j);

    // Check the type and throw exception if incorrect
    if(typeid(*item->second) != typeid(ValueType))
      throw ValuesIncorrectType(j, typeid(*item->second), typeid(ValueType));

    // We have already checked the type, so do a "blind" static_cast, not dynamic_cast
    return static_cast<const ValueType&>(*item->second);
  }

#if 0
  /* ************************************************************************* */
  inline ValueAutomaticCasting Values::at(Key j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    // Throw exception if it does not exist
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("retrieve", j);

    return ValueAutomaticCasting(item->first, *item->second);
  }
#endif

#if 0
  /* ************************************************************************* */
  inline ValueAutomaticCasting Values::operator[](Key j) const {
    return at(j);
  }
#endif

  /* ************************************************************************* */
  template<typename ValueType>
  boost::optional<const ValueType&> Values::exists(Key j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    if(item != values_.end()) {
      // Check the type and throw exception if incorrect
      if(typeid(*item->second) != typeid(ValueType))
        throw ValuesIncorrectType(j, typeid(*item->second), typeid(ValueType));

      // We have already checked the type, so do a "blind" static_cast, not dynamic_cast
      return static_cast<const ValueType&>(*item->second);
    } else {
      return boost::none;
    }
  }

}
