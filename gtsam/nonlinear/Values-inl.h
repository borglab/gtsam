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

#include <utility>
#include <boost/type_traits/conditional.hpp>
#include <boost/type_traits/is_base_of.hpp>

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

  /* ************************************************************************* */
  class ValueAutomaticCasting {
    const Symbol& key_;
    const Value& value_;
  public:
    ValueAutomaticCasting(const Symbol& key, const Value& value) : key_(key), value_(value) {}

    template<class ValueType>
    operator const ValueType& () const {
      // Check the type and throw exception if incorrect
      if(typeid(ValueType) != typeid(value_))
        throw ValuesIncorrectType(key_, typeid(ValueType), typeid(value_));

      // We have already checked the type, so do a "blind" static_cast, not dynamic_cast
      return static_cast<const ValueType&>(value_);
    }
  };

//  /* ************************************************************************* */
//  template<class ValueType>
//  ValueType& operator=(ValueType& lhs, const ValueAutomaticCasting& rhs) {
//    lhs = rhs;
//  }

  /* ************************************************************************* */
  template<typename ValueType>
  const ValueType& Values::at(const Symbol& j) const {
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

  /* ************************************************************************* */
  inline ValueAutomaticCasting Values::at(const Symbol& j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    // Throw exception if it does not exist
    if(item == values_.end())
      throw ValuesKeyDoesNotExist("retrieve", j);

    return ValueAutomaticCasting(item->first, *item->second);
  }

  /* ************************************************************************* */
  template<typename TypedKey>
  const typename TypedKey::Value& Values::at(const TypedKey& j) const {
    // Convert to Symbol
    const Symbol symbol(j.symbol());

    // Call at with the Value type from the key
    return at<typename TypedKey::Value>(symbol);
  }

  /* ************************************************************************* */
  inline ValueAutomaticCasting Values::operator[](const Symbol& j) const {
    return at(j);
  }

  /* ************************************************************************* */
  template<typename ValueType>
  boost::optional<const ValueType&> Values::exists(const Symbol& j) const {
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

  /* ************************************************************************* */
  template<class TypedKey>
  boost::optional<const typename TypedKey::Value&> Values::exists(const TypedKey& j) const {
    // Convert to Symbol
    const Symbol symbol(j.symbol());

    // Call exists with the Value type from the key
    return exists<typename TypedKey::Value>(symbol);
  }

}
