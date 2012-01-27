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

#include <utility>

#include <gtsam/nonlinear/DynamicValues.h> // Only so Eclipse finds class definition

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
  template<typename ValueType>
  const ValueType& DynamicValues::at(const Symbol& j) const {
    // Find the item
    const_iterator item = values_.find(j);

    // Throw exception if it does not exist
    if(item == values_.end())
      throw DynamicValuesKeyDoesNotExist("retrieve", j);

    // Check the type and throw exception if incorrect
    if(typeid(*item->second) != typeid(ValueType))
      throw DynamicValuesIncorrectType(j, typeid(*item->second), typeid(ValueType));

    // We have already checked the type, so do a "blind" static_cast, not dynamic_cast
    return static_cast<const ValueType&>(*item->second);
  }

  /* ************************************************************************* */
  template<typename TypedKey>
  const typename TypedKey::Value& DynamicValues::at(const TypedKey& j) const {
    // Convert to Symbol
    const Symbol symbol(j.symbol());

    // Call at with the Value type from the key
    return at<typename TypedKey::Value>(symbol);
  }

  /* ************************************************************************* */
  template<typename ValueType>
  boost::optional<const ValueType&> DynamicValues::exists(const Symbol& j) const {
    // Find the item
    const_iterator item = values_.find(j);

    if(item != values_.end()) {
      // Check the type and throw exception if incorrect
      if(typeid(*item->second) != typeid(ValueType))
        throw DynamicValuesIncorrectType(j, typeid(*item->second), typeid(ValueType));

      // We have already checked the type, so do a "blind" static_cast, not dynamic_cast
      return static_cast<const ValueType&>(*item->second);
    } else {
      return boost::none;
    }
  }

  /* ************************************************************************* */
  template<class TypedKey>
  boost::optional<const typename TypedKey::Value&> DynamicValues::exists(const TypedKey& j) const {
    // Convert to Symbol
    const Symbol symbol(j.symbol());

    // Call exists with the Value type from the key
    return exists<typename TypedKey::Value>(symbol);
  }

}
