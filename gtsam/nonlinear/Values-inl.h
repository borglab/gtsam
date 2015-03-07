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

  /* ************************************************************************* */
  template<class ValueType>
  struct _ValuesKeyValuePair {
    const Key key; ///< The key
    ValueType& value;  ///< The value

    _ValuesKeyValuePair(Key _key, ValueType& _value) : key(_key), value(_value) {}
  };

  /* ************************************************************************* */
  template<class ValueType>
  struct _ValuesConstKeyValuePair {
    const Key key; ///< The key
    const ValueType& value;  ///< The value

    _ValuesConstKeyValuePair(Key _key, const ValueType& _value) : key(_key), value(_value) {}
    _ValuesConstKeyValuePair(const _ValuesKeyValuePair<ValueType>& rhs) : key(rhs.key), value(rhs.value) {}
  };

  /* ************************************************************************* */
  template<class ValueType>
  class Values::Filtered {
  public:
    /** A key-value pair, with the value a specific derived Value type. */
    typedef _ValuesKeyValuePair<ValueType> KeyValuePair;
    typedef _ValuesConstKeyValuePair<ValueType> ConstKeyValuePair;
    typedef KeyValuePair value_type;

    typedef
      boost::transform_iterator<
      KeyValuePair(*)(Values::KeyValuePair),
      boost::filter_iterator<
      boost::function<bool(const Values::ConstKeyValuePair&)>,
      Values::iterator> >
      iterator;

    typedef iterator const_iterator;

    typedef
      boost::transform_iterator<
      ConstKeyValuePair(*)(Values::ConstKeyValuePair),
      boost::filter_iterator<
      boost::function<bool(const Values::ConstKeyValuePair&)>,
      Values::const_iterator> >
      const_const_iterator;

    iterator begin() { return begin_; }
    iterator end() { return end_; }
    const_iterator begin() const { return begin_; }
    const_iterator end() const { return end_; }
    const_const_iterator beginConst() const { return constBegin_; }
    const_const_iterator endConst() const { return constEnd_; }

    /** Returns the number of values in this view */
    size_t size() const {
      size_t i = 0;
      for (const_const_iterator it = beginConst(); it != endConst(); ++it)
        ++i;
      return i;
    }

  private:
    Filtered(const boost::function<bool(const Values::ConstKeyValuePair&)>& filter, Values& values) :
      begin_(boost::make_transform_iterator(
      boost::make_filter_iterator(
      filter, values.begin(), values.end()),
      &castHelper<ValueType, KeyValuePair, Values::KeyValuePair>)),
      end_(boost::make_transform_iterator(
      boost::make_filter_iterator(
      filter, values.end(), values.end()),
      &castHelper<ValueType, KeyValuePair, Values::KeyValuePair>)),
      constBegin_(boost::make_transform_iterator(
      boost::make_filter_iterator(
      filter, ((const Values&)values).begin(), ((const Values&)values).end()),
      &castHelper<const ValueType, ConstKeyValuePair, Values::ConstKeyValuePair>)),
      constEnd_(boost::make_transform_iterator(
      boost::make_filter_iterator(
      filter, ((const Values&)values).end(), ((const Values&)values).end()),
      &castHelper<const ValueType, ConstKeyValuePair, Values::ConstKeyValuePair>)) {}

    friend class Values;
    iterator begin_;
    iterator end_;
    const_const_iterator constBegin_;
    const_const_iterator constEnd_;
  };

  /* ************************************************************************* */
  template<class ValueType>
  class Values::ConstFiltered {
  public:
    /** A const key-value pair, with the value a specific derived Value type. */
    typedef _ValuesConstKeyValuePair<ValueType> KeyValuePair;
    typedef KeyValuePair value_type;

    typedef typename Filtered<ValueType>::const_const_iterator iterator;
    typedef typename Filtered<ValueType>::const_const_iterator const_iterator;

    /** Conversion from Filtered to ConstFiltered */
    ConstFiltered(const Filtered<ValueType>& rhs) :
      begin_(rhs.beginConst()),
      end_(rhs.endConst()) {}

    iterator begin() { return begin_; }
    iterator end() { return end_; }
    const_iterator begin() const { return begin_; }
    const_iterator end() const { return end_; }

    /** Returns the number of values in this view */
    size_t size() const {
      size_t i = 0;
      for (const_iterator it = begin(); it != end(); ++it)
        ++i;
      return i;
    }

    FastList<Key> keys() const {
      FastList<Key> result;
      for(const_iterator it = begin(); it != end(); ++it)
        result.push_back(it->key);
      return result;
    }

  private:
    friend class Values;
    const_iterator begin_;
    const_iterator end_;
    ConstFiltered(const boost::function<bool(const Values::ConstKeyValuePair&)>& filter, const Values& values) {
      // We remove the const from values to create a non-const Filtered
      // view, then pull the const_iterators out of it.
      const Filtered<ValueType> filtered(filter, const_cast<Values&>(values));
      begin_ = filtered.beginConst();
      end_ = filtered.endConst();
    }
  };

  /* ************************************************************************* */
  /** Constructor from a Filtered view copies out all values */
  template<class ValueType>
  Values::Values(const Values::Filtered<ValueType>& view) {
    BOOST_FOREACH(const typename Filtered<ValueType>::KeyValuePair& key_value, view) {
      Key key = key_value.key;
      insert(key, key_value.value);
    }
  }

  /* ************************************************************************* */
  template<class ValueType>
  Values::Values(const Values::ConstFiltered<ValueType>& view) {
    BOOST_FOREACH(const typename ConstFiltered<ValueType>::KeyValuePair& key_value, view) {
      Key key = key_value.key;
      insert(key, key_value.value);
    }
  }

  /* ************************************************************************* */
  Values::Filtered<Value>
  inline Values::filter(const boost::function<bool(Key)>& filterFcn) {
    return filter<Value>(filterFcn);
  }

  /* ************************************************************************* */
  template<class ValueType>
  Values::Filtered<ValueType>
  Values::filter(const boost::function<bool(Key)>& filterFcn) {
    return Filtered<ValueType>(boost::bind(&filterHelper<ValueType>, filterFcn, _1), *this);
  }

  /* ************************************************************************* */
  Values::ConstFiltered<Value>
  inline Values::filter(const boost::function<bool(Key)>& filterFcn) const {
    return filter<Value>(filterFcn);
  }

  /* ************************************************************************* */
  template<class ValueType>
  Values::ConstFiltered<ValueType>
  Values::filter(const boost::function<bool(Key)>& filterFcn) const {
    return ConstFiltered<ValueType>(boost::bind(&filterHelper<ValueType>, filterFcn, _1), *this);
  }

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
