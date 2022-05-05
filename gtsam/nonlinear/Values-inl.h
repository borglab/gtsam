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

#include <boost/bind/bind.hpp>

#include <gtsam/nonlinear/Values.h> // Only so Eclipse finds class definition

namespace gtsam {


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

    _ValuesConstKeyValuePair(Key _key, const ValueType& _value) :
        key(_key), value(_value) {
    }
    _ValuesConstKeyValuePair(const _ValuesKeyValuePair<ValueType>& rhs) :
        key(rhs.key), value(rhs.value) {
    }
  };

  /* ************************************************************************* */

  // Cast helpers for making _Values[Const]KeyValuePair's from Values::[Const]KeyValuePair
  // need to use a struct here for later partial specialization
  template<class ValueType, class CastedKeyValuePairType, class KeyValuePairType>
  struct ValuesCastHelper {
  static CastedKeyValuePairType cast(KeyValuePairType key_value) {
    // Static cast because we already checked the type during filtering
    return CastedKeyValuePairType(key_value.key,
        const_cast<GenericValue<ValueType>&>(static_cast<const GenericValue<
            ValueType>&>(key_value.value)).value());
  }
  };
  // partial specialized version for ValueType == Value
  template<class CastedKeyValuePairType, class KeyValuePairType>
  struct ValuesCastHelper<Value, CastedKeyValuePairType, KeyValuePairType> {
    static CastedKeyValuePairType cast(KeyValuePairType key_value) {
      // Static cast because we already checked the type during filtering
      // in this case the casted and keyvalue pair are essentially the same type
      // (key, Value&) so perhaps this could be done with just a cast of the key_value?
      return CastedKeyValuePairType(key_value.key, key_value.value);
    }
  };
  // partial specialized version for ValueType == Value
  template<class CastedKeyValuePairType, class KeyValuePairType>
  struct ValuesCastHelper<const Value, CastedKeyValuePairType, KeyValuePairType> {
    static CastedKeyValuePairType cast(KeyValuePairType key_value) {
      // Static cast because we already checked the type during filtering
      // in this case the casted and keyvalue pair are essentially the same type
      // (key, Value&) so perhaps this could be done with just a cast of the key_value?
      return CastedKeyValuePairType(key_value.key, key_value.value);
    }
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
      std::function<bool(const Values::ConstKeyValuePair&)>,
      Values::iterator> >
      iterator;

    typedef iterator const_iterator;

    typedef
      boost::transform_iterator<
      ConstKeyValuePair(*)(Values::ConstKeyValuePair),
      boost::filter_iterator<
      std::function<bool(const Values::ConstKeyValuePair&)>,
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
    Filtered(
        const std::function<bool(const Values::ConstKeyValuePair&)>& filter,
        Values& values) :
        begin_(
            boost::make_transform_iterator(
                boost::make_filter_iterator(filter, values.begin(), values.end()),
                &ValuesCastHelper<ValueType, KeyValuePair, Values::KeyValuePair>::cast)), end_(
            boost::make_transform_iterator(
                boost::make_filter_iterator(filter, values.end(), values.end()),
                &ValuesCastHelper<ValueType, KeyValuePair, Values::KeyValuePair>::cast)), constBegin_(
            boost::make_transform_iterator(
                boost::make_filter_iterator(filter,
                    ((const Values&) values).begin(),
                    ((const Values&) values).end()),
                &ValuesCastHelper<ValueType, ConstKeyValuePair,
                    Values::ConstKeyValuePair>::cast)), constEnd_(
            boost::make_transform_iterator(
                boost::make_filter_iterator(filter,
                    ((const Values&) values).end(),
                    ((const Values&) values).end()),
                &ValuesCastHelper<ValueType, ConstKeyValuePair,
                    Values::ConstKeyValuePair>::cast)) {
    }

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
    ConstFiltered(
        const std::function<bool(const Values::ConstKeyValuePair&)>& filter,
        const Values& values) {
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
    for(const auto key_value: view) {
      Key key = key_value.key;
      insert(key, static_cast<const ValueType&>(key_value.value));
    }
  }

  /* ************************************************************************* */
  template<class ValueType>
  Values::Values(const Values::ConstFiltered<ValueType>& view) {
    for(const auto key_value: view) {
      Key key = key_value.key;
      insert(key, static_cast<const ValueType&>(key_value.value));
    }
  }

  /* ************************************************************************* */
  Values::Filtered<Value>
  inline Values::filter(const std::function<bool(Key)>& filterFcn) {
    return filter<Value>(filterFcn);
  }

  /* ************************************************************************* */
  template<class ValueType>
  Values::Filtered<ValueType>
  Values::filter(const std::function<bool(Key)>& filterFcn) {
    return Filtered<ValueType>(std::bind(&filterHelper<ValueType>, filterFcn,
      std::placeholders::_1), *this);
  }

  /* ************************************************************************* */
  Values::ConstFiltered<Value>
  inline Values::filter(const std::function<bool(Key)>& filterFcn) const {
    return filter<Value>(filterFcn);
  }

  /* ************************************************************************* */
  template<class ValueType>
  Values::ConstFiltered<ValueType>
  Values::filter(const std::function<bool(Key)>& filterFcn) const {
    return ConstFiltered<ValueType>(std::bind(&filterHelper<ValueType>,
      filterFcn, std::placeholders::_1), *this);
  }

  /* ************************************************************************* */
   template<>
   inline bool Values::filterHelper<Value>(const std::function<bool(Key)> filter,
       const ConstKeyValuePair& key_value) {
     // Filter and check the type
     return filter(key_value.key);
   }

   /* ************************************************************************* */

   namespace internal {

   // Check the type and throw exception if incorrect
   // Generic version, partially specialized below for various Eigen Matrix types
   template <typename ValueType>
   struct handle {
     ValueType operator()(Key j, const Value* const pointer) {
       auto ptr = dynamic_cast<const GenericValue<ValueType>*>(pointer);
       if (ptr) {
         // value returns a const ValueType&, and the return makes a copy !!!!!
         return ptr->value();
       } else {
         throw ValuesIncorrectType(j, typeid(*pointer), typeid(ValueType));
       }
     }
   };

   template <typename MatrixType, bool isDynamic>
   struct handle_matrix;

   // Handle dynamic matrices
   template <int M, int N>
   struct handle_matrix<Eigen::Matrix<double, M, N>, true> {
     inline Eigen::Matrix<double, M, N> operator()(Key j, const Value* const pointer) {
       auto ptr = dynamic_cast<const GenericValue<Eigen::Matrix<double, M, N>>*>(pointer);
       if (ptr) {
         // value returns a const Matrix&, and the return makes a copy !!!!!
         return ptr->value();
       } else {
         // If a fixed matrix was stored, we end up here as well.
         throw ValuesIncorrectType(j, typeid(*pointer), typeid(Eigen::Matrix<double, M, N>));
       }
     }
   };

   // Handle fixed matrices
   template <int M, int N>
   struct handle_matrix<Eigen::Matrix<double, M, N>, false> {
     inline Eigen::Matrix<double, M, N> operator()(Key j, const Value* const pointer) {
       auto ptr = dynamic_cast<const GenericValue<Eigen::Matrix<double, M, N>>*>(pointer);
       if (ptr) {
         // value returns a const MatrixMN&, and the return makes a copy !!!!!
         return ptr->value();
       } else {
         Matrix A;
         // Check if a dynamic matrix was stored
         auto ptr = dynamic_cast<const GenericValue<Eigen::MatrixXd>*>(pointer);
         if (ptr) {
           A = ptr->value();
         } else {
           // Or a dynamic vector
           A = handle_matrix<Eigen::VectorXd, true>()(j, pointer);  // will throw if not....
         }
         // Yes: check size, and throw if not a match
         if (A.rows() != M || A.cols() != N)
           throw NoMatchFoundForFixed(M, N, A.rows(), A.cols());
         else
           return A; // copy but not malloc
       }
     }
   };

   // Handle matrices
   template <int M, int N>
   struct handle<Eigen::Matrix<double, M, N>> {
     Eigen::Matrix<double, M, N> operator()(Key j, const Value* const pointer) {
       return handle_matrix<Eigen::Matrix<double, M, N>,
                            (M == Eigen::Dynamic || N == Eigen::Dynamic)>()(j, pointer);
     }
   };

   }  // internal

   /* ************************************************************************* */
   template <typename ValueType>
   const ValueType Values::at(Key j) const {
     // Find the item
     KeyValueMap::const_iterator item = values_.find(j);

     // Throw exception if it does not exist
     if (item == values_.end()) throw ValuesKeyDoesNotExist("at", j);

     // Check the type and throw exception if incorrect
     // h() split in two lines to avoid internal compiler error (MSVC2017)
     auto h = internal::handle<ValueType>();
     return h(j, item->second);
  }

  /* ************************************************************************* */
  template<typename ValueType>
  boost::optional<const ValueType&> Values::exists(Key j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    if(item != values_.end()) {
      // dynamic cast the type and throw exception if incorrect
      auto ptr = dynamic_cast<const GenericValue<ValueType>*>(item->second);
      if (ptr) {
        return ptr->value();
      } else {
        // NOTE(abe): clang warns about potential side effects if done in typeid
        const Value* value = item->second;
        throw ValuesIncorrectType(j, typeid(*value), typeid(ValueType));
      }
     } else {
      return boost::none;
    }
  }

  /* ************************************************************************* */

  // insert a templated value
  template<typename ValueType>
  void Values::insert(Key j, const ValueType& val) {
    insert(j, static_cast<const Value&>(GenericValue<ValueType>(val)));
  }

  // update with templated value
  template <typename ValueType>
  void Values::update(Key j, const ValueType& val) {
    update(j, static_cast<const Value&>(GenericValue<ValueType>(val)));
  }

  // insert_or_assign with templated value
  template <typename ValueType>
  void Values::insert_or_assign(Key j, const ValueType& val) {
    insert_or_assign(j, static_cast<const Value&>(GenericValue<ValueType>(val)));
  }

}
