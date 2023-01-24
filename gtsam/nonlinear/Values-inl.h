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
#include <gtsam/nonlinear/Values.h>

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
  template <class ValueType>
  size_t Values::count() const {
    size_t i = 0;
    for (const auto& [_, value] : values_) {
      if (dynamic_cast<const GenericValue<ValueType>*>(value.get())) ++i;
    }
    return i;
  }

  /* ************************************************************************* */
  template <class ValueType>
  std::map<Key, ValueType>
  Values::extract(const std::function<bool(Key)>& filterFcn) const {
    std::map<Key, ValueType> result;
    for (const auto& [key,value] : values_) {
      // Check if key matches
      if (filterFcn(key)) {
        // Check if type matches (typically does as symbols matched with types)
        if (auto t =
                dynamic_cast<const GenericValue<ValueType>*>(value.get()))
          result[key] = t->value();
      }
    }
    return result;
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
     return h(j, item->second.get());
  }

  /* ************************************************************************* */
  template<typename ValueType>
  const ValueType * Values::exists(Key j) const {
    // Find the item
    KeyValueMap::const_iterator item = values_.find(j);

    if(item != values_.end()) {
      const Value* value = item->second.get();
      // dynamic cast the type and throw exception if incorrect
      auto ptr = dynamic_cast<const GenericValue<ValueType>*>(value);
      if (ptr) {
        return &ptr->value();
      } else {
        // NOTE(abe): clang warns about potential side effects if done in typeid
        throw ValuesIncorrectType(j, typeid(*value), typeid(ValueType));
      }
     } else {
      return nullptr;
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
