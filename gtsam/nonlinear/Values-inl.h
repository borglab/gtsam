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

#ifdef __JETBRAINS_IDE__
#include <gtsam/nonlinear/Values.h> // Only so Eclipse finds class definition
#endif

#include <range/v3/view.hpp>

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

   namespace internal {

   // Check the type and throw exception if incorrect
   // Generic version, partially specialized below for various Eigen Matrix types
   template <typename ValueType>
   struct handle {
     ValueType operator()(Key j, const Value* const pointer) {
       try {
         // value returns a const ValueType&, and the return makes a copy !!!!!
         return dynamic_cast<const GenericValue<ValueType>&>(*pointer).value();
       } catch (std::bad_cast&) {
         throw ValuesIncorrectType(j, typeid(*pointer), typeid(ValueType));
       }
     }
   };

   template <typename MatrixType, bool isDynamic>
   struct handle_matrix;

   // Handle dynamic matrices
   template <int M, int N>
   struct handle_matrix<Eigen::Matrix<double, M, N>, true> {
     Eigen::Matrix<double, M, N> operator()(Key j, const Value* const pointer) {
       try {
         // value returns a const Matrix&, and the return makes a copy !!!!!
         return dynamic_cast<const GenericValue<Eigen::Matrix<double, M, N>>&>(*pointer).value();
       } catch (std::bad_cast&) {
         // If a fixed matrix was stored, we end up here as well.
         throw ValuesIncorrectType(j, typeid(*pointer), typeid(Eigen::Matrix<double, M, N>));
       }
     }
   };

   // Handle fixed matrices
   template <int M, int N>
   struct handle_matrix<Eigen::Matrix<double, M, N>, false> {
     Eigen::Matrix<double, M, N> operator()(Key j, const Value* const pointer) {
       try {
         // value returns a const MatrixMN&, and the return makes a copy !!!!!
         return dynamic_cast<const GenericValue<Eigen::Matrix<double, M, N>>&>(*pointer).value();
       } catch (std::bad_cast&) {
         Matrix A;
         try {
           // Check if a dynamic matrix was stored
           A = handle_matrix<Eigen::MatrixXd, true>()(j, pointer);  // will throw if not....
         } catch (const ValuesIncorrectType&) {
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
      const Value& value = *item->second;
      try {
        return dynamic_cast<const GenericValue<ValueType>&>(value).value();
      } catch (std::bad_cast &) {
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

}
