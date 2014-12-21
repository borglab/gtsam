/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file GenericValue.h
 * @brief Wraps any type T so it can play as a Value
 * @date October, 2014
 * @author Michael Bosse, Abel Gawel, Renaud Dube
 * based on DerivedValue.h by Duy Nguyen Ta
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Value.h>

#include <boost/make_shared.hpp>
#include <boost/pool/pool_alloc.hpp>

#include <cmath>
#include <iostream>

namespace gtsam {

// To play as a GenericValue, we need the following traits
namespace traits {

// trait to wrap the default equals of types
template<typename T>
struct equals {
  typedef T type;
  typedef bool result_type;
  bool operator()(const T& a, const T& b, double tol) {
    return a.equals(b, tol);
  }
};

// trait to wrap the default print of types
template<typename T>
struct print {
  typedef T type;
  typedef void result_type;
  void operator()(const T& obj, const std::string& str) {
    obj.print(str);
  }
};

// equals for scalars
template<>
struct equals<double> {
  typedef double type;
  typedef bool result_type;
  bool operator()(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
  }
};

// print for scalars
template<>
struct print<double> {
  typedef double type;
  typedef void result_type;
  void operator()(double a, const std::string& str) {
    std::cout << str << ": " << a << std::endl;
  }
};

// equals for Matrix types
template<int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct equals<Eigen::Matrix<double, _Rows, _Cols, _Options, _MaxRows, _MaxCols> > {
  typedef Eigen::Matrix<double, _Rows, _Cols, _Options, _MaxRows, _MaxCols> type;
  typedef bool result_type;
  bool operator()(const type& A, const type& B, double tol) {
    return equal_with_abs_tol(A, B, tol);
  }
};

// print for Matrix types
template<int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct print<Eigen::Matrix<double, _Rows, _Cols, _Options, _MaxRows, _MaxCols> > {
  typedef Eigen::Matrix<double, _Rows, _Cols, _Options, _MaxRows, _MaxCols> type;
  typedef void result_type;
  void operator()(const type& A, const std::string& str) {
    std::cout << str << ": " << A << std::endl;
  }
};

}

/**
 * Wraps any type T so it can play as a Value
 */
template<class T>
class GenericValue: public Value {

public:

  typedef T type;

protected:

  T value_; ///< The wrapped value

public:
  // Only needed for serialization.
  GenericValue(){}

  /// Construct from value
  GenericValue(const T& value) :
      value_(value) {
  }

  /// Return a constant value
  const T& value() const {
    return value_;
  }

  /// Return the value
  T& value() {
    return value_;
  }

  /// Destructor
  virtual ~GenericValue() {
  }

  /// equals implementing generic Value interface
  virtual bool equals_(const Value& p, double tol = 1e-9) const {
    // Cast the base class Value pointer to a templated generic class pointer
    const GenericValue& genericValue2 = static_cast<const GenericValue&>(p);
    // Return the result of using the equals traits for the derived class
    return traits::equals<T>()(this->value_, genericValue2.value_, tol);
  }

  /// non virtual equals function, uses traits
  bool equals(const GenericValue &other, double tol = 1e-9) const {
    return traits::equals<T>()(this->value(), other.value(), tol);
  }

  /// Virtual print function, uses traits
  virtual void print(const std::string& str) const {
    traits::print<T>()(value_, str);
  }

    /**
     * Create a duplicate object returned as a pointer to the generic Value interface.
     * For the sake of performance, this function use singleton pool allocator instead of the normal heap allocator.
     * The result must be deleted with Value::deallocate_, not with the 'delete' operator.
     */
    virtual Value* clone_() const {
      void *place = boost::singleton_pool<PoolTag, sizeof(GenericValue)>::malloc();
      GenericValue* ptr = new (place) GenericValue(*this); // calls copy constructor to fill in
      return ptr;
    }

    /**
     * Destroy and deallocate this object, only if it was originally allocated using clone_().
     */
    virtual void deallocate_() const {
      this->~GenericValue(); // Virtual destructor cleans up the derived object
      boost::singleton_pool<PoolTag, sizeof(GenericValue)>::free((void*) this); // Release memory from pool
    }

    /**
     * Clone this value (normal clone on the heap, delete with 'delete' operator)
     */
    virtual boost::shared_ptr<Value> clone() const {
      return boost::make_shared<GenericValue>(*this);
    }

    /// Generic Value interface version of retract
    virtual Value* retract_(const Vector& delta) const {
      // Call retract on the derived class using the retract trait function
      const T retractResult = traits_x<T>::Retract(GenericValue<T>::value(), delta);

      // Create a Value pointer copy of the result
      void* resultAsValuePlace =
          boost::singleton_pool<PoolTag, sizeof(GenericValue)>::malloc();
      Value* resultAsValue = new (resultAsValuePlace) GenericValue(retractResult);

      // Return the pointer to the Value base class
      return resultAsValue;
    }

    /// Generic Value interface version of localCoordinates
    virtual Vector localCoordinates_(const Value& value2) const {
      // Cast the base class Value pointer to a templated generic class pointer
      const GenericValue<T>& genericValue2 =
          static_cast<const GenericValue<T>&>(value2);

      // Return the result of calling localCoordinates trait on the derived class
      return traits_x<T>::Local(GenericValue<T>::value(), genericValue2.value());
    }

    /// Non-virtual version of retract
    GenericValue retract(const Vector& delta) const {
      return GenericValue(traits_x<T>::Retract(GenericValue<T>::value(), delta));
    }

    /// Non-virtual version of localCoordinates
    Vector localCoordinates(const GenericValue& value2) const {
      return localCoordinates_(value2);
    }

    /// Return run-time dimensionality
    virtual size_t dim() const {
      // need functional form here since the dimension may be dynamic
      return traits_x<T>::GetDimension(GenericValue<T>::value());
    }

    /// Assignment operator
    virtual Value& operator=(const Value& rhs) {
      // Cast the base class Value pointer to a derived class pointer
      const GenericValue& derivedRhs = static_cast<const GenericValue&>(rhs);

      // Do the assignment and return the result
      *this = GenericValue(derivedRhs); // calls copy constructor
      return *this;
    }

  protected:

    // implicit assignment operator for (const GenericValue& rhs) works fine here
    /// Assignment operator, protected because only the Value or DERIVED
    /// assignment operators should be used.
    //  DerivedValue<DERIVED>& operator=(const DerivedValue<DERIVED>& rhs) {
    //    // Nothing to do, do not call base class assignment operator
    //    return *this;
    //  }

  private:

    /// Fake Tag struct for singleton pool allocator. In fact, it is never used!
    struct PoolTag {
    };

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("GenericValue",
              boost::serialization::base_object<Value>(*this));
      ar & boost::serialization::make_nvp("value", value_);
    }

};

// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
const ValueType& Value::cast() const {
  return dynamic_cast<const GenericValue<ValueType>&>(*this).value();
}

} /* namespace gtsam */
