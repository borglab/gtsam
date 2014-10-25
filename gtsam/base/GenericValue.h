/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file GenericValue.h
 * @date Jan 26, 2012
 * @author Michael Bosse, Abel Gawel, Renaud Dube
 * based on DrivedValue.h by Duy Nguyen Ta
 */

#pragma once

#include <gtsam/base/Value.h>
#include <boost/make_shared.hpp>

//////////////////
// The following includes windows.h in some MSVC versions, so we undef min, max, and ERROR
#include <boost/pool/singleton_pool.hpp>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#ifdef ERROR
#undef ERROR
#endif
//////////////////


namespace gtsam {

namespace traits {

// trait to wrap the default equals of types
template<typename T>
  bool equals(const T& a, const T& b, double tol) {
    return a.equals(b,tol);
  }

// trait to compute the local coordinates of other with respect to origin
template<typename T>
Vector localCoordinates(const T& origin, const T& other) {
  return origin.localCoordinates(other);
}

template<typename T>
T retract(const T& origin, const Vector& delta) {
  return origin.retract(delta);
}

template<typename T>
void print(const T& obj, const std::string& str) {
  obj.print(str);
}

template<typename T>
size_t getDimension(const T& obj) {
  return obj.dim();
}
}

template<class T>
class GenericValue : public Value {
public:
  typedef T ValueType;
  typedef GenericValue This;
protected:
  T value_;

public:
  GenericValue() {}
  GenericValue(const T& value) : value_(value) {}

  T& value() { return value_; }
  const T& value() const { return value_; }

  virtual ~GenericValue() {}

  /**
   * Create a duplicate object returned as a pointer to the generic Value interface.
   * For the sake of performance, this function use singleton pool allocator instead of the normal heap allocator.
   * The result must be deleted with Value::deallocate_, not with the 'delete' operator.
   */
  virtual Value* clone_() const {
    void *place = boost::singleton_pool<PoolTag, sizeof(This)>::malloc();
    This* ptr = new(place) This(*this);
    return ptr;
  }

  /**
   * Destroy and deallocate this object, only if it was originally allocated using clone_().
   */
  virtual void deallocate_() const {
    this->~GenericValue(); // Virtual destructor cleans up the derived object
    boost::singleton_pool<PoolTag, sizeof(This)>::free((void*)this); // Release memory from pool
  }

  /**
   * Clone this value (normal clone on the heap, delete with 'delete' operator)
   */
  virtual boost::shared_ptr<Value> clone() const {
    return boost::make_shared<This>(*this);
  }

  /// equals implementing generic Value interface
  virtual bool equals_(const Value& p, double tol = 1e-9) const {
    // Cast the base class Value pointer to a templated generic class pointer
    const This& genericValue2 = static_cast<const This&>(p);

    // Return the result of using the equals traits for the derived class
    return traits::equals<T>(this->value_, genericValue2.value_, tol);

  }

  /// Generic Value interface version of retract
  virtual Value* retract_(const Vector& delta) const {
    // Call retract on the derived class using the retract trait function
    const T retractResult = traits::retract<T>(value_,delta);

    // Create a Value pointer copy of the result
    void* resultAsValuePlace = boost::singleton_pool<PoolTag, sizeof(This)>::malloc();
    Value* resultAsValue = new(resultAsValuePlace) This(retractResult);

    // Return the pointer to the Value base class
    return resultAsValue;
  }

  /// Generic Value interface version of localCoordinates
  virtual Vector localCoordinates_(const Value& value2) const {
    // Cast the base class Value pointer to a templated generic class pointer
    const This& genericValue2 = static_cast<const This&>(value2);

    // Return the result of calling localCoordinates trait on the derived class
    return traits::localCoordinates<T>(value_,genericValue2.value_);
  }

  virtual void print(const std::string& str) const {
    traits::print<T>(value_,str);
  }
  virtual size_t dim() const {
    return traits::getDimension<T>(value_); // need functional form here since the dimension may be dynamic
  }

  /// Assignment operator
  virtual Value& operator=(const Value& rhs) {
    // Cast the base class Value pointer to a derived class pointer
    const This& derivedRhs = static_cast<const This&>(rhs);

    // Do the assignment and return the result
    this->value_ = derivedRhs.value_;
    return *this;
  }

  /// Conversion to the derived class
  operator const T& () const {
    return value_;
  }

  /// Conversion to the derived class
  operator T& () {
    return value_;
  }


protected:
  /// Assignment operator, protected because only the Value or DERIVED
  /// assignment operators should be used.
//  DerivedValue<DERIVED>& operator=(const DerivedValue<DERIVED>& rhs) {
//    // Nothing to do, do not call base class assignment operator
//    return *this;
//  }

private:
  /// Fake Tag struct for singleton pool allocator. In fact, it is never used!
  struct PoolTag { };
};

// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
 const ValueType& Value::cast() const {
   return dynamic_cast<const GenericValue<ValueType>&>(*this).value();
 }


} /* namespace gtsam */
