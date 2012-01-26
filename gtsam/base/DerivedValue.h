/*
 * DerivedValue.h
 *
 *  Created on: Jan 26, 2012
 *      Author: thduynguyen
 */

#pragma once

#include <boost/pool/singleton_pool.hpp>
#include <gtsam/base/Value.h>
namespace gtsam {

template<class DERIVED>
class DerivedValue : public Value {
private:
	/// Fake Tag struct for singleton pool allocator. In fact, it is never used!
	struct PoolTag { };

public:

	DerivedValue() {}

	virtual ~DerivedValue() {}

  /**
   * Create a duplicate object returned as a pointer to the generic Value interface.
   * For the shake of performance, this function use singleton pool allocator instead of the normal heap allocator
   */
  virtual Value* clone_() const {
  	void *place = boost::singleton_pool<PoolTag, sizeof(DERIVED)>::malloc();
  	DERIVED* ptr = new(place) DERIVED(static_cast<const DERIVED&>(*this));
  	return ptr;
  }

  /**
   * Create a duplicate object returned as a pointer to the generic Value interface
   */
  virtual void deallocate_() const {
  	boost::singleton_pool<PoolTag, sizeof(DERIVED)>::free((void*)this);
  }

  /// equals implementing generic Value interface
  virtual bool equals_(const Value& p, double tol = 1e-9) const {
    // Cast the base class Value pointer to a derived class pointer
    const DERIVED& derivedValue2 = dynamic_cast<const DERIVED&>(p);

    // Return the result of calling equals on the derived class
    return (static_cast<const DERIVED*>(this))->equals(derivedValue2, tol);
//  	return CallDerivedEquals(this, p, tol);
  }

  /// Generic Value interface version of retract
	virtual std::auto_ptr<Value> retract_(const Vector& delta) const {
    // Call retract on the derived class
    const DERIVED retractResult = (static_cast<const DERIVED*>(this))->retract(delta);

    // Create a Value pointer copy of the result
    std::auto_ptr<Value> resultAsValue(new DERIVED(retractResult));

    // Return the pointer to the Value base class
    return resultAsValue;
	}

	/// Generic Value interface version of localCoordinates
	virtual Vector localCoordinates_(const Value& value2) const {
    // Cast the base class Value pointer to a derived class pointer
    const DERIVED& derivedValue2 = dynamic_cast<const DERIVED&>(value2);

    // Return the result of calling localCoordinates on the derived class
    return (static_cast<const DERIVED*>(this))->localCoordinates(derivedValue2);
	}

};

} /* namespace gtsam */
