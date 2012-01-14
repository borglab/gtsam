/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Value.h
 * @brief The interface class for any variable that can be optimized or used in a factor.
 * @author Richard Roberts
 * @date Jan 14, 2012
 */

#pragma once

#include <memory>

#include <gtsam/base/Vector.h>

namespace gtsam {

  /**
   * This is the interface class for any value that may be used as a variable
   * assignment in a factor graph, and which you must derive to create new
   * variable types to use with gtsam.  Examples of built-in classes
   * implementing this are mainly in geometry, including Rot3, Pose2, etc.
   *
   * This interface specifies pure virtual retract_ and localCoordinates_
   * functions that work with pointers to this interface class.  When you
   * implement these functions in the derived class, the objects behind these
   * pointers must always be instances of the proper derived class:
   * \code
     // This is example code you would never write, but illustrates that
     // consistent derived value instances are passed to and from retract
     // and localCoordinates as base class (Value) pointers.

     // This is a base class pointer that is actually a Rot3:
     auto_ptr<Value> value = new Rot3();

     // The retract implementation must always returns a Rot3 instance as a
     // base class pointer, i.e. this code must run successfully:
     auto_ptr<Value> retracted = value->retract_(delta);
     Rot3* rot3retracted = dynamic_cast<Rot3*>(retracted.get());

     // localCoordinates will always be passed a derived class instance as
     // a base class pointer:
     Vector coordinates = value->localCoordinates_(retracted);
     \endcode
   *
   * The reason we have require functions is so that containers, such as
   * Values can operate generically on Value objects, retracting or computing
   * local coordinates for many Value objects of different types.
   *
   * When you implement retract_() and localCoordinates_(), we suggest first
   * implementing versions of these functions that work directly with derived
   * objects, then using the provided helper functions to implement the
   * generic Value versions.  This makes your implementation easier, and also
   * improves performance in situations where the derived type is in fact
   * known, such as in most implementations of \c evaluateError() in classes
   * derived from NonlinearFactor.
   *
   * Using the above practice, here is an example of implementing a typical
   * class derived from Value:
   * \code
     class Rot3 : public Value {
     public:
       // Constructor, there is never a need to call the Value base class constructor.
       Rot3() { ... }

       // Tangent space dimensionality (virtual, overrides Value::dim())
       virtual size_t dim() cosnt {
         return 3;
       }

       // retract working directly with Rot3 objects (non-virtual, non-overriding!)
       Rot3 retract(const Vector& delta) const {
         // Math to implement a 3D rotation retraction e.g. exponential map
         return Rot3(result);
       }

       // localCoordinates working directly with Rot3 objects (non-virtual, non-overriding!)
       Vector localCoordinates(const Rot3& r2) const {
         // Math to implement 3D rotation localCoordinates, e.g. logarithm map
         return Vector(result);
       }

       // retract implementing the generic Value interface (virtual, overrides Value::retract())
       virtual std::auto_ptr<Value> retract_(const Vector& delta) const {
         // Call our provided helper function to call your Rot3-specific
         // retract and do the appropriate casting and allocation.
         return CallDerivedRetract(this, delta);
       }

       // localCoordinates implementing the generic Value interface (virtual, overrides Value::localCoordinates())
       virtual Vector localCoordinates_(const Value& value) const {
         // Call our provided helper function to call your Rot3-specific
         // localCoordinates and do the appropriate casting.
         return CallDerivedLocalCoordinates(this, value);
       }
     };
     \endcode
   */
  class Value {
  public:
    /** Return the dimensionality of the tangent space of this value.  This is
     * the dimensionality of \c delta passed into retract() and of the vector
     * returned by localCoordinates().
     * @return The dimensionality of the tangent space
     */
    virtual size_t dim() const = 0;

    /** Increment the value, by mapping from the vector delta in the tangent
     * space of the current value back to the manifold to produce a new,
     * incremented value.
     * @param delta The delta vector in the tangent space of this value, by
     * which to increment this value.
     */
    virtual std::auto_ptr<Value> retract_(const Vector& delta) const = 0;

    /** Compute the coordinates in the tangent space of this value that
     * retract() would map to \c value.
     * @param value The value whose coordinates should be determined in the
     * tangent space of the value on which this function is called.
     * @return The coordinates of \c value in the tangent space of \c this.
     */
    virtual Vector localCoordinates_(const Value& value) const = 0;

    /** Virutal destructor */
    virtual ~Value() {}

  protected:
    /** This is a convenience function to make it easy for you to implement the
     * generic Value inferface, see the example at the top of the Value
     * documentation.
     * @param derived A pointer to the derived class on which to call retract
     * @param delta The delta vector to pass to the derived retract
     * @return The result of retract on the derived class, stored as a Value pointer
     */
    template<class Derived>
    static std::auto_ptr<Value> CallDerivedRetract(const Derived* derived, const Vector& delta) {
      // Call retract on the derived class
      const Derived retractResult = derived->retract(delta);

      // Create a Value pointer copy of the result
      std::auto_ptr<Value> resultAsValue(new Derived(retractResult));

      // Return the pointer to the Value base class
      return resultAsValue;
    }

    /** This is a convenience function to make it easy for you to implement the
     * generic Value inferface, see the example at the top of the Value
     * documentation.
     * @param value1 The object on which to call localCoordinates, stored as a Value pointer
     * @param value2 The argument to pass to the derived localCoordinates function
     * @return The result of localCoordinates of the derived class
     */
    template<class Derived>
    static Vector CallDerivedLocalCoordinates(const Derived* value1, const Value& value2) {
      // Cast the base class Value pointer to a derived class pointer
      const Derived& derivedValue2 = dynamic_cast<const Derived&>(value2);

      // Return the result of calling localCoordinates on the derived class
      return value1->localCoordinates(derivedValue2);
    }
  };

} /* namespace gtsam */
