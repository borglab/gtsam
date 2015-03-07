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
#include <boost/serialization/assume_abstract.hpp>
#include <gtsam/base/Vector.h>

namespace gtsam {

  /**
   * This is the interface class for any value that may be used as a variable
   * assignment in a factor graph, and which you must derive to create new
   * variable types to use with gtsam.  Examples of built-in classes
   * implementing this are mainly in geometry, including Rot3, Pose2, etc.
   *
   * This interface specifies pure virtual retract_(), localCoordinates_() and
   * equals_() functions that work with pointers and references to this interface
   * class, i.e. the base class.  These functions allow containers, such as
   * Values can operate generically on Value objects, retracting or computing
   * local coordinates for many Value objects of different types.
   *
   * When you implement retract_(), localCoordinates_(), and equals_(), we
   * suggest first implementing versions of these functions that work directly
   * with derived objects, then using the provided helper functions to
   * implement the generic Value versions.  This makes your implementation
   * easier, and also improves performance in situations where the derived type
   * is in fact known, such as in most implementations of \c evaluateError() in
   * classes derived from NonlinearFactor.
   *
   * Using the above practice, here is an example of implementing a typical
   * class derived from Value:
   * \code
     class Rot3 : public Value {
     public:
       // Constructor, there is never a need to call the Value base class constructor.
       Rot3() { ... }

       // Print for unit tests and debugging (virtual, implements Value::print())
       virtual void print(const std::string& str = "") const;

       // Equals working directly with Rot3 objects (non-virtual, non-overriding!)
       bool equals(const Rot3& other, double tol = 1e-9) const;

       // Tangent space dimensionality (virtual, implements Value::dim())
       virtual size_t dim() const {
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

       // Equals implementing the generic Value interface (virtual, implements Value::equals_())
       virtual bool equals_(const Value& other, double tol = 1e-9) const {
         // Call our provided helper function to call your Rot3-specific
         // equals with appropriate casting.
         return CallDerivedEquals(this, other, tol);
       }

       // retract implementing the generic Value interface (virtual, implements Value::retract_())
       virtual std::auto_ptr<Value> retract_(const Vector& delta) const {
         // Call our provided helper function to call your Rot3-specific
         // retract and do the appropriate casting and allocation.
         return CallDerivedRetract(this, delta);
       }

       // localCoordinates implementing the generic Value interface (virtual, implements Value::localCoordinates_())
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

    /** Clone this value in a special memory pool, must be deleted with Value::deallocate_, *not* with the 'delete' operator. */
    virtual Value* clone_() const = 0;

    /** Deallocate a raw pointer of this value */
    virtual void deallocate_() const = 0;

		/** Clone this value (normal clone on the heap, delete with 'delete' operator) */
		virtual boost::shared_ptr<Value> clone() const = 0;

    /** Compare this Value with another for equality. */
    virtual bool equals_(const Value& other, double tol = 1e-9) const = 0;

    /** Print this value, for debugging and unit tests */
    virtual void print(const std::string& str = "") const = 0;

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
    virtual Value* retract_(const Vector& delta) const = 0;

    /** Compute the coordinates in the tangent space of this value that
     * retract() would map to \c value.
     * @param value The value whose coordinates should be determined in the
     * tangent space of the value on which this function is called.
     * @return The coordinates of \c value in the tangent space of \c this.
     */
    virtual Vector localCoordinates_(const Value& value) const = 0;

    /** Assignment operator */
    virtual Value& operator=(const Value& rhs) = 0;

    /** Virutal destructor */
    virtual ~Value() {}

  private:
  	/** Empty serialization function.
  	 *
  	 * There are two important things that users need to do to serialize derived objects in Values successfully:
  	 * (Those derived objects are stored in Values as pointer to this abstract base class Value)
  	 *
  	 * 		1. All DERIVED classes derived from Value must put the following line in their serialization function:
  	 * 			\code
  	  					ar & boost::serialization::make_nvp("DERIVED", boost::serialization::base_object<Value>(*this));
						\endcode
  	 * 			or, alternatively
  	 * 			\code
  	  		      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Value);
  	  		  \endcode
  	 * 			See: http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#runtimecasting
  	 *
  	 * 		2. The source module that includes archive class headers to serialize objects of derived classes
  	 * 		 (boost/archive/text_oarchive.h, for example) must *export* all derived classes, using either
  	 * 		 BOOST_CLASS_EXPORT or BOOST_CLASS_EXPORT_GUID macros:
  	 	 	 	 	 \code
  	  					BOOST_CLASS_EXPORT(DERIVED_CLASS_1)
  	  					BOOST_CLASS_EXPORT_GUID(DERIVED_CLASS_2, "DERIVED_CLASS_2_ID_STRING")
  	 	 	 	 	 \endcode
  	 * 		  See: 	http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#derivedpointers
  	 * 		  			http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#export
  	 * 		  			http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#instantiation\
  	 * 		  			http://www.boost.org/doc/libs/release/libs/serialization/doc/special.html#export
  	 * 		  			http://www.boost.org/doc/libs/release/libs/serialization/doc/traits.html#export
  	 * 		  The last two links explain why these export lines have to be in the same source module that includes
  	 * 		  any of the archive class headers.
  	 * */
  	friend class boost::serialization::access;
  	template<class ARCHIVE>
  	void serialize(ARCHIVE & ar, const unsigned int version) {}

  };

} /* namespace gtsam */

BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::Value)
