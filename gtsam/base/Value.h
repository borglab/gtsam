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

#include <iostream>

#include <gtsam/inference/HypoTree.h>

#include <gtsam/config.h>      // Configuration from CMake

#include <gtsam/base/Vector.h>
#include <boost/serialization/assume_abstract.hpp>
#include <memory>
#include <vector>

namespace gtsam {

class HypoNode;
class HypoLayer;
class HypoTree;

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
   * Inheriting from the DerivedValue class template provides a generic implementation of
   * the pure virtual functions retract_(), localCoordinates_(), and equals_(), eliminating
   * the need to implement these functions in your class. Note that you must inherit from
   * DerivedValue templated on the class you are defining. For example you cannot define
   * the following
   * \code
   * class Rot3 : public DerivedValue<Point3>{ \\classdef }
   * \endcode
   *
   * Using the above practice, here is an example of implementing a typical
   * class derived from Value:
   * \code
     class GTSAM_EXPORT Rot3 : public DerivedValue<Rot3> {
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
     };
     \endcode
   */
  class GTSAM_EXPORT Value {
  public:
    
    //[MH-A]: 
    typedef std::list<HypoNode*> HypoList; //for virtual getHypoList()
    
    typedef boost::shared_ptr<Value> sharedImplyGeneric;
   
    typedef std::list<sharedImplyGeneric> GenericList;
        
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
    
    //[MH-A]: 
    virtual size_t hypoNum() const = 0; //added for zeroVector()
    
    virtual const HypoList& getHypoList() const {
      std::cout << "const Value::getHypoList() return empty" << std::endl;
      HypoList* empty_list = new HypoList();
      return *empty_list; //should NEVER be called
    }
     
    virtual HypoLayer* getHypoLayer() const {
      std::cout << "const Value::getHypoLayer() return NULL" << std::endl;
      return NULL; //should NEVER be called
    }
    virtual const GenericList& getGenericList() const {
      std::cout << "Value::getGenericList() return empty" << std::endl;
      GenericList* empty_list = new GenericList();
      return *empty_list; //should NEVER be called
    }
    
    //[MH-A]: used in mhSolve()
    virtual void mergeHypoAndSetAgree(std::vector<Vector>& vec_arr, const int& max_layer_idx, const Key& key, const double& splitThreshold) {
      std::cout << "Value::mergeHypoAndSetAgree() should NEVER be called" << std::endl;
    }
    
    virtual bool removeAccumulatedPruned() {
      std::cout << "Value::removeAccumulatedPruned() should NEVER be called" << std::endl;
      return false;
    }

    //[MH-C]:
    virtual void setAgreeWith(const Key& key, HypoLayer* target_layer) {
      std::cout << "Value::setAgreeWith() should NEVER be called" << std::endl;
    }

    /** Increment the value, by mapping from the vector delta in the tangent
     * space of the current value back to the manifold to produce a new,
     * incremented value.
     * @param delta The delta vector in the tangent space of this value, by
     * which to increment this value.
     */
    virtual Value* retract_(const Vector& delta) const = 0;
    
    virtual Value* retractInPlace_(const Vector& delta) {
      Value* empty_value = NULL;
      std::cout << "Value::retractInPlace_() should NEVER be called" << std::endl;
      return empty_value;
    }
    
    virtual Value* this_() {
      Value* empty_value = NULL;
      std::cout << "Value::this_() should NEVER be called" << std::endl;
      return empty_value;
    }

    /** Compute the coordinates in the tangent space of this value that
     * retract() would map to \c value.
     * @param value The value whose coordinates should be determined in the
     * tangent space of the value on which this function is called.
     * @return The coordinates of \c value in the tangent space of \c this.
     */
    virtual Vector localCoordinates_(const Value& value) const = 0;

    /** Assignment operator */
    virtual Value& operator=(const Value& /*rhs*/) {
      //needs a empty definition so recursion in implicit derived assignment operators work
     return *this;
    }

    /** Cast to known ValueType */
    template<typename ValueType>
    const ValueType& cast() const;

    /** Virutal destructor */
    virtual ~Value() {}

  private:
    /** Empty serialization function.
     *
     * There are two important things that users need to do to serialize derived objects in Values successfully:
     * (Those derived objects are stored in Values as pointer to this abstract base class Value)
     *
     *     1. All DERIVED classes derived from Value must put the following line in their serialization function:
     *       \code
                ar & boost::serialization::make_nvp("DERIVED", boost::serialization::base_object<Value>(*this));
            \endcode
     *       or, alternatively
     *       \code
                ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Value);
            \endcode
     *       See: http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#runtimecasting
     *
     *     2. The source module that includes archive class headers to serialize objects of derived classes
     *      (boost/archive/text_oarchive.h, for example) must *export* all derived classes, using either
     *      BOOST_CLASS_EXPORT or BOOST_CLASS_EXPORT_GUID macros:
                 \code
                BOOST_CLASS_EXPORT(DERIVED_CLASS_1)
                BOOST_CLASS_EXPORT_GUID(DERIVED_CLASS_2, "DERIVED_CLASS_2_ID_STRING")
                 \endcode
     *       See:   http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#derivedpointers
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#export
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/serialization.html#instantiation\
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/special.html#export
     *             http://www.boost.org/doc/libs/release/libs/serialization/doc/traits.html#export
     *       The last two links explain why these export lines have to be in the same source module that includes
     *       any of the archive class headers.
     * */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & /*ar*/, const unsigned int /*version*/) {
    }

  };

//============================== MHValue ===============================
  class MHValue: public Value {

  public:
  
    typedef boost::shared_ptr<Value> sharedImplyGeneric;

    typedef std::list<HypoNode*> HypoList;
    typedef std::list<sharedImplyGeneric> GenericList; //mhsiao: actually ValueList
    
    typedef typename HypoList::iterator HypoListIter;
    typedef typename HypoList::const_iterator HypoListCstIter;
    typedef typename GenericList::iterator GenericListIter;
    typedef typename GenericList::const_iterator GenericListCstIter;

  public:

    GenericList generic_list_;

    HypoLayer* resulting_layer_;

  public:
    
    MHValue() {};
    ~MHValue() {}; 
   
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
    
    virtual size_t hypoNum() const = 0; //mhsiao: added for zeroVector()

    //virtual const HypoList& getHypoList() const = 0;
    //virtual HypoLayer* getHypoLayer() const = 0;
    //virtual const GenericList& getGenericList() const = 0;
    
    virtual const HypoList& getHypoList() const {
      std::cout << "const MHValue::getHypoList() return empty" << std::endl;
      HypoList* empty_list = new HypoList();
      return *empty_list; //should NEVER be called
    }
     
    virtual HypoLayer* getHypoLayer() const {
      std::cout << "const MHValue::getHypoLayer() return NULL" << std::endl;
      return NULL;
    }
    virtual const GenericList& getGenericList() const {
      std::cout << "MHValue::getGenericList() return empty" << std::endl;
      GenericList* empty_list = new GenericList();
      return *empty_list; //should NEVER be called
    }
    
    //[MH-A]: used in mhSolve()
    virtual void mergeHypoAndSetAgree(std::vector<Vector>& vec_arr, const int& max_layer_idx, const Key& key, const double& splitThreshold) {
      std::cout << "MHValue::mergeHypoAndSetAgree() should NEVER be called" << std::endl;
    }

    virtual bool removeAccumulatedPruned() {
      std::cout << "MHValue::removeAccumulatedPruned() should NEVER be called" << std::endl;
      return false;
    }
    
    //[MH-C]:
    virtual void setAgreeWith(const Key& key, HypoLayer* target_layer) {
      std::cout << "MHValue::setAgreeWith() should NEVER be called" << std::endl;
    }
    
    /** Increment the value, by mapping from the vector delta in the tangent
     * space of the current value back to the manifold to produce a new,
     * incremented value.
     * @param delta The delta vector in the tangent space of this value, by
     * which to increment this value.
     */
    virtual Value* retract_(const Vector& delta) const = 0;
    
    virtual Value* retractInPlace_(const Vector& delta) {
      Value* empty_value = NULL;
      std::cout << "Value::retractInPlace_() should NEVER be called" << std::endl;
      return empty_value;
    }
    
    virtual Value* this_() {
      Value* empty_value = NULL;
      std::cout << "Value::this_() should NEVER be called" << std::endl;
      return empty_value;
    }

    /** Compute the coordinates in the tangent space of this value that
     * retract() would map to \c value.
     * @param value The value whose coordinates should be determined in the
     * tangent space of the value on which this function is called.
     * @return The coordinates of \c value in the tangent space of \c this.
     */
    virtual Vector localCoordinates_(const Value& value) const = 0;

    /** Assignment operator */
    virtual Value& operator=(const Value& /*rhs*/) {
      //needs a empty definition so recursion in implicit derived assignment operators work
     return *this;
    }

    /** Cast to known ValueType */
    template<typename ValueType>
    const ValueType& cast() const;
    
  private:
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & /*ar*/, const unsigned int /*version*/) {
    }
  
  };
//============================== MHValue ===============================

} /* namespace gtsam */

BOOST_SERIALIZATION_ASSUME_ABSTRACT(gtsam::Value)


