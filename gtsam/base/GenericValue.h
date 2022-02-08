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

#include <gtsam/inference/HypoTree.h>

#include <gtsam/base/Manifold.h>
#include <gtsam/base/Value.h>

#include <boost/make_shared.hpp>
#include <boost/pool/pool_alloc.hpp>

#include <cmath>
#include <iostream>
#include <typeinfo> // operator typeid

namespace gtsam {

class PruningRecord;
class HypoNode;
class HypoLayer;
class HypoTree;


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

  //[MH-A]: called in MHGV::resetThisWith()
  void set(const T& value) {
    value_ = value;
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
    return traits<T>::Equals(this->value_, genericValue2.value_, tol);
  }

  /// non virtual equals function, uses traits
  bool equals(const GenericValue &other, double tol = 1e-9) const {
    return traits<T>::Equals(this->value(), other.value(), tol);
  }

  /// Virtual print function, uses traits
  virtual void print(const std::string& str) const {
    std::cout << "(" << typeid(T).name() << ") ";
    traits<T>::Print(value_, str);
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
      const T retractResult = traits<T>::Retract(GenericValue<T>::value(), delta);

      // Create a Value pointer copy of the result
      void* resultAsValuePlace =
          boost::singleton_pool<PoolTag, sizeof(GenericValue)>::malloc();
      Value* resultAsValue = new (resultAsValuePlace) GenericValue(retractResult);

      // Return the pointer to the Value base class
      return resultAsValue;
    }
    
    //[MH-A]:
    virtual Value* retractInPlace_(const Vector& delta) {
      Value* empty_value = new GenericValue<T>();
      std::cout << "ERROR: GV::retractInPlace_() should NEVER be called" << std::endl;
      return empty_value;
    }

    //[MH-A]:
    virtual Value* this_() {
      Value* empty_value = new GenericValue<T>();
      std::cout << "ERROR: GV::this_() should NEVER be called" << std::endl;
      return empty_value;
    }

    /// Generic Value interface version of localCoordinates
    virtual Vector localCoordinates_(const Value& value2) const {
      // Cast the base class Value pointer to a templated generic class pointer
      const GenericValue<T>& genericValue2 =
          static_cast<const GenericValue<T>&>(value2);

      // Return the result of calling localCoordinates trait on the derived class
      return traits<T>::Local(GenericValue<T>::value(), genericValue2.value());
    }

    /// Non-virtual version of retract
    GenericValue retract(const Vector& delta) const {
      return GenericValue(traits<T>::Retract(GenericValue<T>::value(), delta));
    }

    /// Non-virtual version of localCoordinates
    Vector localCoordinates(const GenericValue& value2) const {
      return localCoordinates_(value2);
    }

    /// Return run-time dimensionality
    virtual size_t dim() const {
      return traits<T>::GetDimension(value_);
    }
   
    //[MH-A]: 
    virtual size_t hypoNum() const {
      return 1; //just to keep consistant with MHGenericValue
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
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("GenericValue",
              boost::serialization::base_object<Value>(*this));
      ar & boost::serialization::make_nvp("value", value_);
    }

/// use this macro instead of BOOST_CLASS_EXPORT for GenericValues
#define GTSAM_VALUE_EXPORT(Type) BOOST_CLASS_EXPORT(gtsam::GenericValue<Type>)

};

// traits
template <typename ValueType>
struct traits<GenericValue<ValueType> >
    : public Testable<GenericValue<ValueType> > {};

// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
const ValueType& Value::cast() const {
  return dynamic_cast<const GenericValue<ValueType>&>(*this).value();
}

//================================================== MHGenericValue =========================================================================
//[MH-A]: not defining template GenericValue<T> in this class
template<class T>
class MHGenericValue: public MHValue {

public:
  
  typedef boost::shared_ptr<Value> sharedImplyGeneric;

  typedef std::list<HypoNode*> HypoList;
  typedef std::list<sharedImplyGeneric> GenericList; //mhsiao: actually ValueList
  typedef typename HypoList::iterator HypoListIter;
  typedef typename HypoList::const_iterator HypoListCstIter;
  typedef typename GenericList::iterator GenericListIter;
  typedef typename GenericList::const_iterator GenericListCstIter;
  typedef std::vector<PruningRecord*> RecordArr;
  
  typedef std::list<T> ValueList;
  typedef typename ValueList::iterator ValueListIter;
  typedef typename ValueList::const_iterator ValueListCstIter;
  
  typedef std::vector<T> ValueArr;
  typedef typename ValueArr::iterator ValueArrIter;
  typedef typename ValueArr::const_iterator ValueArrCstIter;

public:
  // Only needed for serialization.
  MHGenericValue(){}

  MHGenericValue(const GenericList& generic_list, HypoLayer* resulting_layer) {
  
    for (GenericListCstIter it = generic_list.begin(); it != generic_list.end(); ++it) {
      
      generic_list_.push_back((*it));
      
    }
    resulting_layer_ = resulting_layer;
  }

  //[MH-A]: Used as prior in TEST_01. Construct from single value & single hypo (usually used to setup prior)
  //template<class T>
  MHGenericValue(const T& value, HypoLayer* resulting_layer) {
    
    //[MH-A]: create template object
    sharedImplyGeneric gv_ptr( new GenericValue<T>(value) );
    generic_list_.push_back(gv_ptr);

    resulting_layer_ = resulting_layer;
  }
  
  
  //[MH-A]: Used for direct prediction with same T
  template<class pose_T>
  MHGenericValue(const std::vector<pose_T>& ref_arr, const std::vector<T>& new_arr, HypoLayer* resulting_layer) {
  
  typedef std::vector<pose_T> PoseArr;
  typedef typename PoseArr::const_iterator PoseArrCstIter;
  
    for (PoseArrCstIter rit = ref_arr.begin(); rit != ref_arr.end(); ++rit) {
      for (ValueArrCstIter nit = new_arr.begin(); nit != new_arr.end(); ++nit) {
        sharedImplyGeneric gv_ptr( new GenericValue<T>( (*rit)*(*nit)) ); //mhsiao: Pose3 overload "*"

        generic_list_.push_back(gv_ptr);
        
      }
      
    }
   
    resulting_layer_ = resulting_layer;
  
  } // END PREDICT MHGenericValue()
  
  /// Construct from list<value>
  //[MH-A]: Originally used in retract_() (NOT anymore if we use resetThisWith()...)
  MHGenericValue(const std::list<T>& value_list, HypoLayer* resulting_layer) {
    
    for (typename std::list<T>::const_iterator it = value_list.begin(); it != value_list.end(); ++it) {
    
      //[MH-A]: new template object
      sharedImplyGeneric gv_ptr( new GenericValue<T>((*it)) );
      generic_list_.push_back(gv_ptr);
      
    }
    resulting_layer_ = resulting_layer;
  }
  
  MHGenericValue(const std::vector<T>& value_arr, HypoLayer* resulting_layer) {
    
    for (typename std::vector<T>::const_iterator it = value_arr.begin(); it != value_arr.end(); ++it) {
    
      //[MH-A]: new template object
      sharedImplyGeneric gv_ptr( new GenericValue<T>((*it)) );
      generic_list_.push_back(gv_ptr);
    }
    resulting_layer_ = resulting_layer;
  }
  
  //[MH-A]: Used in retract_()... Do NOT create new GV... just update the old ones so that we do not have to update HypoNode::key_value_map_
  void resetThisWith(const std::list<T>& value_list, HypoLayer* resulting_layer) {
  
    GenericListIter git = generic_list_.begin();
    for (typename std::list<T>::const_iterator it = value_list.begin(); it != value_list.end(); ++it, ++git) {
      
      boost::dynamic_pointer_cast< GenericValue<T> >(*git)->set(*it);
    }
    resulting_layer_ = resulting_layer;
  } // END resetThisWith()

  /// Return a constant valueList
  //template<class T>
  const std::list<T> valueList() const {
    std::list<T> value_list;
    for (GenericListCstIter it = generic_list_.begin(); it != generic_list_.end(); ++it) {
      //[MH-A]: cast to template ptr 
      value_list.push_back((boost::dynamic_pointer_cast<GenericValue<T> >(*it))->value());    
    }
    return value_list;
  }
  
  //[MH-A]: Output values of all hypos as a vector
  const std::vector<T> valueArr() const {

    std::vector<T> value_arr(generic_list_.size());
    size_t va_idx = 0;
    for (GenericListCstIter it = generic_list_.begin(); it != generic_list_.end(); ++it) {

      //[MH-A]: cast to template ptr 
      value_arr[va_idx] = (boost::dynamic_pointer_cast<GenericValue<T> >(*it))->value();
      ++va_idx;
    
    }

    return value_arr;
  } // END valueArr()
  

  /// Destructor
  virtual ~MHGenericValue() {
  }

  /// equals implementing generic Value interface
  virtual bool equals_(const Value& p, double tol = 1e-9) const {
    /*
    // Cast the base class Value pointer to a templated generic class pointer
    const MHGenericValue& mh_genericValue2 = static_cast<const MHGenericValue&>(p);
    // Return the result of using the equals traits for the derived class
    GenericListIter it2 = mh_genericValue2.generic_list_.begin();
    for (GenericListIter it = generic_list_.begin(); it != generic_list_.end(); ++it, ++it2) {
      if (!(traits<T>::Equals(it->value_, it2->value_, tol))) {
        return false;
      }
    }
    return true;
    // */
    std::cout << "MHGV::equals_() NOT implemented yet" << std::endl;
    return false;  
  }

  /// non virtual equals function, uses traits
  bool equals(const MHGenericValue &other, double tol = 1e-9) const {
    /*
    GenericListIter it2 = other.generic_list_.begin();
    for (GenericListIter it = generic_list_.begin(); it != generic_list_.end(); ++it, ++it2) {
      if (!(traits<T>::Equals(it->value(), it2->value(), tol))) {
        return false;
      }
    }
    return true;
    // */
    std::cout << "MHGV::equals_() NOT implemented yet" << std::endl;
    return false;  
  }

  /// Virtual print function, uses traits
  virtual void print(const std::string& str) const {
    for (GenericListCstIter it = generic_list_.begin(); it != generic_list_.end(); ++it) {
      (*it)->print(str);
    }
  }

    /**
     * Create a duplicate object returned as a pointer to the generic Value interface.
     * For the sake of performance, this function use singleton pool allocator instead of the normal heap allocator.
     * The result must be deleted with Value::deallocate_, not with the 'delete' operator.
     */
    virtual Value* clone_() const {
      void *place = boost::singleton_pool<PoolTag, sizeof(MHGenericValue)>::malloc();
      MHGenericValue* ptr = new (place) MHGenericValue(*this); // calls copy constructor to fill in
      return ptr;
    }
    
    /// Return run-time dimensionality
    //[MH-A]: return front dim() that represents all dim() 
    virtual size_t dim() const {
      return (generic_list_.front())->dim();
    }
    
    virtual size_t hypoNum() const {
      return getHypoList().size();
    }
   
    HypoList& getHypoList() const {
      return resulting_layer_->getNodeList();
    }
    
    HypoLayer* getHypoLayer() const { 
      return resulting_layer_;
    } 
    
    const GenericList& getGenericList() const {
      return generic_list_;
    }
    

    /**
     * Destroy and deallocate this object, only if it was originally allocated using clone_().
     */
    virtual void deallocate_() const {
      this->~MHGenericValue(); // Virtual destructor cleans up the derived object
      boost::singleton_pool<PoolTag, sizeof(MHGenericValue)>::free((void*) this); // Release memory from pool
    }

    /**
     * Clone this value (normal clone on the heap, delete with 'delete' operator)
     */
    //[MH-A]: used in Values::mhRetract()
    virtual boost::shared_ptr<Value> clone() const {
      return boost::make_shared<MHGenericValue>(*this);
    }

    /// Generic Value interface version of retract
    //[MH-A]: used in Values::mhRetract()
    virtual Value* retract_(const Vector& delta) const {
      //TODO should use this retract_()
      // Call retract on the derived class using the retract trait function

      std::list<T> retractResult_list;
      size_t this_dim = dim();
      int count = 0;

if (generic_list_.size() * this_dim != delta.size()) {
std::cout << "SIZE_DIM: " << generic_list_.size() << " " << this_dim << " " << delta.size() << std::endl;
std::cout << "HYPO: " << resulting_layer_->getNodeList().size() << std::endl;
}
      for (GenericListCstIter it = generic_list_.begin(); it != generic_list_.end(); ++it) {
        retractResult_list.push_back(traits<T>::Retract((boost::dynamic_pointer_cast<GenericValue<T> >(*it))->value(), delta.segment(count, this_dim)));
        count += this_dim;
      }

      // Create a Value pointer copy of the result
      void* resultAsValuePlace =
          boost::singleton_pool<PoolTag, sizeof(MHGenericValue<T>)>::malloc();
      
      Value* resultAsValue = new (resultAsValuePlace) MHGenericValue<T>(retractResult_list, resulting_layer_); 
     
      // Return the pointer to the Value base class
      return resultAsValue;

    } // END MHGV::retract_()
    
    //[MH-A]: 
    virtual Value* retractInPlace_(const Vector& delta) {
      //[MH-A]: should use this retractInPlace_() if we want to keep HypoNode::key_value_map_ unchanged...
      // Call retract on the derived class using the retract trait function
      std::list<T> retractResult_list;
      size_t this_dim = dim();
      int count = 0;
      for (GenericListCstIter it = generic_list_.begin(); it != generic_list_.end(); ++it) {
        
        retractResult_list.push_back(traits<T>::Retract((boost::dynamic_pointer_cast<GenericValue<T> >(*it))->value(), delta.segment(count, this_dim)));
        
        count += this_dim;
      }

      //[MH-A]: 
      resetThisWith(retractResult_list, resulting_layer_);

      return static_cast<Value*>(this);

    } // END MHGV::retractInPlace_()
    
    //[MH-A]:
    virtual Value* this_() {
      return static_cast<Value*>(this);
    }

    /// Generic Value interface version of localCoordinates
    virtual Vector localCoordinates_(const Value& value2) const {
      //TODO should use this localCoordinates_()
      /*
      // Cast the base class Value pointer to a templated generic class pointer
      const MHGenericValue<T>& mh_genericValue2 =
          static_cast<const MHGenericValue<T>&>(value2);
       
      size_t dim = this->dim();
      int len = dim*hypo_list_.size();
      Vector out_vector(len, 1);
      int count = 0;
      GenericListIter it2 = mh_genericValue2.generic_list_.begin();
      for (GenericListIter it = generic_list_.begin(); it != generic_list_.end(); ++it, ++it2) {
        out_vector.segment(count, dim) = it->localCoordinates_((*it2));
        count += dim;
      }

      // Return the result of calling localCoordinates trait on the derived class
      return out_vector;
      // */

      std::cout << "MHGV::localCoordinates_() NOT implemented yet" << std::endl;
      return Vector(1, 1);  
    }

    /*
    //TODO: do we really need this function?
    /// Non-virtual version of retract
    MHGenericValue retract(const Vector& delta) const {
     
      std::list<T> retractResult_list;
      size_t dim = this->dim();
      int count = 0;
      for (GenericListIter it = generic_list_.begin(); it != generic_list_.end(); ++it) {
        
        retractResult_list.push_back(traits<T>::Retract((*it)->value(), delta.segment(count, dim)));
        //retractResult_list.push_back(traits<T>::Retract(it->value(), delta.segment(count, dim)));
        
        count += dim;
      }

      return MHGenericValue(retractResult_list, hypo_list_);
      
      //std::cout << "MHGV::retract() NOT implemented yet and should NEVER be called" << std::endl;
      
      //return MHGenericValue();
    }
    // */

    /// Non-virtual version of localCoordinates
    Vector localCoordinates(const MHGenericValue& value2) const {
      return localCoordinates_(value2);
    }
    
    //[MH-A]: generate num_arr for next step
    inline bool isVecMergeable(const std::vector<Vector>& vec_arr, std::vector<int>& num_arr, const int& layer_diff, const double& splitThreshold) {

      int idx = 0;
      //for (HypoListIter it = hypo_list_.begin(); it != hypo_list_.end(); ++it) { //[D]
      HypoList& hypo_list = getHypoList();
      for (HypoListIter it = hypo_list.begin(); it != hypo_list.end(); ++it) {

        //[MH-S]:
        std::list<size_t> tmp_list;
        (*it)->findSegDescendantNum(tmp_list, layer_diff, 1);
        
        //[MH-S]:
        for (auto h_seg = tmp_list.begin(); h_seg != tmp_list.end(); ++h_seg) {
          
          //[MH-S]:
          for (size_t k = 1; k < (*h_seg); ++k) { //compairison so start from 1...
          
            if ((vec_arr[idx] - vec_arr[idx + k]).lpNorm<Eigen::Infinity>() >= splitThreshold) { //CANNOT be merged
              return false;
            }
          }
          
          //[MH-S]:
          idx += (*h_seg);
        
        }
        
        num_arr.insert(num_arr.end(), tmp_list.begin(), tmp_list.end());
      }

      return true;
    } // END isVecMergeable()

    //[MH-A]: replace vec_arr with the nerged one...
    inline void mergeVec(std::vector<Vector>& vec_arr, const std::vector<int>& num_arr) {
      std::vector<Vector> merged_arr(num_arr.size());
      size_t idx = 0;
      for (size_t i = 0; i < num_arr.size(); ++i) {
        // Just take the first, not the averaged...
        //TODO: could be improved...
        merged_arr[i] = vec_arr[idx];
        idx += num_arr[i];
      }
      vec_arr = merged_arr; //replace
    }
    
    //[MH-A]: A lot of work!!!! Try merging. If not complete then expand MHGV
    void mergeHypoAndSetAgree(std::vector<Vector>& vec_arr, const int& max_layer_idx, const Key& key, const double& splitThreshold) { 

      const int this_layer_idx = resulting_layer_->getLayerIdx();
      const int layer_diff = max_layer_idx - this_layer_idx;

      //[MH-A]: combine vec_arr based on HypoTree structure
      int remained_layer = 0; 
      for (int i = 0; i < layer_diff; ++i) {

        std::vector<int> num_arr;
        if (isVecMergeable(vec_arr, num_arr, (layer_diff - i), splitThreshold)) {

          // Merge the vec_arr
          mergeVec(vec_arr, num_arr);

        } else {

          // Stop here and have to duplicate this MHGV
          remained_layer = layer_diff - i;
          break;
        }
      }

      if (remained_layer != 0) { //have to expand MHGV

        //[MH-A]: duplicate generic_list_ items if the combined vec_arr still CANNOT reach the same HypoLayer
        setAgreeToDiff(key, remained_layer);

      } // END if NOT the same HypoLayer...

//*
if (generic_list_.size() != vec_arr.size()) {
std::cout << "GL_SIZE: " << generic_list_.size() << " " << vec_arr.size() << std::endl;
std::cout << "LL: " << this_layer_idx << " " << max_layer_idx << std::endl;
}
// */      
    } // END mergeHypoAndSetAgree()
  
    //[MH-C]:
    void eraseGenericListAt(const std::vector<size_t>& idx_arr) {
      size_t count_idx = 0;
      size_t count_gv = 0;
      for (GenericListIter git = generic_list_.begin(); git != generic_list_.end(); ++git) {
        if (count_gv == idx_arr[count_idx]) {
          git = generic_list_.erase(git);
          --git;
          ++count_idx;
          if (count_idx == idx_arr.size()) {
            break;
          }
        }
        ++count_gv;
      }
    } // END eraseGenericListAt()
   
    //[MH-C]:
    bool removeAccumulatedPruned() {
      HypoList& hypo_list = resulting_layer_->getNodeList();
      if (generic_list_.size() == hypo_list.size()) {
        return false; //already up-to-date, no need any changes
      } else {
        RecordArr& record_arr = resulting_layer_->getRecordArr();
        
        for (size_t r = 0; r < record_arr.size(); ++r) {
        
          if (record_arr[r]->original_size_ == generic_list_.size()) {
            for (size_t t = r; t < record_arr.size(); ++t) {
              eraseGenericListAt(record_arr[t]->pruned_idx_arr_);
            }
            return true;
          }
        }
        std::cout << "ERROR: MHGV::removeAccumulatedPruned() should match exactly one original_size_ (generic_list_.size(): " << generic_list_.size() << ")" << std::endl;

        std::cout << "All original_size_: ";
        std::cout << "hypo_list.size(): " << hypo_list.size() << std::endl;

        for (size_t r = 0; r < record_arr.size(); ++r) {
            std::cout << record_arr[r]->original_size_ << " ";
        }
        std::cout << std::endl;

        return false;
      }
    } // END removeAccumulatedPruned()
    
    void setAgreeToDiff(const Key& key, const int& layer_diff) {
      GenericListIter git = generic_list_.begin();  
      HypoList& hypo_list = getHypoList();

      for (HypoListIter it = hypo_list.begin(); it != hypo_list.end(); ++it) {
        
        size_t num = (*it)->findDescendantNum(layer_diff); 
        
        T& copy_value = (boost::dynamic_pointer_cast<GenericValue<T> >(*git))->value();
        for (size_t i = 0; i < (num - 1); ++i) {

          sharedImplyGeneric gv_ptr( new GenericValue<T>(copy_value) );
          generic_list_.insert(git, gv_ptr);

        }
        ++git;

        (*it)->removeValueLink(key);
      }
        
      resulting_layer_->removeKeyMap(key);


      resulting_layer_ = &(resulting_layer_->toDiffLayer(layer_diff));
      resulting_layer_->setKeyMap(key);

      GenericListIter new_git = generic_list_.begin();
        
      HypoList& new_list = resulting_layer_->getNodeList(); //[D]
      for (HypoListIter it = new_list.begin(); it != new_list.end(); ++it) {
        (*it)->addKeyValuePair(key, (*new_git));
        ++new_git;
      }
    } // END setAgreeToDiff()
    
    void setAgreeWith(const Key& key, HypoLayer* target_layer) {
      setAgreeToDiff( key, (target_layer->getLayerIdx() - resulting_layer_->getLayerIdx()) );
    }
     
    /*
    /// Assignment operator
    virtual Value& operator=(const Value& rhs) {
      // Cast the base class Value pointer to a derived class pointer
      const GenericValue& derivedRhs = static_cast<const GenericValue&>(rhs);

      // Do the assignment and return the result
      *this = GenericValue(derivedRhs); // calls copy constructor
      return *this;
    }
    // */

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
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & boost::serialization::make_nvp("MHGenericValue",
              boost::serialization::base_object<Value>(*this));
      ar & boost::serialization::make_nvp("generic_list", generic_list_);
    }

/// use this macro instead of BOOST_CLASS_EXPORT for MHGenericValues
#define GTSAM_MHVALUE_EXPORT(Type) BOOST_CLASS_EXPORT(gtsam::MHGenericValue<Type>)

};

/*
// traits
struct traits<MHGenericValue>
    : public Testable<MHGenericValue> {};
// */
/*
//TODO can be problematic...
// define Value::cast here since now GenericValue has been declared
template<typename ValueType>
const ValueType& Value::cast() const {
  return dynamic_cast<const MHGenericValue<ValueType>&>(*this).value();
}
// */
//================================================== END MHGenericValue =========================================================================

} /* namespace gtsam */
