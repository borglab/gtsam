/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ValuesOld.h
 * @author Richard Roberts
 *
 * @brief A templated config for Manifold-group elements
 *
 *  Detailed story:
 *  A values structure is a map from keys to values. It is used to specify the value of a bunch
 *  of variables in a factor graph. A ValuesOld is a values structure which can hold variables that
 *  are elements on manifolds, not just vectors. It then, as a whole, implements a aggregate type
 *  which is also a manifold element, and hence supports operations dim, retract, and localCoordinates.
 */

#pragma once

#include <set>

#include <gtsam/base/Testable.h>
#include <gtsam/base/FastMap.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>
#include <gtsam/nonlinear/Ordering.h>

namespace boost { template<class T> class optional; }
namespace gtsam { class VectorValues; class Ordering; }

namespace gtsam {

  // Forward declarations
  template<class J> class KeyDoesNotExist;
  template<class J> class KeyAlreadyExists;

	/**
	 * Manifold type values structure
	 * Takes two template types
	 *  J: a key type to look up values in the values structure (need to be sortable)
	 *
	 * Key concept:
	 *  The key will be assumed to be sortable, and must have a
	 *  typedef called "Value" with the type of the value the key
	 *  labels (example: Pose2, Point2, etc)
	 */
  template<class J>
  class ValuesOld {

  public:

    /**
     * Typedefs
     */
  	typedef J Key;
  	typedef typename J::Value Value;
  	typedef std::map<Key, Value, std::less<Key>, boost::fast_pool_allocator<std::pair<const Key, Value> > > KeyValueMap;
	//    typedef FastMap<J,Value> KeyValueMap;
    typedef typename KeyValueMap::value_type KeyValuePair;
    typedef typename KeyValueMap::iterator iterator;
    typedef typename KeyValueMap::const_iterator const_iterator;

  private:

    /** concept check */
    GTSAM_CONCEPT_TESTABLE_TYPE(Value)
    GTSAM_CONCEPT_MANIFOLD_TYPE(Value)

    KeyValueMap values_;

  public:

    ValuesOld() {}
    ValuesOld(const ValuesOld& config) :
      values_(config.values_) {}
    template<class J_ALT>
    ValuesOld(const ValuesOld<J_ALT>& other) {} // do nothing when initializing with wrong type
    virtual ~ValuesOld() {}

    /// @name Testable
    /// @{

    /** print */
    void print(const std::string &s="") const;

    /** Test whether configs are identical in keys and values */
    bool equals(const ValuesOld& expected, double tol=1e-9) const;

    /// @}

    /** Retrieve a variable by j, throws KeyDoesNotExist<J> if not found */
    const Value& at(const J& j) const;

    /** operator[] syntax for get, throws KeyDoesNotExist<J> if not found */
    const Value& operator[](const J& j) const {
      return at(j); }

    /** Check if a variable exists */
    bool exists(const J& i) const { return values_.find(i)!=values_.end(); }

    /** Check if a variable exists and return it if so */
    boost::optional<Value> exists_(const J& i) const {
    	const_iterator it = values_.find(i);
    	if (it==values_.end()) return boost::none; else	return it->second;
    }

    /** The number of variables in this config */
    size_t size() const { return values_.size(); }

    /** whether the config is empty */
    bool empty() const { return values_.empty(); }

    /** Get a zero VectorValues of the correct structure */
    VectorValues zero(const Ordering& ordering) const;

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }

    /// @name Manifold Operations
    /// @{

    /** The dimensionality of the tangent space */
    size_t dim() const;

    /** Add a delta config to current config and returns a new config */
    ValuesOld retract(const VectorValues& delta, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    VectorValues localCoordinates(const ValuesOld& cp, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    void localCoordinates(const ValuesOld& cp, const Ordering& ordering, VectorValues& delta) const;

    /// @}

    // imperative methods:

    /** Add a variable with the given j, throws KeyAlreadyExists<J> if j is already present */
    void insert(const J& j, const Value& val);

    /** Add a set of variables, throws KeyAlreadyExists<J> if a key is already present */
    void insert(const ValuesOld& cfg);

    /** update the current available values without adding new ones */
    void update(const ValuesOld& cfg);

    /** single element change of existing element */
    void update(const J& j, const Value& val);

    /** Remove a variable from the config, throws KeyDoesNotExist<J> if j is not present */
    void erase(const J& j);

    /** Remove a variable from the config while returning the dimensionality of
     * the removed element (normally not needed by user code), throws
     * KeyDoesNotExist<J> if j is not present.
     */
    void erase(const J& j, size_t& dim);

    /**
     * Returns a set of keys in the config
     * Note: by construction, the list is ordered
     */
    std::list<J> keys() const;

    /** Replace all keys and variables */
    ValuesOld& operator=(const ValuesOld& rhs) {
      values_ = rhs.values_;
      return (*this);
    }

    /** Remove all variables from the config */
    void clear() {
      values_.clear();
    }

    /**
     * Apply a class with an application operator() to a const_iterator over
     * every <key,value> pair.  The operator must be able to handle such an
     * iterator for every type in the ValuesOld, (i.e. through templating).
     */
    template<typename A>
    void apply(A& operation) {
      for(iterator it = begin(); it != end(); ++it)
        operation(it);
    }
    template<typename A>
    void apply(A& operation) const {
      for(const_iterator it = begin(); it != end(); ++it)
        operation(it);
    }

    /** Create an array of variable dimensions using the given ordering */
    std::vector<size_t> dims(const Ordering& ordering) const;

    /**
     * Generate a default ordering, simply in key sort order.  To instead
     * create a fill-reducing ordering, use
     * NonlinearFactorGraph::orderingCOLAMD().  Alternatively, you may permute
     * this ordering yourself (as orderingCOLAMD() does internally).
     */
    Ordering::shared_ptr orderingArbitrary(Index firstVar = 0) const;

  private:
  	/** Serialization function */
  	friend class boost::serialization::access;
  	template<class ARCHIVE>
  	void serialize(ARCHIVE & ar, const unsigned int version) {
  		ar & BOOST_SERIALIZATION_NVP(values_);
  	}

  };

  struct _ValuesDimensionCollector {
    const Ordering& ordering;
    std::vector<size_t> dimensions;
    _ValuesDimensionCollector(const Ordering& _ordering) : ordering(_ordering), dimensions(_ordering.nVars()) {}
    template<typename I> void operator()(const I& key_value) {
      Index var;
      if(ordering.tryAt(key_value->first, var)) {
        assert(var < dimensions.size());
        dimensions[var] = key_value->second.dim();
      }
    }
  };

  /* ************************************************************************* */
  struct _ValuesKeyOrderer {
    Index var;
    Ordering::shared_ptr ordering;
    _ValuesKeyOrderer(Index firstVar) : var(firstVar), ordering(new Ordering) {}
    template<typename I> void operator()(const I& key_value) {
      ordering->insert(key_value->first, var);
      ++ var;
    }
  };


/* ************************************************************************* */
template<class J>
class KeyAlreadyExists : public std::exception {
protected:
  const J key_; ///< The key that already existed
  const typename J::Value value_; ///< The value attempted to be inserted

private:
  mutable std::string message_;

public:
  /// Construct with the key-value pair attemped to be added
  KeyAlreadyExists(const J& key, const typename J::Value& value) throw() :
    key_(key), value_(value) {}

  virtual ~KeyAlreadyExists() throw() {}

  /// The duplicate key that was attemped to be added
  const J& key() const throw() { return key_; }

  /// The value that was attempted to be added
  const typename J::Value& value() const throw() { return value_; }

  /// The message to be displayed to the user
  virtual const char* what() const throw();
};

/* ************************************************************************* */
template<class J>
class KeyDoesNotExist : public std::exception {
protected:
  const char* operation_; ///< The operation that attempted to access the key
  const J key_; ///< The key that does not exist

private:
  mutable std::string message_;

public:
  /// Construct with the key that does not exist in the values
  KeyDoesNotExist(const char* operation, const J& key) throw() :
    operation_(operation), key_(key) {}

  virtual ~KeyDoesNotExist() throw() {}

  /// The key that was attempted to be accessed that does not exist
  const J& key() const throw() { return key_; }

  /// The message to be displayed to the user
  virtual const char* what() const throw();
};

} // \namespace gtsam

#include <gtsam/nonlinear/ValuesOld-inl.h>

