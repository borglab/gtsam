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
 * @brief A templated config for Manifold-group elements
 *
 *  Detailed story:
 *  A values structure is a map from keys to values. It is used to specify the value of a bunch
 *  of variables in a factor graph. A Values is a values structure which can hold variables that
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
  class Values {

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

    Values() {}
    Values(const Values& config) :
      values_(config.values_) {}
    template<class J_ALT>
    Values(const Values<J_ALT>& other) {} // do nothing when initializing with wrong type
    virtual ~Values() {}

    /** print */
    void print(const std::string &s="") const;

    /** Test whether configs are identical in keys and values */
    bool equals(const Values& expected, double tol=1e-9) const;

    /** Retrieve a variable by j, throws std::invalid_argument if not found */
    const Value& at(const J& j) const;

    /** operator[] syntax for get */
    const Value& operator[](const J& j) const { return at(j); }

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

    // Manifold operations

    /** The dimensionality of the tangent space */
    size_t dim() const;

    /** Add a delta config to current config and returns a new config */
    Values retract(const VectorValues& delta, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    VectorValues localCoordinates(const Values& cp, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    void localCoordinates(const Values& cp, const Ordering& ordering, VectorValues& delta) const;

    // imperative methods:

    /** Add a variable with the given j - does not replace existing values */
    void insert(const J& j, const Value& val);

    /** Add a set of variables - does note replace existing values */
    void insert(const Values& cfg);

    /** update the current available values without adding new ones */
    void update(const Values& cfg);

    /** single element change of existing element */
    void update(const J& j, const Value& val);

    /** Remove a variable from the config */
    void erase(const J& j);

    /** Remove a variable from the config while returning the dimensionality of
     * the removed element (normally not needed by user code).
     */
    void erase(const J& j, size_t& dim);

    /**
     * Returns a set of keys in the config
     * Note: by construction, the list is ordered
     */
    std::list<J> keys() const;

    /** Replace all keys and variables */
    Values& operator=(const Values& rhs) {
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
     * iterator for every type in the Values, (i.e. through templating).
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

}

