/**
 * @file LieConfig.h
 * @Author: Richard Roberts
 *
 * @brief A templated config for Lie-group elements
 *
 *  Detailed story:
 *  A configuration is a map from keys to values. It is used to specify the value of a bunch
 *  of variables in a factor graph. A LieConfig is a configuration which can hold variables that
 *  are elements of Lie groups, not just vectors. It then, as a whole, implements a aggregate type
 *  which is also a Lie group, and hence supports operations dim, expmap, and logmap.
 */

#pragma once

#include <map>
#include <set>

#include <boost/pool/pool_alloc.hpp>

//#include <boost/serialization/map.hpp>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Ordering.h>

namespace boost { template<class T> class optional; }
namespace gtsam { class VectorConfig; class Ordering; }

namespace gtsam {

	/**
	 * Lie type configuration
	 * Takes two template types
	 *  J: a key type to look up values in the configuration (need to be sortable)
	 *
	 * Key concept:
	 *  The key will be assumed to be sortable, and must have a
	 *  typedef called "Value_t" with the type of the value the key
	 *  labels (example: Pose2, Point2, etc)
	 */
  template<class J>
  class LieConfig : public Testable<LieConfig<J> > {

  public:

    /**
     * Typedefs
     */
  	typedef J Key;
  	typedef typename J::Value_t Value;
    typedef std::map<J,Value, std::less<J>, boost::fast_pool_allocator<std::pair<const J,Value> > > KeyValueMap;
    typedef typename KeyValueMap::value_type KeyValuePair;
    typedef typename KeyValueMap::iterator iterator;
    typedef typename KeyValueMap::const_iterator const_iterator;

  private:

    KeyValueMap values_;

  public:

    LieConfig() {}
    LieConfig(const LieConfig& config) :
      values_(config.values_) {}
    template<class J_alt>
    LieConfig(const LieConfig<J_alt>& other) {} // do nothing when initializing with wrong type
    virtual ~LieConfig() {}

    /** print */
    void print(const std::string &s="") const;

    /** Test whether configs are identical in keys and values */
    bool equals(const LieConfig& expected, double tol=1e-9) const;

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

    /** The dimensionality of the tangent space */
    size_t dim() const;

    /** Get a zero VectorConfig of the correct structure */
    VectorConfig zero(const Ordering& ordering) const;

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }

    // Lie operations

    /** Add a delta config to current config and returns a new config */
    LieConfig expmap(const VectorConfig& delta, const Ordering& ordering) const;

//    /** Add a delta vector to current config and returns a new config, uses iterator order */
//    LieConfig expmap(const Vector& delta) const;

    /** Get a delta config about a linearization point c0 (*this) */
    VectorConfig logmap(const LieConfig& cp, const Ordering& ordering) const;

    /** Get a delta config about a linearization point c0 (*this) */
    void logmap(const LieConfig& cp, const Ordering& ordering, VectorConfig& delta) const;

    // imperative methods:

    /** Add a variable with the given j - does not replace existing values */
    void insert(const J& j, const Value& val);

    /** Add a set of variables - does note replace existing values */
    void insert(const LieConfig& cfg);

    /** update the current available values without adding new ones */
    void update(const LieConfig& cfg);

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
    LieConfig& operator=(const LieConfig& rhs) {
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
     * iterator for every type in the Config, (i.e. through templating).
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
    Ordering::shared_ptr orderingArbitrary(varid_t firstVar = 0) const;

  private:
  	/** Serialization function */
  	friend class boost::serialization::access;
  	template<class Archive>
  	void serialize(Archive & ar, const unsigned int version) {
  		ar & BOOST_SERIALIZATION_NVP(values_);
  	}

  };

  struct _ConfigDimensionCollector {
    const Ordering& ordering;
    std::vector<size_t> dimensions;
    _ConfigDimensionCollector(const Ordering& _ordering) : ordering(_ordering), dimensions(_ordering.nVars()) {}
    template<typename I> void operator()(const I& key_value) {
      varid_t var = ordering[key_value->first];
      assert(var < dimensions.size());
      dimensions[var] = key_value->second.dim();
    }
  };

  /* ************************************************************************* */
  struct _ConfigKeyOrderer {
    varid_t var;
    Ordering::shared_ptr ordering;
    _ConfigKeyOrderer(varid_t firstVar) : var(firstVar), ordering(new Ordering) {}
    template<typename I> void operator()(const I& key_value) {
      ordering->insert(key_value->first, var);
      ++ var;
    }
  };

}

