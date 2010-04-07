/*
 * LieConfig.h
 *
 *  Created on: Jan 5, 2010
 *      Author: Richard Roberts
 *
 *  A templated config for Lie-group elements
 */

#pragma once

#include <map>
#include <set>

#include <boost/serialization/map.hpp>

#include "Vector.h"
#include "Testable.h"
#include "VectorConfig.h"

namespace boost { template<class T> class optional; }
namespace gtsam { class VectorConfig; }

namespace gtsam {

	/**
	 * Lie type configuration
	 */
  template<class J, class T>
  class LieConfig : public Testable<LieConfig<J,T> > {

  public:

    /**
     * Typedefs
     */
    typedef J Key;
    typedef T Value;
    typedef std::map<J, T> Values;
    typedef typename Values::iterator iterator;
    typedef typename Values::const_iterator const_iterator;

  private:

    Values values_;

  public:

    LieConfig() {}
    LieConfig(const LieConfig& config) :
      values_(config.values_) {}
    virtual ~LieConfig() {}

    /** print */
    void print(const std::string &s="") const;

    /** Test whether configs are identical in keys and values */
    bool equals(const LieConfig& expected, double tol=1e-9) const;

    /** Retrieve a variable by j, throws std::invalid_argument if not found */
    const T& at(const J& j) const;

    /** operator[] syntax for get */
		const T& operator[](const J& j) const { return at(j); }

	  /** Check if a variable exists */
	  bool exists(const J& i) const { return values_.find(i)!=values_.end(); }

    /** The number of variables in this config */
    size_t size() const { return values_.size(); }

    /** whether the config is empty */
    bool empty() const { return values_.empty(); }

    /** The dimensionality of the tangent space */
    size_t dim() const;

    /** Get a zero Vectorconfig of the correct structure */
    VectorConfig zero() const;

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }

    // imperative methods:

    /** Add a variable with the given j */
    void insert(const J& j, const T& val);

    /** Add a set of variables */
    void insert(const LieConfig& cfg);

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

  private:
  	/** Serialization function */
  	friend class boost::serialization::access;
  	template<class Archive>
  	void serialize(Archive & ar, const unsigned int version) {
  		ar & BOOST_SERIALIZATION_NVP(values_);
  	}

  };

  /** Dimensionality of the tangent space */
  template<class J, class T>
  inline size_t dim(const LieConfig<J,T>& c) { return c.dim(); }

  /** Add a delta config */
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const VectorConfig& delta);

  /** Add a delta vector, uses iterator order */
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const Vector& delta);

  /** Get a delta config about a linearization point c0 */
  template<class J, class T>
  VectorConfig logmap(const LieConfig<J,T>& c0, const LieConfig<J,T>& cp);

}

