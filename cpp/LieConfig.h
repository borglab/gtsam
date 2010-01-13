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

#include "Vector.h"
#include "Testable.h"
#include "Lie.h"
#include "Key.h"

namespace boost { template<class T> class optional; }

namespace gtsam {

	class VectorConfig;

	// TODO: why are these defined *before* the class ?
  template<class J, class T> class LieConfig;

  /** Dimensionality of the tangent space */
  template<class J, class T>
  size_t dim(const LieConfig<J,T>& c);

  /** Add a delta config */
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const VectorConfig& delta);

  /** Add a delta vector, uses iterator order */
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const Vector& delta);

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
    typedef std::map<Key, T> Values;
    typedef typename Values::iterator iterator;
    typedef typename Values::const_iterator const_iterator;

  private:

    Values values_;
    size_t dim_;

  public:

    LieConfig() : dim_(0) {}
    LieConfig(const LieConfig& config) :
      values_(config.values_), dim_(dim(config)) {}
    virtual ~LieConfig() {}

    /** print */
    void print(const std::string &s) const;

    /** Test whether configs are identical in keys and values */
    bool equals(const LieConfig& expected, double tol=1e-9) const;

    /** Retrieve a variable by key, throws std::invalid_argument if not found */
    const T& at(const Key& key) const;

    /** operator[] syntax for get */
		inline const T& operator[](const Key& key) const { return at(key);}

	  /** Check if a variable exists */
	  bool exists(const Key& i) const {return values_.find(i)!=values_.end();}

    /** The number of variables in this config */
    int size() const { return values_.size(); }

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }

    // imperative methods:

    /** Add a variable with the given key */
    void insert(const Key& key, const T& val);

    /** Remove a variable from the config */
    void erase(const Key& key) ;

    /** Replace all keys and variables */
    LieConfig& operator=(const LieConfig& rhs) {
      values_ = rhs.values_;
      dim_ = dim(rhs);
      return (*this);
    }

    /** Remove all variables from the config */
    void clear() {
      values_.clear();
      dim_ = 0;
    }

//    friend LieConfig<J,T> expmap<T>(const LieConfig<J,T>& c, const VectorConfig& delta);
//    friend LieConfig<J,T> expmap<T>(const LieConfig<J,T>& c, const Vector& delta);
    friend size_t dim<J,T>(const LieConfig<J,T>& c);

  };

  /** Dimensionality of the tangent space */
  template<class J, class T>
  size_t dim(const LieConfig<J,T>& c) { return c.dim_; }
}

