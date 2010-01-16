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
    size_t dim_;

  public:

    LieConfig() : dim_(0) {}
    LieConfig(const LieConfig& config) :
      values_(config.values_), dim_(config.dim_) {}
    virtual ~LieConfig() {}

    /** print */
    void print(const std::string &s) const;

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

    /**
     * The dimensionality of the tangent space
     */
    size_t dim() const { return dim_; }

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }

    // imperative methods:

    /** Add a variable with the given j */
    void insert(const J& j, const T& val);

    /** Remove a variable from the config */
    void erase(const J& j);

    /** Remove a variable from the config while returning the dimensionality of
     * the removed element (normally not needed by user code).
     */
    void erase(const J& j, size_t& dim);

    /** Replace all keys and variables */
    LieConfig& operator=(const LieConfig& rhs) {
      values_ = rhs.values_;
      dim_ = rhs.dim_;
      return (*this);
    }

    /** Remove all variables from the config */
    void clear() {
      values_.clear();
      dim_ = 0;
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
}

