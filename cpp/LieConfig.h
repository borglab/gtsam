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

namespace boost { template<class T> class optional; }
namespace gtsam { class VectorConfig; }

namespace gtsam {

  template<class T> class LieConfig;

  /** Dimensionality of the tangent space */
  template<class T>
  size_t dim(const LieConfig<T>& c);

  /** Add a delta config */
  template<class T>
  LieConfig<T> expmap(const LieConfig<T>& c, const VectorConfig& delta);

  /** Add a delta vector, uses iterator order */
  template<class T>
  LieConfig<T> expmap(const LieConfig<T>& c, const Vector& delta);


  template<class T>
  class LieConfig : public Testable<LieConfig<T> > {
  private:
    std::map<std::string, T> values_;
    size_t dim_;

  public:
    typedef typename std::map<std::string, T>::const_iterator iterator;
    typedef iterator const_iterator;

    LieConfig() : dim_(0) {}
    LieConfig(const LieConfig& config) :
      values_(config.values_), dim_(dim(config)) {}
    virtual ~LieConfig() {}

    /** Retrieve a variable by key, throws std::invalid_argument if not found */
    const T& get(const std::string& key) const;

    /** operator[] syntax for get */
		inline const T& operator[](const std::string& name) const {
			return get(name);
		}

    /** Retrieve a variable by key, returns nothing if not found */
    boost::optional<const T&> gettry(const std::string& key) const;

    /** Add a variable with the given key */
    void insert(const std::string& name, const T& val);

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

    /** The number of variables in this config */
    int size() { return values_.size(); }

    /** Test whether configs are identical in keys and values */
    bool equals(const LieConfig& expected, double tol=1e-9) const;

    void print(const std::string &s) const;

    const_iterator begin() const { return values_.begin(); }
    const_iterator end() const { return values_.end(); }
    iterator begin() { return values_.begin(); }
    iterator end() { return values_.end(); }

    friend LieConfig<T> expmap<T>(const LieConfig<T>& c, const VectorConfig& delta);
    friend LieConfig<T> expmap<T>(const LieConfig<T>& c, const Vector& delta);
    friend size_t dim<T>(const LieConfig<T>& c);

  };

  /** Dimensionality of the tangent space */
  template<class T>
  size_t dim(const LieConfig<T>& c) { return c.dim_; }
}

