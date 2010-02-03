/*
 * TupleConfig.h
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts and Manohar Paluri
 */

#include "LieConfig.h"
#include "VectorConfig.h"

#pragma once

namespace gtsam {

  /**
   * PairConfig:  a config that holds two data types.
   */
  template<class J1, class X1, class J2, class X2>
  class PairConfig : public Testable<PairConfig<J1, X1, J2, X2> > {

  public:

  	// publicly available types
    typedef LieConfig<J1, X1> Config1;
    typedef LieConfig<J2, X2> Config2;

    // Two configs in the pair as in std:pair
    LieConfig<J1, X1> first;
    LieConfig<J2, X2> second;

  private:

    size_t size_;
    size_t dim_;

    PairConfig(const LieConfig<J1,X1>& config1, const LieConfig<J2,X2>& config2) :
      first(config1), second(config2),
      size_(config1.size()+config2.size()), dim_(gtsam::dim(config1)+gtsam::dim(config2)) {}

  public:

    /**
     * Default constructor creates an empty config.
     */
    PairConfig(): size_(0), dim_(0) {}

    /**
     * Copy constructor
     */
    PairConfig(const PairConfig<J1, X1, J2, X2>& c):
      first(c.first), second(c.second), size_(c.size_), dim_(c.dim_) {}

    /**
     * Print
     */
    void print(const std::string& s = "") const;

    /**
     * Test for equality in keys and values
     */
    bool equals(const PairConfig<J1, X1, J2, X2>& c, double tol=1e-9) const {
      return first.equals(c.first, tol) && second.equals(c.second, tol); }

    /**
     * operator[] syntax to get a value by j, throws invalid_argument if
     * value with specified j is not present.  Will generate compile-time
     * errors if j type does not match that on which the Config is templated.
     */
    const X1& operator[](const J1& j) const { return first[j]; }
    const X2& operator[](const J2& j) const { return second[j]; }

    /**
     * size is the total number of variables in this config.
     */
    size_t size() const { return size_; }

    /**
     * dim is the dimensionality of the tangent space
     */
    size_t dim() const { return dim_; }

  private:
    template<class Config, class Key, class Value>
    void insert_helper(Config& config, const Key& j, const Value& value) {
      config.insert(j, value);
      size_ ++;
      dim_ += gtsam::dim(value);
    }

    template<class Config, class Key>
    void erase_helper(Config& config, const Key& j) {
      size_t dim;
      config.erase(j, dim);
      dim_ -= dim;
      size_ --;
    }

  public:
    /**
     * expmap each element
     */
    PairConfig<J1,X1,J2,X2> expmap(const VectorConfig& delta) const {
      return PairConfig(gtsam::expmap(first, delta), gtsam::expmap(second, delta)); }

    /**
     * logmap each element
     */
    VectorConfig logmap(const PairConfig<J1,X1,J2,X2>& cp) const {
    	VectorConfig ret(gtsam::logmap(first, cp.first));
    	ret.insert(gtsam::logmap(second, cp.second));
    	return ret;
    }

    /**
     * Insert a variable with the given j
     */
    void insert(const J1& j, const X1& value) { insert_helper(first, j, value); }
    void insert(const J2& j, const X2& value) { insert_helper(second, j, value); }

    void insert(const PairConfig& config);

    /**
     * Remove the variable with the given j.  Throws invalid_argument if the
     * j is not present in the config.
     */
    void erase(const J1& j) { erase_helper(first, j); }
    void erase(const J2& j) { erase_helper(second, j); }

    /**
     * Check if a variable exists
     */
    bool exists(const J1& j) const { return first.exists(j); }
    bool exists(const J2& j) const { return second.exists(j); }


  };

  template<class J1, class X1, class J2, class X2>
  inline PairConfig<J1,X1,J2,X2> expmap(const PairConfig<J1,X1,J2,X2> c, const VectorConfig& delta) { return c.expmap(delta); }

  template<class J1, class X1, class J2, class X2>
  inline VectorConfig logmap(const PairConfig<J1,X1,J2,X2> c0, const PairConfig<J1,X1,J2,X2>& cp) { return c0.logmap(cp); }


  /**
   *  Tuple configs to handle subconfigs of LieConfigs
   *
   *  This uses a recursive structure of config pairs to form a lisp-like
   *  list, with a special case (TupleConfigEnd) that contains only one config
   *  at the end.  In a final use case, this should be aliased to something clearer
   *  but still with the same recursive type machinery.
   *
   *  STILL UNDER TESTING - DO NOT USE
   */
  template<class Config1, class Config2>
  class TupleConfig : public Testable<TupleConfig<Config1, Config2> > {

  protected:
	  // Data for internal configs
	  Config1 first_;
	  Config2 second_;

  public:
	  // typedefs
	  typedef class Config1::Key Key1;
	  typedef class Config1::Value Value1;

  public:
	  TupleConfig() {}
	  virtual ~TupleConfig() {}

	  /** Print */
	  void print(const std::string& s = "") const {}

	  /** Test for equality in keys and values */
	  bool equals(const TupleConfig<Config1, Config2>& c, double tol=1e-9) const {
		  return false;
	  }

	  // insert function that uses the second (recursive) config
	  template<class Key, class Value>
	  void insert(const Key& key, const Value& value) {second_.insert(key, value);}
	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value);}

	  // erase an element by key
	  template<class Key>
	  void erase(const Key& j)  {  }
	  void erase(const Key1& j)  {  }

	  // determine whether an element exists
	  template<class Key>
	  bool exists(const Key& j) const { return second_.exists(j); }
	  bool exists(const Key1& j) const { return first_.exists(j); }

	  // access operator - currently configs after the first one will not be found
	  template<class Key, class Value>
	  const Value& operator[](const Key& j) const { return second_[j]; }
	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  template<class Key, class Value>
	  const Value& at(const Key& j) const { return second_.at(j); }
	  const Value1& at(const Key1& j) const { return first_.at(j); }

  };

  template<class Config>
  class TupleConfigEnd : public Testable<TupleConfigEnd<Config> > {
  protected:
	  // Data for internal configs
	  Config first_;

  public:
	  // typedefs
	  typedef class Config::Key Key1;
	  typedef class Config::Value Value1;

  public:
	  TupleConfigEnd() {}
	  virtual ~TupleConfigEnd() {}

	  /** Print */
	  void print(const std::string& s = "") const {}

	  /** Test for equality in keys and values */
	  bool equals(const TupleConfigEnd<Config>& c, double tol=1e-9) const {
		  return false;
	  }

	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value); }

	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  void erase(const Key1& j) {  }

	  bool exists(const Key1& j) const { return first_.exists(j); }

	  const Value1& at(const Key1& j) const { return first_.at(j); }

  };
}
