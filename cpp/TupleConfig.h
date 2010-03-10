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
   *  Tuple configs to handle subconfigs of LieConfigs
   *
   *  This uses a recursive structure of config pairs to form a lisp-like
   *  list, with a special case (TupleConfigEnd) that contains only one config
   *  at the end.  In a final use case, this should be aliased to something clearer
   *  but still with the same recursive type machinery.
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

	  /** default constructor */
	  TupleConfig() {}

	  /** Copy constructor */
	  TupleConfig(const TupleConfig<Config1, Config2>& config) :
		  first_(config.first_), second_(config.second_) {}

	  /** Construct from configs */
	  TupleConfig(const Config1& cfg1, const Config2& cfg2) :
		  first_(cfg1), second_(cfg2) {}

	  virtual ~TupleConfig() {}

	  /** Print */
	  void print(const std::string& s = "") const;

	  /** Test for equality in keys and values */
	  bool equals(const TupleConfig<Config1, Config2>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol) && second_.equals(c.second_, tol);
	  }

	  // insert function that uses the second (recursive) config
	  template<class Key, class Value>
	  void insert(const Key& key, const Value& value) {second_.insert(key, value);}
	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value);}

	  // insert function for whole configs
	  template<class Cfg1, class Cfg2>
	  void insert(const TupleConfig<Cfg1, Cfg2>& config) { second_.insert(config); }
	  void insert(const TupleConfig<Config1, Config2>& config) {
		  first_.insert(config.first_);
		  second_.insert(config.second_);
	  }

	  // erase an element by key
	  template<class Key>
	  void erase(const Key& j)  { second_.erase(j); }
	  void erase(const Key1& j)  { first_.erase(j); }

	  // determine whether an element exists
	  template<class Key>
	  bool exists(const Key& j) const { return second_.exists(j); }
	  bool exists(const Key1& j) const { return first_.exists(j); }

	  // access operator
	  template<class Key>
	  const typename Key::Value_t & operator[](const Key& j) const { return second_[j]; }
	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  // at access function
	  template<class Key>
	  const typename Key::Value_t & at(const Key& j) const { return second_.at(j); }
	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  // direct config access
	  const Config1& config() const { return first_; }
	  const Config2& rest() const { return second_; }

	  // size function - adds recursively
	  size_t size() const { return first_.size() + second_.size(); }

	  // dim function
	  size_t dim() const { return first_.dim() + second_.dim(); }

	  // Expmap
	  TupleConfig<Config1, Config2> expmap(const VectorConfig& delta) const {
	        return TupleConfig(gtsam::expmap(first_, delta), second_.expmap(delta));
	  }

	  /** logmap each element */
	  VectorConfig logmap(const TupleConfig<Config1, Config2>& cp) const {
		  VectorConfig ret(gtsam::logmap(first_, cp.first_));
		  ret.insert(second_.logmap(cp.second_));
		  return ret;
	  }

  };

  /**
   * End of a recursive TupleConfig - contains only one config
   *
   * This should not be used directly
   */
  template<class Config>
  class TupleConfigEnd : public Testable<TupleConfigEnd<Config> > {

  protected:
	  // Data for internal configs
	  Config first_;

  public:
	  // typedefs
	  typedef class Config::Key Key1;
	  typedef class Config::Value Value1;

	  TupleConfigEnd() {}

	  TupleConfigEnd(const TupleConfigEnd<Config>& config) :
		  first_(config.first_) {}

	  TupleConfigEnd(const Config& cfg) :
		  first_(cfg) {}

	  virtual ~TupleConfigEnd() {}

	  /** Print */
	  void print(const std::string& s = "") const;

	  /** Test for equality in keys and values */
	  bool equals(const TupleConfigEnd<Config>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol);
	  }

	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value); }

	  // insert function for whole configs
	  void insert(const TupleConfigEnd<Config>& config) {first_.insert(config.first_); }

	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  const Config& config() const { return first_; }

	  void erase(const Key1& j) { first_.erase(j); }

	  bool exists(const Key1& j) const { return first_.exists(j); }

	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  size_t size() const { return first_.size(); }

	  size_t dim() const { return first_.dim(); }

	  TupleConfigEnd<Config> expmap(const VectorConfig& delta) const {
	        return TupleConfigEnd(gtsam::expmap(first_, delta));
	  }

	  VectorConfig logmap(const TupleConfigEnd<Config>& cp) const {
		  VectorConfig ret(gtsam::logmap(first_, cp.first_));
		  return ret;
	  }
  };

  /** Exmap static functions */
  template<class Config1, class Config2>
  inline TupleConfig<Config1, Config2> expmap(const TupleConfig<Config1, Config2>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  /** logmap static functions */
  template<class Config1, class Config2>
    inline VectorConfig logmap(const TupleConfig<Config1, Config2>& c0, const TupleConfig<Config1, Config2>& cp) {
	  return c0.logmap(cp);
  }

  /**
   * Wrapper classes to act as containers for configs.  Note that these can be cascaded
   * recursively, as they are TupleConfigs, and are primarily a short form of the config
   * structure to make use of the TupleConfigs easier.
   *
   * The interface is designed to mimic PairConfig, but for 2-6 config types.
   */
  template<class Config1, class Config2>
  class TupleConfig2 : public TupleConfig<Config1, TupleConfigEnd<Config2> > {
  public:
	  // typedefs
	  typedef Config1 Config1_t;
	  typedef Config2 Config2_t;

	  typedef TupleConfig<Config1, TupleConfigEnd<Config2> > Base;
	  typedef TupleConfig2<Config1, Config2> This;

	  TupleConfig2() {}
	  TupleConfig2(const This& config);
	  TupleConfig2(const Base& config);
	  TupleConfig2(const Config1& cfg1, const Config2& cfg2);

	  // access functions
	  inline const Config1_t& first() const { return this->config(); }
	  inline const Config2_t& second() const { return this->rest().config(); }
  };

  template<class Config1, class Config2>
  TupleConfig2<Config1, Config2> expmap(const TupleConfig2<Config1, Config2>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class Config1, class Config2, class Config3>
  class TupleConfig3 : public TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > > {
  public:
	  // typedefs
	  typedef Config1 Config1_t;
	  typedef Config2 Config2_t;
	  typedef Config3 Config3_t;

	  TupleConfig3() {}
	  TupleConfig3(const TupleConfig<Config1, TupleConfig<Config2, TupleConfigEnd<Config3> > >& config);
	  TupleConfig3(const TupleConfig3<Config1, Config2, Config3>& config);
	  TupleConfig3(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3);

	  // access functions
	  inline const Config1_t& first() const { return this->config(); }
	  inline const Config2_t& second() const { return this->rest().config(); }
	  inline const Config3_t& third() const { return this->rest().rest().config(); }
  };

  template<class Config1, class Config2, class Config3>
  TupleConfig3<Config1, Config2, Config3> expmap(const TupleConfig3<Config1, Config2, Config3>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class Config1, class Config2, class Config3, class Config4>
  class TupleConfig4 : public TupleConfig<Config1, TupleConfig<Config2,TupleConfig<Config3, TupleConfigEnd<Config4> > > > {
  public:
	  // typedefs
	  typedef Config1 Config1_t;
	  typedef Config2 Config2_t;
	  typedef Config3 Config3_t;
	  typedef Config4 Config4_t;

	  typedef TupleConfig<Config1, TupleConfig<Config2,TupleConfig<Config3, TupleConfigEnd<Config4> > > > Base;
	  typedef TupleConfig4<Config1, Config2, Config3, Config4> This;

	  TupleConfig4() {}
	  TupleConfig4(const This& config);
	  TupleConfig4(const Base& config);
	  TupleConfig4(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,const Config4& cfg4);

	  // access functions
	  inline const Config1_t& first() const { return this->config(); }
	  inline const Config2_t& second() const { return this->rest().config(); }
	  inline const Config3_t& third() const { return this->rest().rest().config(); }
	  inline const Config4_t& fourth() const { return this->rest().rest().rest().config(); }
  };

  template<class Config1, class Config2, class Config3, class Config4>
  TupleConfig4<Config1, Config2, Config3, Config4> expmap(const TupleConfig4<Config1, Config2, Config3, Config4>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class Config1, class Config2, class Config3, class Config4, class Config5>
  class TupleConfig5 : public TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > > > > {
  public:
	  // typedefs
	  typedef Config1 Config1_t;
	  typedef Config2 Config2_t;
	  typedef Config3 Config3_t;
	  typedef Config4 Config4_t;
	  typedef Config5 Config5_t;

	  TupleConfig5() {}
	  TupleConfig5(const TupleConfig5<Config1, Config2, Config3, Config4, Config5>& config);
	  TupleConfig5(const TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfigEnd<Config5> > > > >& config);
	  TupleConfig5(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5);

	  // access functions
	  inline const Config1_t& first() const { return this->config(); }
	  inline const Config2_t& second() const { return this->rest().config(); }
	  inline const Config3_t& third() const { return this->rest().rest().config(); }
	  inline const Config4_t& fourth() const { return this->rest().rest().rest().config(); }
	  inline const Config5_t& fifth() const { return this->rest().rest().rest().rest().config(); }
  };

  template<class Config1, class Config2, class Config3, class Config4, class Config5>
  TupleConfig5<Config1, Config2, Config3, Config4, Config5> expmap(const TupleConfig5<Config1, Config2, Config3, Config4, Config5>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class Config1, class Config2, class Config3, class Config4, class Config5, class Config6>
  class TupleConfig6 : public TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > > > > {
  public:
	  // typedefs
	  typedef Config1 Config1_t;
	  typedef Config2 Config2_t;
	  typedef Config3 Config3_t;
	  typedef Config4 Config4_t;
	  typedef Config5 Config5_t;
	  typedef Config6 Config6_t;

	  TupleConfig6() {}
	  TupleConfig6(const TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>& config);
	  TupleConfig6(const TupleConfig<Config1, TupleConfig<Config2, TupleConfig<Config3, TupleConfig<Config4, TupleConfig<Config5, TupleConfigEnd<Config6> > > > > >& config);
	  TupleConfig6(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5, const Config6& cfg6);
	  // access functions
	  inline const Config1_t& first() const { return this->config(); }
	  inline const Config2_t& second() const { return this->rest().config(); }
	  inline const Config3_t& third() const { return this->rest().rest().config(); }
	  inline const Config4_t& fourth() const { return this->rest().rest().rest().config(); }
	  inline const Config5_t& fifth() const { return this->rest().rest().rest().rest().config(); }
	  inline const Config6_t& sixth() const { return this->rest().rest().rest().rest().rest().config(); }
  };

  template<class Config1, class Config2, class Config3, class Config4, class Config5, class Config6>
  TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6> expmap(const TupleConfig6<Config1, Config2, Config3, Config4, Config5, Config6>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  /**
   * PairConfig: an alias for a pair of configs using TupleConfig2
   * STILL IN TESTING - will soon replace PairConfig
   */
//  template<class J1, class X1, class J2, class X2>
//  class PairConfig : public  TupleConfig2<LieConfig<J1, X1>, LieConfig<J2, X2> > {
//  public:
//	  PairConfig() {}
//	  PairConfig(const PairConfig<J1, X1, J2, X2>& config) :
//		  TupleConfig2<LieConfig<J1, X1>, LieConfig<J2, X2> >(config) {}
//	  PairConfig(const LieConfig<J1, X1>& cfg1,const LieConfig<J2, X2>& cfg2) :
//		  TupleConfig2<LieConfig<J1, X1>, LieConfig<J2, X2> >(cfg1, cfg2) {}
//  };

  /**
   * PairConfig:  a config that holds two data types.
   * Note: this should eventually be replaced with a wrapper on TupleConfig2
   */
  template<class J1, class X1, class J2, class X2>
  class PairConfig : public Testable<PairConfig<J1, X1, J2, X2> > {

  public:

  	// publicly available types
    typedef LieConfig<J1, X1> Config1;
    typedef LieConfig<J2, X2> Config2;

  protected:

    // Two configs in the pair as in std:pair
    LieConfig<J1, X1> first_;
    LieConfig<J2, X2> second_;

  private:

    PairConfig(const LieConfig<J1,X1>& config1, const LieConfig<J2,X2>& config2) :
      first_(config1), second_(config2){}

  public:

    /**
     * Default constructor creates an empty config.
     */
    PairConfig(){}

    /**
     * Copy constructor
     */
    PairConfig(const PairConfig<J1, X1, J2, X2>& c):
      first_(c.first_), second_(c.second_){}

    /**
     * Print
     */
    void print(const std::string& s = "") const;

    /**
     * Test for equality in keys and values
     */
    bool equals(const PairConfig<J1, X1, J2, X2>& c, double tol=1e-9) const {
      return first_.equals(c.first_, tol) && second_.equals(c.second_, tol); }

    /** Returns the real config */
    inline const Config1& first() const { return first_; }
    inline const Config2& second() const { return second_; }

    /**
     * operator[] syntax to get a value by j, throws invalid_argument if
     * value with specified j is not present.  Will generate compile-time
     * errors if j type does not match that on which the Config is templated.
     */
    const X1& operator[](const J1& j) const { return first_[j]; }
    const X2& operator[](const J2& j) const { return second_[j]; }

    /**
     * size is the total number of variables in this config.
     */
    size_t size() const { return first_.size() + second_.size(); }

    /**
     * dim is the dimensionality of the tangent space
     */
    size_t dim() const { return first_.dim() + second_.dim(); }

  private:
    template<class Config, class Key, class Value>
    void insert_helper(Config& config, const Key& j, const Value& value) {
      config.insert(j, value);
    }

    template<class Config, class Key>
    void erase_helper(Config& config, const Key& j) {
      size_t dim;
      config.erase(j, dim);
    }

  public:

    /** zero: create VectorConfig of appropriate structure */
    VectorConfig zero() const {
    	VectorConfig z1 = first_.zero(), z2 = second_.zero();
    	z1.insert(z2);
    	return z1;
    }

    /**
     * Exponential map: expmap each element
     */
    PairConfig<J1,X1,J2,X2> expmap(const VectorConfig& delta) const {
      return PairConfig(gtsam::expmap(first_, delta), gtsam::expmap(second_, delta)); }

    /**
     * Logarithm: logmap each element
     */
    VectorConfig logmap(const PairConfig<J1,X1,J2,X2>& cp) const {
    	VectorConfig ret(gtsam::logmap(first_, cp.first_));
    	ret.insert(gtsam::logmap(second_, cp.second_));
    	return ret;
    }

    /**
     * Insert a variable with the given j
     */
    void insert(const J1& j, const X1& value) { insert_helper(first_, j, value); }
    void insert(const J2& j, const X2& value) { insert_helper(second_, j, value); }

    void insert(const PairConfig& config);

    /**
     * Remove the variable with the given j.  Throws invalid_argument if the
     * j is not present in the config.
     */
    void erase(const J1& j) { erase_helper(first_, j); }
    void erase(const J2& j) { erase_helper(second_, j); }

    /**
     * Check if a variable exists
     */
    bool exists(const J1& j) const { return first_.exists(j); }
    bool exists(const J2& j) const { return second_.exists(j); }


  };

  /** exponential map */
  template<class J1, class X1, class J2, class X2>
	inline PairConfig<J1, X1, J2, X2> expmap(const PairConfig<J1, X1, J2, X2>& c,
			const VectorConfig& delta) {
		return c.expmap(delta);
	}

  /** log, inverse of exponential map */
	template<class J1, class X1, class J2, class X2>
	inline VectorConfig logmap(const PairConfig<J1, X1, J2, X2>& c0,
			const PairConfig<J1, X1, J2, X2>& cp) {
		return c0.logmap(cp);
	}

}
