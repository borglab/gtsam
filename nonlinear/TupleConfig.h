/**
 * @file TupleConfig.h
 * @author Richard Roberts
 * @author Manohar Paluri
 * @author Alex Cunningham
 */

#include <gtsam/nonlinear/LieConfig.h>
#include <gtsam/linear/VectorConfig.h>

#pragma once

namespace gtsam {

 /**
   *  TupleConfigs are a structure to manage heterogenous LieConfigs, so as to
   *  enable different types of variables/keys to be used simultaneously.  The
   *  interface is designed to mimic that of a single LieConfig.
   *
   *  This uses a recursive structure of config pairs to form a lisp-like
   *  list, with a special case (TupleConfigEnd) that contains only one config
   *  at the end.  Because this recursion is done at compile time, there is no
   *  runtime performance hit to using this structure.
   *
   *  For an easier to use approach, there are TupleConfigN classes, which wrap
   *  the recursive TupleConfigs together as a single class, so you can have
   *  mixed-type classes from 2-6 types.  Note that a TupleConfig2 is equivalent
   *  to the previously used PairConfig.
   *
   *  Design and extension note:
   *  To implement a recursively templated data structure, note that most operations
   *  have two versions: one with templates and one without.  The templated one allows
   *  for the arguments to be passed to the next config, while the specialized one
   *  operates on the "first" config. TupleConfigEnd contains only the specialized version.
   */
  template<class Config1, class Config2>
  class TupleConfig : public Testable<TupleConfig<Config1, Config2> > {

  protected:
	  // Data for internal configs
	  Config1 first_;	/// Arbitrary config
	  Config2 second_;	/// A TupleConfig or TupleConfigEnd, which wraps an arbitrary config

  public:
	  // typedefs for config subtypes
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
	  void print(const std::string& s = "") const {
			first_.print(s);
			second_.print();
	  }

	  /** Equality with tolerance for keys and values */
	  bool equals(const TupleConfig<Config1, Config2>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol) && second_.equals(c.second_, tol);
	  }

	  /**
	   * Insert a key/value pair to the config.
	   * Note: if the key is already in the config, the config will not be changed.
	   * Use update() to allow for changing existing values.
	   * @param key is the key - can be an int (second version) if the can can be initialized from an int
	   * @param value is the value to insert
	   */
	  template<class Key, class Value>
	  void insert(const Key& key, const Value& value) {second_.insert(key, value);}
	  void insert(const int& key, const Value1& value) {first_.insert(Key1(key), value);}
	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value);}

	  /**
	   * Insert a complete config at a time.
	   * Note: if the key is already in the config, the config will not be changed.
	   * Use update() to allow for changing existing values.
	   * @param config is a full config to add
	   */
	  template<class Cfg1, class Cfg2>
	  void insert(const TupleConfig<Cfg1, Cfg2>& config) { second_.insert(config); }
	  void insert(const TupleConfig<Config1, Config2>& config) {
		  first_.insert(config.first_);
		  second_.insert(config.second_);
	  }

	  /**
	   * Update function for whole configs - this will change existing values
	   * @param config is a config to add
	   */
	  template<class Cfg1, class Cfg2>
	  void update(const TupleConfig<Cfg1, Cfg2>& config) { second_.update(config); }
	  void update(const TupleConfig<Config1, Config2>& config) {
	  	first_.update(config.first_);
	  	second_.update(config.second_);
	  }

	  /**
	   * Update function for single key/value pairs - will change existing values
	   * @param key is the variable identifier
	   * @param value is the variable value to update
	   */
	  template<class Key, class Value>
	  void update(const Key& key, const Value& value) { second_.update(key, value); }
	  void update(const Key1& key, const Value1& value) { first_.update(key, value); }

	  /**
	   * Insert a subconfig
	   * @param config is the config to insert
	   */
	  template<class Cfg>
	  void insertSub(const Cfg& config) { second_.insertSub(config); }
	  void insertSub(const Config1& config) { first_.insert(config);  }

	  /** erase an element by key */
	  template<class Key>
	  void erase(const Key& j)  { second_.erase(j); }
	  void erase(const Key1& j)  { first_.erase(j); }

	  /** clears the config */
	  void clear() { first_.clear(); second_.clear(); }

	  /** determine whether an element exists */
	  template<class Key>
	  bool exists(const Key& j) const { return second_.exists(j); }
	  bool exists(const Key1& j) const { return first_.exists(j); }

	  /** a variant of exists */
	  template<class Key>
	  boost::optional<typename Key::Value_t> exists_(const Key& j)  const { return second_.exists_(j); }
	  boost::optional<Value1>                exists_(const Key1& j) const { return first_.exists_(j); }

	  /** access operator */
	  template<class Key>
	  const typename Key::Value_t & operator[](const Key& j) const { return second_[j]; }
	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  /** at access function */
	  template<class Key>
	  const typename Key::Value_t & at(const Key& j) const { return second_.at(j); }
	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  /** direct config access */
	  const Config1& config() const { return first_; }
	  const Config2& rest() const { return second_; }

	  /** zero: create VectorConfig of appropriate structure */
	  VectorConfig zero() const {
		  VectorConfig z1 = first_.zero(), z2 = second_.zero();
		  z2.insert(z1);
		  return z2;
	  }

	  /** @return number of key/value pairs stored */
	  size_t size() const { return first_.size() + second_.size(); }

	  /** @return true if config is empty */
	  bool empty() const { return first_.empty() && second_.empty(); }

	  /** @return The dimensionality of the tangent space */
	  size_t dim() const { return first_.dim() + second_.dim(); }

	  /** Expmap */
	  TupleConfig<Config1, Config2> expmap(const VectorConfig& delta) const {
	        return TupleConfig(gtsam::expmap(first_, delta), second_.expmap(delta));
	  }

	  /** logmap each element */
	  VectorConfig logmap(const TupleConfig<Config1, Config2>& cp) const {
		  VectorConfig ret(gtsam::logmap(first_, cp.first_));
		  ret.insert(second_.logmap(cp.second_));
		  return ret;
	  }

  private:
	  /** Serialization function */
	  friend class boost::serialization::access;
	  template<class Archive>
	  void serialize(Archive & ar, const unsigned int version) {
		  ar & BOOST_SERIALIZATION_NVP(first_);
		  ar & BOOST_SERIALIZATION_NVP(second_);
	  }

  };

  /**
   * End of a recursive TupleConfig - contains only one config
   *
   * Do not use this class directly - it should only be used as a part
   * of a recursive structure
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

	  void print(const std::string& s = "") const {
			first_.print();
	  }

	  bool equals(const TupleConfigEnd<Config>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol);
	  }

	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value); }
	  void insert(const int& key, const Value1& value) {first_.insert(Key1(key), value);}

	  void insert(const TupleConfigEnd<Config>& config) {first_.insert(config.first_); }

	  void update(const TupleConfigEnd<Config>& config) {first_.update(config.first_); }

	  void update(const Key1& key, const Value1& value) { first_.update(key, value); }

	  void insertSub(const Config& config) {first_.insert(config); }

	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  const Config& config() const { return first_; }

	  void erase(const Key1& j) { first_.erase(j); }

	  void clear() { first_.clear(); }

	  bool empty() const { return first_.empty(); }

	  bool exists(const Key1& j) const { return first_.exists(j); }

	  boost::optional<Value1> exists_(const Key1& j) const { return first_.exists_(j); }

	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  VectorConfig zero() const {
		  VectorConfig z = first_.zero();
		  return z;
	  }

	  size_t size() const { return first_.size(); }

	  size_t dim() const { return first_.dim(); }

	  TupleConfigEnd<Config> expmap(const VectorConfig& delta) const {
	        return TupleConfigEnd(gtsam::expmap(first_, delta));
	  }

	  VectorConfig logmap(const TupleConfigEnd<Config>& cp) const {
		  VectorConfig ret(gtsam::logmap(first_, cp.first_));
		  return ret;
	  }

  private:
	  friend class boost::serialization::access;
	  template<class Archive>
	  void serialize(Archive & ar, const unsigned int version) {
		  ar & BOOST_SERIALIZATION_NVP(first_);
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

  template<class C1>
  class TupleConfig1 : public TupleConfigEnd<C1> {
  public:
 	  // typedefs
 	  typedef C1 Config1;

 	  typedef TupleConfigEnd<C1> Base;
 	  typedef TupleConfig1<C1> This;

 	  TupleConfig1() {}
 	  TupleConfig1(const This& config);
 	  TupleConfig1(const Base& config);
 	  TupleConfig1(const Config1& cfg1);

 	  // access functions
 	  inline const Config1& first() const { return this->config(); }
  };

  template<class C1>
  TupleConfig1<C1> expmap(const TupleConfig1<C1>& c, const VectorConfig& delta) {
 	  return c.expmap(delta);
  }

  template<class C1>
  VectorConfig logmap(const TupleConfig1<C1>& c1, const TupleConfig1<C1>& c2) {
 	  return c1.logmap(c2);
  }

  template<class C1, class C2>
  class TupleConfig2 : public TupleConfig<C1, TupleConfigEnd<C2> > {
  public:
	  // typedefs
	  typedef C1 Config1;
	  typedef C2 Config2;

	  typedef TupleConfig<C1, TupleConfigEnd<C2> > Base;
	  typedef TupleConfig2<C1, C2> This;

	  TupleConfig2() {}
	  TupleConfig2(const This& config);
	  TupleConfig2(const Base& config);
	  TupleConfig2(const Config1& cfg1, const Config2& cfg2);

	  // access functions
	  inline const Config1& first() const { return this->config(); }
	  inline const Config2& second() const { return this->rest().config(); }
  };

  template<class C1, class C2>
  TupleConfig2<C1, C2> expmap(const TupleConfig2<C1, C2>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class C1, class C2>
  VectorConfig logmap(const TupleConfig2<C1, C2>& c1, const TupleConfig2<C1, C2>& c2) {
	  return c1.logmap(c2);
  }

  template<class C1, class C2, class C3>
  class TupleConfig3 : public TupleConfig<C1, TupleConfig<C2, TupleConfigEnd<C3> > > {
  public:
	  // typedefs
	  typedef C1 Config1;
	  typedef C2 Config2;
	  typedef C3 Config3;

	  TupleConfig3() {}
	  TupleConfig3(const TupleConfig<C1, TupleConfig<C2, TupleConfigEnd<C3> > >& config);
	  TupleConfig3(const TupleConfig3<C1, C2, C3>& config);
	  TupleConfig3(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3);

	  // access functions
	  inline const Config1& first() const { return this->config(); }
	  inline const Config2& second() const { return this->rest().config(); }
	  inline const Config3& third() const { return this->rest().rest().config(); }
  };

  template<class C1, class C2, class C3>
  TupleConfig3<C1, C2, C3> expmap(const TupleConfig3<C1, C2, C3>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class C1, class C2, class C3, class C4>
  class TupleConfig4 : public TupleConfig<C1, TupleConfig<C2,TupleConfig<C3, TupleConfigEnd<C4> > > > {
  public:
	  // typedefs
	  typedef C1 Config1;
	  typedef C2 Config2;
	  typedef C3 Config3;
	  typedef C4 Config4;

	  typedef TupleConfig<C1, TupleConfig<C2,TupleConfig<C3, TupleConfigEnd<C4> > > > Base;
	  typedef TupleConfig4<C1, C2, C3, C4> This;

	  TupleConfig4() {}
	  TupleConfig4(const This& config);
	  TupleConfig4(const Base& config);
	  TupleConfig4(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,const Config4& cfg4);

	  // access functions
	  inline const Config1& first() const { return this->config(); }
	  inline const Config2& second() const { return this->rest().config(); }
	  inline const Config3& third() const { return this->rest().rest().config(); }
	  inline const Config4& fourth() const { return this->rest().rest().rest().config(); }
  };

  template<class C1, class C2, class C3, class C4>
  TupleConfig4<C1, C2, C3, C4> expmap(const TupleConfig4<C1, C2, C3, C4>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class C1, class C2, class C3, class C4, class C5>
  class TupleConfig5 : public TupleConfig<C1, TupleConfig<C2, TupleConfig<C3, TupleConfig<C4, TupleConfigEnd<C5> > > > > {
  public:
	  // typedefs
	  typedef C1 Config1;
	  typedef C2 Config2;
	  typedef C3 Config3;
	  typedef C4 Config4;
	  typedef C5 Config5;

	  TupleConfig5() {}
	  TupleConfig5(const TupleConfig5<C1, C2, C3, C4, C5>& config);
	  TupleConfig5(const TupleConfig<C1, TupleConfig<C2, TupleConfig<C3, TupleConfig<C4, TupleConfigEnd<C5> > > > >& config);
	  TupleConfig5(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5);

	  // access functions
	  inline const Config1& first() const { return this->config(); }
	  inline const Config2& second() const { return this->rest().config(); }
	  inline const Config3& third() const { return this->rest().rest().config(); }
	  inline const Config4& fourth() const { return this->rest().rest().rest().config(); }
	  inline const Config5& fifth() const { return this->rest().rest().rest().rest().config(); }
  };

  template<class C1, class C2, class C3, class C4, class C5>
  TupleConfig5<C1, C2, C3, C4, C5> expmap(const TupleConfig5<C1, C2, C3, C4, C5>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

  template<class C1, class C2, class C3, class C4, class C5, class C6>
  class TupleConfig6 : public TupleConfig<C1, TupleConfig<C2, TupleConfig<C3, TupleConfig<C4, TupleConfig<C5, TupleConfigEnd<C6> > > > > > {
  public:
	  // typedefs
	  typedef C1 Config1;
	  typedef C2 Config2;
	  typedef C3 Config3;
	  typedef C4 Config4;
	  typedef C5 Config5;
	  typedef C6 Config6;

	  TupleConfig6() {}
	  TupleConfig6(const TupleConfig6<C1, C2, C3, C4, C5, C6>& config);
	  TupleConfig6(const TupleConfig<C1, TupleConfig<C2, TupleConfig<C3, TupleConfig<C4, TupleConfig<C5, TupleConfigEnd<C6> > > > > >& config);
	  TupleConfig6(const Config1& cfg1, const Config2& cfg2, const Config3& cfg3,
				   const Config4& cfg4, const Config5& cfg5, const Config6& cfg6);
	  // access functions
	  inline const Config1& first() const { return this->config(); }
	  inline const Config2& second() const { return this->rest().config(); }
	  inline const Config3& third() const { return this->rest().rest().config(); }
	  inline const Config4& fourth() const { return this->rest().rest().rest().config(); }
	  inline const Config5& fifth() const { return this->rest().rest().rest().rest().config(); }
	  inline const Config6& sixth() const { return this->rest().rest().rest().rest().rest().config(); }
  };

  template<class C1, class C2, class C3, class C4, class C5, class C6>
  TupleConfig6<C1, C2, C3, C4, C5, C6> expmap(const TupleConfig6<C1, C2, C3, C4, C5, C6>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
  }

}
