/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file TupleValues.h
 * @author Richard Roberts
 * @author Manohar Paluri
 * @author Alex Cunningham
 */

#include <gtsam/nonlinear/LieValues.h>
#include <gtsam/linear/VectorValues.h>

#pragma once

namespace gtsam {

 /**
   *  TupleValuess are a structure to manage heterogenous LieValuess, so as to
   *  enable different types of variables/keys to be used simultaneously.  We
   *  do this with recursive templates (instead of blind pointer casting) to
   *  reduce run-time overhead and keep static type checking.  The interface
   *  mimics that of a single LieValues.
   *
   *  This uses a recursive structure of config pairs to form a lisp-like
   *  list, with a special case (TupleValuesEnd) that contains only one config
   *  at the end.  Because this recursion is done at compile time, there is no
   *  runtime performance hit to using this structure.
   *
   *  For an easy interface, there are TupleValuesN classes, which wrap
   *  the recursive TupleValuess together as a single class, so you can have
   *  mixed-type classes from 2-6 types.  Note that a TupleValues2 is equivalent
   *  to the previously used PairValues.
   *
   *  Design and extension note:
   *  To implement a recursively templated data structure, note that most operations
   *  have two versions: one with templates and one without.  The templated one allows
   *  for the arguments to be passed to the next config, while the specialized one
   *  operates on the "first" config. TupleValuesEnd contains only the specialized version.
   */
  template<class Values1, class Values2>
  class TupleValues : public Testable<TupleValues<Values1, Values2> > {

  protected:
	  // Data for internal configs
	  Values1 first_;	/// Arbitrary config
	  Values2 second_;	/// A TupleValues or TupleValuesEnd, which wraps an arbitrary config

  public:
	  // typedefs for config subtypes
	  typedef class Values1::Key Key1;
	  typedef class Values1::Value Value1;

	  /** default constructor */
	  TupleValues() {}

	  /** Copy constructor */
	  TupleValues(const TupleValues<Values1, Values2>& config) :
		  first_(config.first_), second_(config.second_) {}

	  /** Construct from configs */
	  TupleValues(const Values1& cfg1, const Values2& cfg2) :
		  first_(cfg1), second_(cfg2) {}

	  /** Print */
	  void print(const std::string& s = "") const {
			first_.print(s);
			second_.print();
	  }

	  /** Equality with tolerance for keys and values */
	  bool equals(const TupleValues<Values1, Values2>& c, double tol=1e-9) const {
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
	  void insert(int key, const Value1& value) {first_.insert(Key1(key), value);}
	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value);}

	  /**
	   * Insert a complete config at a time.
	   * Note: if the key is already in the config, the config will not be changed.
	   * Use update() to allow for changing existing values.
	   * @param config is a full config to add
	   */
	  template<class Cfg1, class Cfg2>
	  void insert(const TupleValues<Cfg1, Cfg2>& config) { second_.insert(config); }
	  void insert(const TupleValues<Values1, Values2>& config) {
		  first_.insert(config.first_);
		  second_.insert(config.second_);
	  }

	  /**
	   * Update function for whole configs - this will change existing values
	   * @param config is a config to add
	   */
	  template<class Cfg1, class Cfg2>
	  void update(const TupleValues<Cfg1, Cfg2>& config) { second_.update(config); }
	  void update(const TupleValues<Values1, Values2>& config) {
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
	  void insertSub(const Values1& config) { first_.insert(config);  }

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
	  boost::optional<typename Key::Value> exists_(const Key& j)  const { return second_.exists_(j); }
	  boost::optional<Value1>                exists_(const Key1& j) const { return first_.exists_(j); }

	  /** access operator */
	  template<class Key>
	  const typename Key::Value & operator[](const Key& j) const { return second_[j]; }
	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  /** at access function */
	  template<class Key>
	  const typename Key::Value & at(const Key& j) const { return second_.at(j); }
	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  /** direct config access */
	  const Values1& config() const { return first_; }
	  const Values2& rest() const { return second_; }

	  /** zero: create VectorValues of appropriate structure */
	  VectorValues zero(const Ordering& ordering) const {
		  VectorValues z(this->dims(ordering));
		  z.makeZero();
		  return z;
	  }

	  /** @return number of key/value pairs stored */
	  size_t size() const { return first_.size() + second_.size(); }

	  /** @return true if config is empty */
	  bool empty() const { return first_.empty() && second_.empty(); }

	  /** @return The dimensionality of the tangent space */
	  size_t dim() const { return first_.dim() + second_.dim(); }

	  /** Create an array of variable dimensions using the given ordering */
	  std::vector<size_t> dims(const Ordering& ordering) const {
	    _ValuesDimensionCollector dimCollector(ordering);
	    this->apply(dimCollector);
	    return dimCollector.dimensions;
	  }

    /**
     * Generate a default ordering, simply in key sort order.  To instead
     * create a fill-reducing ordering, use
     * NonlinearFactorGraph::orderingCOLAMD().  Alternatively, you may permute
     * this ordering yourself (as orderingCOLAMD() does internally).
     */
    Ordering::shared_ptr orderingArbitrary(Index firstVar = 0) const {
      // Generate an initial key ordering in iterator order
      _ValuesKeyOrderer keyOrderer(firstVar);
      this->apply(keyOrderer);
      return keyOrderer.ordering;
    }

	  /** Expmap */
	  TupleValues<Values1, Values2> expmap(const VectorValues& delta, const Ordering& ordering) const {
	    return TupleValues(first_.expmap(delta, ordering), second_.expmap(delta, ordering));
	  }

	  /** logmap each element */
	  VectorValues logmap(const TupleValues<Values1, Values2>& cp, const Ordering& ordering) const {
		  VectorValues delta(this->dims(ordering));
		  logmap(cp, ordering, delta);
		  return delta;
	  }

    /** logmap each element */
    void logmap(const TupleValues<Values1, Values2>& cp, const Ordering& ordering, VectorValues& delta) const {
      first_.logmap(cp.first_, ordering, delta);
      second_.logmap(cp.second_, ordering, delta);
    }

	  /**
	   * Apply a class with an application operator() to a const_iterator over
	   * every <key,value> pair.  The operator must be able to handle such an
	   * iterator for every type in the Values, (i.e. through templating).
	   */
    template<typename A>
    void apply(A& operation) {
      first_.apply(operation);
      second_.apply(operation);
    }
	  template<typename A>
	  void apply(A& operation) const {
	    first_.apply(operation);
	    second_.apply(operation);
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
   * End of a recursive TupleValues - contains only one config
   *
   * Do not use this class directly - it should only be used as a part
   * of a recursive structure
   */
  template<class Values>
  class TupleValuesEnd : public Testable<TupleValuesEnd<Values> > {

  protected:
	  // Data for internal configs
	  Values first_;

  public:
	  // typedefs
	  typedef class Values::Key Key1;
	  typedef class Values::Value Value1;

	  TupleValuesEnd() {}

	  TupleValuesEnd(const TupleValuesEnd<Values>& config) :
		  first_(config.first_) {}

	  TupleValuesEnd(const Values& cfg) :
		  first_(cfg) {}

	  void print(const std::string& s = "") const {
			first_.print();
	  }

	  bool equals(const TupleValuesEnd<Values>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol);
	  }

	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value); }
	  void insert(int key, const Value1& value) {first_.insert(Key1(key), value);}

	  void insert(const TupleValuesEnd<Values>& config) {first_.insert(config.first_); }

	  void update(const TupleValuesEnd<Values>& config) {first_.update(config.first_); }

	  void update(const Key1& key, const Value1& value) { first_.update(key, value); }

	  void insertSub(const Values& config) {first_.insert(config); }

	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  const Values& config() const { return first_; }

	  void erase(const Key1& j) { first_.erase(j); }

	  void clear() { first_.clear(); }

	  bool empty() const { return first_.empty(); }

	  bool exists(const Key1& j) const { return first_.exists(j); }

	  boost::optional<Value1> exists_(const Key1& j) const { return first_.exists_(j); }

	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  VectorValues zero(const Ordering& ordering) const {
		  VectorValues z(this->dims(ordering));
		  z.makeZero();
		  return z;
	  }

	  size_t size() const { return first_.size(); }

	  size_t dim() const { return first_.dim(); }

	  TupleValuesEnd<Values> expmap(const VectorValues& delta, const Ordering& ordering) const {
	        return TupleValuesEnd(first_.expmap(delta, ordering));
	  }

    VectorValues logmap(const TupleValuesEnd<Values>& cp, const Ordering& ordering) const {
      VectorValues delta(this->dims(ordering));
      logmap(cp, ordering, delta);
      return delta;
    }

    void logmap(const TupleValuesEnd<Values>& cp, const Ordering& ordering, VectorValues& delta) const {
      first_.logmap(cp.first_, ordering, delta);
    }

    /**
     * Apply a class with an application operator() to a const_iterator over
     * every <key,value> pair.  The operator must be able to handle such an
     * iterator for every type in the Values, (i.e. through templating).
     */
    template<typename A>
    void apply(A& operation) {
      first_.apply(operation);
    }
    template<typename A>
    void apply(A& operation) const {
      first_.apply(operation);
    }

  private:
	  friend class boost::serialization::access;
	  template<class Archive>
	  void serialize(Archive & ar, const unsigned int version) {
		  ar & BOOST_SERIALIZATION_NVP(first_);
	  }
  };

  /**
   * Wrapper classes to act as containers for configs.  Note that these can be cascaded
   * recursively, as they are TupleValuess, and are primarily a short form of the config
   * structure to make use of the TupleValuess easier.
   *
   * The interface is designed to mimic PairValues, but for 2-6 config types.
   */

  template<class C1>
  class TupleValues1 : public TupleValuesEnd<C1> {
  public:
 	  // typedefs
 	  typedef C1 Values1;

 	  typedef TupleValuesEnd<C1> Base;
 	  typedef TupleValues1<C1> This;

 	  TupleValues1() {}
 	  TupleValues1(const This& config);
 	  TupleValues1(const Base& config);
 	  TupleValues1(const Values1& cfg1);

 	  // access functions
 	  inline const Values1& first() const { return this->config(); }
  };

  template<class C1, class C2>
  class TupleValues2 : public TupleValues<C1, TupleValuesEnd<C2> > {
  public:
	  // typedefs
	  typedef C1 Values1;
	  typedef C2 Values2;

	  typedef TupleValues<C1, TupleValuesEnd<C2> > Base;
	  typedef TupleValues2<C1, C2> This;

	  TupleValues2() {}
	  TupleValues2(const This& config);
	  TupleValues2(const Base& config);
	  TupleValues2(const Values1& cfg1, const Values2& cfg2);

	  // access functions
	  inline const Values1& first() const { return this->config(); }
	  inline const Values2& second() const { return this->rest().config(); }
  };

  template<class C1, class C2, class C3>
  class TupleValues3 : public TupleValues<C1, TupleValues<C2, TupleValuesEnd<C3> > > {
  public:
	  // typedefs
	  typedef C1 Values1;
	  typedef C2 Values2;
	  typedef C3 Values3;

	  TupleValues3() {}
	  TupleValues3(const TupleValues<C1, TupleValues<C2, TupleValuesEnd<C3> > >& config);
	  TupleValues3(const TupleValues3<C1, C2, C3>& config);
	  TupleValues3(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3);

	  // access functions
	  inline const Values1& first() const { return this->config(); }
	  inline const Values2& second() const { return this->rest().config(); }
	  inline const Values3& third() const { return this->rest().rest().config(); }
  };

  template<class C1, class C2, class C3, class C4>
  class TupleValues4 : public TupleValues<C1, TupleValues<C2,TupleValues<C3, TupleValuesEnd<C4> > > > {
  public:
	  // typedefs
	  typedef C1 Values1;
	  typedef C2 Values2;
	  typedef C3 Values3;
	  typedef C4 Values4;

	  typedef TupleValues<C1, TupleValues<C2,TupleValues<C3, TupleValuesEnd<C4> > > > Base;
	  typedef TupleValues4<C1, C2, C3, C4> This;

	  TupleValues4() {}
	  TupleValues4(const This& config);
	  TupleValues4(const Base& config);
	  TupleValues4(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,const Values4& cfg4);

	  // access functions
	  inline const Values1& first() const { return this->config(); }
	  inline const Values2& second() const { return this->rest().config(); }
	  inline const Values3& third() const { return this->rest().rest().config(); }
	  inline const Values4& fourth() const { return this->rest().rest().rest().config(); }
  };

  template<class C1, class C2, class C3, class C4, class C5>
  class TupleValues5 : public TupleValues<C1, TupleValues<C2, TupleValues<C3, TupleValues<C4, TupleValuesEnd<C5> > > > > {
  public:
	  // typedefs
	  typedef C1 Values1;
	  typedef C2 Values2;
	  typedef C3 Values3;
	  typedef C4 Values4;
	  typedef C5 Values5;

	  TupleValues5() {}
	  TupleValues5(const TupleValues5<C1, C2, C3, C4, C5>& config);
	  TupleValues5(const TupleValues<C1, TupleValues<C2, TupleValues<C3, TupleValues<C4, TupleValuesEnd<C5> > > > >& config);
	  TupleValues5(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,
				   const Values4& cfg4, const Values5& cfg5);

	  // access functions
	  inline const Values1& first() const { return this->config(); }
	  inline const Values2& second() const { return this->rest().config(); }
	  inline const Values3& third() const { return this->rest().rest().config(); }
	  inline const Values4& fourth() const { return this->rest().rest().rest().config(); }
	  inline const Values5& fifth() const { return this->rest().rest().rest().rest().config(); }
  };

  template<class C1, class C2, class C3, class C4, class C5, class C6>
  class TupleValues6 : public TupleValues<C1, TupleValues<C2, TupleValues<C3, TupleValues<C4, TupleValues<C5, TupleValuesEnd<C6> > > > > > {
  public:
	  // typedefs
	  typedef C1 Values1;
	  typedef C2 Values2;
	  typedef C3 Values3;
	  typedef C4 Values4;
	  typedef C5 Values5;
	  typedef C6 Values6;

	  TupleValues6() {}
	  TupleValues6(const TupleValues6<C1, C2, C3, C4, C5, C6>& config);
	  TupleValues6(const TupleValues<C1, TupleValues<C2, TupleValues<C3, TupleValues<C4, TupleValues<C5, TupleValuesEnd<C6> > > > > >& config);
	  TupleValues6(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,
				   const Values4& cfg4, const Values5& cfg5, const Values6& cfg6);
	  // access functions
	  inline const Values1& first() const { return this->config(); }
	  inline const Values2& second() const { return this->rest().config(); }
	  inline const Values3& third() const { return this->rest().rest().config(); }
	  inline const Values4& fourth() const { return this->rest().rest().rest().config(); }
	  inline const Values5& fifth() const { return this->rest().rest().rest().rest().config(); }
	  inline const Values6& sixth() const { return this->rest().rest().rest().rest().rest().config(); }
  };

}
