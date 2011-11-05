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
   *  TupleValues are a structure to manage heterogenous LieValues, so as to
   *  enable different types of variables/keys to be used simultaneously.  We
   *  do this with recursive templates (instead of blind pointer casting) to
   *  reduce run-time overhead and keep static type checking.  The interface
   *  mimics that of a single LieValues.
   *
   *  This uses a recursive structure of values pairs to form a lisp-like
   *  list, with a special case (TupleValuesEnd) that contains only one values
   *  at the end.  Because this recursion is done at compile time, there is no
   *  runtime performance hit to using this structure.
   *
   *  For an easy interface, there are TupleValuesN classes, which wrap
   *  the recursive TupleValues together as a single class, so you can have
   *  mixed-type classes from 2-6 types.  Note that a TupleValues2 is equivalent
   *  to the previously used PairValues.
   *
   *  Design and extension note:
   *  To implement a recursively templated data structure, note that most operations
   *  have two versions: one with templates and one without.  The templated one allows
   *  for the arguments to be passed to the next values, while the specialized one
   *  operates on the "first" values. TupleValuesEnd contains only the specialized version.
   */
  template<class VALUES1, class VALUES2>
  class TupleValues {

  protected:
	  // Data for internal valuess
	  VALUES1 first_;	/// Arbitrary values
	  VALUES2 second_;	/// A TupleValues or TupleValuesEnd, which wraps an arbitrary values

	  /** concept checks */
	  GTSAM_CONCEPT_TESTABLE_TYPE(VALUES1)
	  GTSAM_CONCEPT_TESTABLE_TYPE(VALUES2)

  public:
	  // typedefs for values subtypes
	  typedef typename VALUES1::Key Key1;
	  typedef typename VALUES1::Value Value1;

	  /** default constructor */
	  TupleValues() {}

	  /** Copy constructor */
	  TupleValues(const TupleValues<VALUES1, VALUES2>& values) :
		  first_(values.first_), second_(values.second_) {}

	  /** Construct from valuess */
	  TupleValues(const VALUES1& cfg1, const VALUES2& cfg2) :
		  first_(cfg1), second_(cfg2) {}

	  /** Print */
	  void print(const std::string& s = "") const {
			first_.print(s);
			second_.print();
	  }

	  /** Equality with tolerance for keys and values */
	  bool equals(const TupleValues<VALUES1, VALUES2>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol) && second_.equals(c.second_, tol);
	  }

	  /**
	   * Insert a key/value pair to the values.
	   * Note: if the key is already in the values, the values will not be changed.
	   * Use update() to allow for changing existing values.
	   * @param key is the key - can be an int (second version) if the can can be initialized from an int
	   * @param value is the value to insert
	   */
	  template<class KEY, class VALUE>
	  void insert(const KEY& key, const VALUE& value) {second_.insert(key, value);}
	  void insert(int key, const Value1& value) {first_.insert(Key1(key), value);}
	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value);}

	  /**
	   * Insert a complete values at a time.
	   * Note: if the key is already in the values, the values will not be changed.
	   * Use update() to allow for changing existing values.
	   * @param values is a full values to add
	   */
	  template<class CFG1, class CFG2>
	  void insert(const TupleValues<CFG1, CFG2>& values) { second_.insert(values); }
	  void insert(const TupleValues<VALUES1, VALUES2>& values) {
		  first_.insert(values.first_);
		  second_.insert(values.second_);
	  }

	  /**
	   * Update function for whole valuess - this will change existing values
	   * @param values is a values to add
	   */
	  template<class CFG1, class CFG2>
	  void update(const TupleValues<CFG1, CFG2>& values) { second_.update(values); }
	  void update(const TupleValues<VALUES1, VALUES2>& values) {
	  	first_.update(values.first_);
	  	second_.update(values.second_);
	  }

	  /**
	   * Update function for single key/value pairs - will change existing values
	   * @param key is the variable identifier
	   * @param value is the variable value to update
	   */
	  template<class KEY, class VALUE>
	  void update(const KEY& key, const VALUE& value) { second_.update(key, value); }
	  void update(const Key1& key, const Value1& value) { first_.update(key, value); }

	  /**
	   * Insert a subvalues
	   * @param values is the values to insert
	   */
	  template<class CFG>
	  void insertSub(const CFG& values) { second_.insertSub(values); }
	  void insertSub(const VALUES1& values) { first_.insert(values);  }

	  /** erase an element by key */
	  template<class KEY>
	  void erase(const KEY& j)  { second_.erase(j); }
	  void erase(const Key1& j)  { first_.erase(j); }

	  /** clears the values */
	  void clear() { first_.clear(); second_.clear(); }

	  /** determine whether an element exists */
	  template<class KEY>
	  bool exists(const KEY& j) const { return second_.exists(j); }
	  bool exists(const Key1& j) const { return first_.exists(j); }

	  /** a variant of exists */
	  template<class KEY>
	  boost::optional<typename KEY::Value> exists_(const KEY& j)  const { return second_.exists_(j); }
	  boost::optional<Value1>                exists_(const Key1& j) const { return first_.exists_(j); }

	  /** access operator */
	  template<class KEY>
	  const typename KEY::Value & operator[](const KEY& j) const { return second_[j]; }
	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  /** at access function */
	  template<class KEY>
	  const typename KEY::Value & at(const KEY& j) const { return second_.at(j); }
	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  /** direct values access */
	  const VALUES1& values() const { return first_; }
	  const VALUES2& rest() const { return second_; }

	  /** zero: create VectorValues of appropriate structure */
	  VectorValues zero(const Ordering& ordering) const {
		  return VectorValues::Zero(this->dims(ordering));
	  }

	  /** @return number of key/value pairs stored */
	  size_t size() const { return first_.size() + second_.size(); }

	  /** @return true if values is empty */
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
	  TupleValues<VALUES1, VALUES2> retract(const VectorValues& delta, const Ordering& ordering) const {
	    return TupleValues(first_.retract(delta, ordering), second_.retract(delta, ordering));
	  }

	  /** logmap each element */
	  VectorValues localCoordinates(const TupleValues<VALUES1, VALUES2>& cp, const Ordering& ordering) const {
		  VectorValues delta(this->dims(ordering));
		  localCoordinates(cp, ordering, delta);
		  return delta;
	  }

    /** logmap each element */
    void localCoordinates(const TupleValues<VALUES1, VALUES2>& cp, const Ordering& ordering, VectorValues& delta) const {
      first_.localCoordinates(cp.first_, ordering, delta);
      second_.localCoordinates(cp.second_, ordering, delta);
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
	  template<class ARCHIVE>
	  void serialize(ARCHIVE & ar, const unsigned int version) {
		  ar & BOOST_SERIALIZATION_NVP(first_);
		  ar & BOOST_SERIALIZATION_NVP(second_);
	  }

  };

  /**
   * End of a recursive TupleValues - contains only one values
   *
   * Do not use this class directly - it should only be used as a part
   * of a recursive structure
   */
  template<class VALUES>
  class TupleValuesEnd {

  protected:
	  // Data for internal valuess
	  VALUES first_;

  public:
	  // typedefs
	  typedef typename VALUES::Key Key1;
	  typedef typename VALUES::Value Value1;

	  TupleValuesEnd() {}

	  TupleValuesEnd(const TupleValuesEnd<VALUES>& values) :
		  first_(values.first_) {}

	  TupleValuesEnd(const VALUES& cfg) :
		  first_(cfg) {}

	  void print(const std::string& s = "") const {
			first_.print();
	  }

	  bool equals(const TupleValuesEnd<VALUES>& c, double tol=1e-9) const {
		  return first_.equals(c.first_, tol);
	  }

	  void insert(const Key1& key, const Value1& value) {first_.insert(key, value); }
	  void insert(int key, const Value1& value) {first_.insert(Key1(key), value);}

	  void insert(const TupleValuesEnd<VALUES>& values) {first_.insert(values.first_); }

	  void update(const TupleValuesEnd<VALUES>& values) {first_.update(values.first_); }

	  void update(const Key1& key, const Value1& value) { first_.update(key, value); }

	  void insertSub(const VALUES& values) {first_.insert(values); }

	  const Value1& operator[](const Key1& j) const { return first_[j]; }

	  const VALUES& values() const { return first_; }

	  void erase(const Key1& j) { first_.erase(j); }

	  void clear() { first_.clear(); }

	  bool empty() const { return first_.empty(); }

	  bool exists(const Key1& j) const { return first_.exists(j); }

	  boost::optional<Value1> exists_(const Key1& j) const { return first_.exists_(j); }

	  const Value1& at(const Key1& j) const { return first_.at(j); }

	  VectorValues zero(const Ordering& ordering) const {
		  return VectorValues::Zero(this->dims(ordering));
	  }

	  size_t size() const { return first_.size(); }

	  size_t dim() const { return first_.dim(); }

	  TupleValuesEnd<VALUES> retract(const VectorValues& delta, const Ordering& ordering) const {
	        return TupleValuesEnd(first_.retract(delta, ordering));
	  }

    VectorValues localCoordinates(const TupleValuesEnd<VALUES>& cp, const Ordering& ordering) const {
      VectorValues delta(this->dims(ordering));
      localCoordinates(cp, ordering, delta);
      return delta;
    }

    void localCoordinates(const TupleValuesEnd<VALUES>& cp, const Ordering& ordering, VectorValues& delta) const {
      first_.localCoordinates(cp.first_, ordering, delta);
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

  private:
	  friend class boost::serialization::access;
	  template<class ARCHIVE>
	  void serialize(ARCHIVE & ar, const unsigned int version) {
		  ar & BOOST_SERIALIZATION_NVP(first_);
	  }
  };

  /**
   * Wrapper classes to act as containers for valuess.  Note that these can be cascaded
   * recursively, as they are TupleValues, and are primarily a short form of the values
   * structure to make use of the TupleValues easier.
   *
   * The interface is designed to mimic PairValues, but for 2-6 values types.
   */

  template<class C1>
  class TupleValues1 : public TupleValuesEnd<C1> {
  public:
 	  // typedefs
 	  typedef C1 Values1;

 	  typedef TupleValuesEnd<C1> Base;
 	  typedef TupleValues1<C1> This;

 	  TupleValues1() {}
 	  TupleValues1(const This& values);
 	  TupleValues1(const Base& values);
 	  TupleValues1(const Values1& cfg1);

 	  // access functions
 	  inline const Values1& first() const { return this->values(); }
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
	  TupleValues2(const This& values);
	  TupleValues2(const Base& values);
	  TupleValues2(const Values1& cfg1, const Values2& cfg2);

	  // access functions
	  inline const Values1& first() const { return this->values(); }
	  inline const Values2& second() const { return this->rest().values(); }
  };

  template<class C1, class C2, class C3>
  class TupleValues3 : public TupleValues<C1, TupleValues<C2, TupleValuesEnd<C3> > > {
  public:
	  // typedefs
	  typedef C1 Values1;
	  typedef C2 Values2;
	  typedef C3 Values3;

	  TupleValues3() {}
	  TupleValues3(const TupleValues<C1, TupleValues<C2, TupleValuesEnd<C3> > >& values);
	  TupleValues3(const TupleValues3<C1, C2, C3>& values);
	  TupleValues3(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3);

	  // access functions
	  inline const Values1& first() const { return this->values(); }
	  inline const Values2& second() const { return this->rest().values(); }
	  inline const Values3& third() const { return this->rest().rest().values(); }
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
	  TupleValues4(const This& values);
	  TupleValues4(const Base& values);
	  TupleValues4(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,const Values4& cfg4);

	  // access functions
	  inline const Values1& first() const { return this->values(); }
	  inline const Values2& second() const { return this->rest().values(); }
	  inline const Values3& third() const { return this->rest().rest().values(); }
	  inline const Values4& fourth() const { return this->rest().rest().rest().values(); }
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
	  TupleValues5(const TupleValues5<C1, C2, C3, C4, C5>& values);
	  TupleValues5(const TupleValues<C1, TupleValues<C2, TupleValues<C3, TupleValues<C4, TupleValuesEnd<C5> > > > >& values);
	  TupleValues5(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,
				   const Values4& cfg4, const Values5& cfg5);

	  // access functions
	  inline const Values1& first() const { return this->values(); }
	  inline const Values2& second() const { return this->rest().values(); }
	  inline const Values3& third() const { return this->rest().rest().values(); }
	  inline const Values4& fourth() const { return this->rest().rest().rest().values(); }
	  inline const Values5& fifth() const { return this->rest().rest().rest().rest().values(); }
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
	  TupleValues6(const TupleValues6<C1, C2, C3, C4, C5, C6>& values);
	  TupleValues6(const TupleValues<C1, TupleValues<C2, TupleValues<C3, TupleValues<C4, TupleValues<C5, TupleValuesEnd<C6> > > > > >& values);
	  TupleValues6(const Values1& cfg1, const Values2& cfg2, const Values3& cfg3,
				   const Values4& cfg4, const Values5& cfg5, const Values6& cfg6);
	  // access functions
	  inline const Values1& first() const { return this->values(); }
	  inline const Values2& second() const { return this->rest().values(); }
	  inline const Values3& third() const { return this->rest().rest().values(); }
	  inline const Values4& fourth() const { return this->rest().rest().rest().values(); }
	  inline const Values5& fifth() const { return this->rest().rest().rest().rest().values(); }
	  inline const Values6& sixth() const { return this->rest().rest().rest().rest().rest().values(); }
  };

}
