/**
 * @file FusionTupleConfig.h
 * @brief Experimental tuple config using boost.Fusion
 * @author Alex Cunningham
 */

#pragma once

#include <boost/fusion/container/set.hpp>
#include <boost/fusion/include/make_set.hpp>
#include <boost/fusion/include/as_vector.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/fusion/include/has_key.hpp>
#include <boost/fusion/include/zip.hpp>
#include <boost/fusion/include/all.hpp>
#include <boost/fusion/algorithm/iteration.hpp>

#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/LieConfig.h>

namespace gtsam {

/**
 * This config uses a real tuple to store types
 * The template parameter should be a boost fusion structure, like
 * set<Config1, Config2>
 */
template<class Configs>
class FusionTupleConfig : public Testable<FusionTupleConfig<Configs> >{

public:
	/** useful types */
	typedef Configs BaseTuple;

protected:
	/** the underlying tuple storing everything */
	Configs base_tuple_;

private:
  /** helper structs to make use of fusion algorithms */
  struct size_helper  {
    typedef size_t result_type;
      template<typename T>
      size_t operator()(const T& t, const size_t& s) const {
          return s + t.size();
      }
  };

  struct dim_helper {
      typedef size_t result_type;
      template<typename T>
      size_t operator()(const T& t, const size_t& s) const {
          return s + t.dim();
      }
  };

  struct zero_helper  {
      typedef VectorConfig result_type;
      template<typename T>
      result_type operator()(const T& t, const result_type& s) const {
        result_type new_s(s);
        new_s.insert(t.zero());
          return new_s;
      }
  };

  struct update_helper {
    typedef Configs result_type;
      template<typename T>
      result_type operator()(const T& t, const result_type& s) const {
        result_type new_s(s);
        boost::fusion::at_key<T>(new_s).update(t);
          return new_s;
      }
  };

  struct expmap_helper {
      typedef FusionTupleConfig<Configs> result_type;
      VectorConfig delta;
      expmap_helper(const VectorConfig& d) : delta(d) {}
      template<typename T>
      result_type operator()(const T& t, const result_type& s) const {
        result_type new_s(s);
        boost::fusion::at_key<T>(new_s.base_tuple_) = T(gtsam::expmap(t, delta));
          return new_s;
      }
  };

  struct logmap_helper {
    typedef VectorConfig result_type;
      template<typename T>
      result_type operator()(const T& t, const result_type& s) const {
        result_type new_s(s);
        new_s.insert(gtsam::logmap(boost::fusion::at_c<0>(t), boost::fusion::at_c<1>(t)));
          return new_s;
      }
  };

  struct empty_helper {
      template<typename T>
      bool operator()(T t) const {
          return t.empty();
      }
  };

  struct print_helper {
      template<typename T>
      void operator()(T t) const {
          t.print();
      }
  };

  struct equals_helper {
    double tol;
    equals_helper(double t) : tol(t) {}
      template<typename T>
      bool operator()(T t) const {
          return boost::fusion::at_c<0>(t).equals(boost::fusion::at_c<1>(t), tol);
      }
  };

  /** two separate function objects for arbitrary copy construction */
  template<typename Ret, typename Config>
  struct assign_inner {
    typedef Ret result_type;

    Config config;
    assign_inner(const Config& cfg) : config(cfg) {}

    template<typename T>
    result_type operator()(const T& t, const result_type& s) const {
      result_type new_s(s);
      T new_cfg(config);
      if (!new_cfg.empty()) boost::fusion::at_key<T>(new_s) = new_cfg;
      return new_s;
    }
  };

  template<typename Ret>
  struct assign_outer {
    typedef Ret result_type;

    template<typename T> // T is the config from the "other" config
    Ret operator()(const T& t, const Ret& s) const {
      assign_inner<Ret, T> helper(t);
      return boost::fusion::fold(s, s, helper); // loop over the "self" config
    }
  };

public:
	/** create an empty config */
	FusionTupleConfig() {}

	/** copy constructor */
	FusionTupleConfig(const FusionTupleConfig<Configs>& other) : base_tuple_(other.base_tuple_) {}

	/** direct initialization of the underlying structure */
	FusionTupleConfig(const Configs& cfg_set) : base_tuple_(cfg_set) {}

	/** initialization from arbitrary other configs */
	template<class Configs2>
	FusionTupleConfig(const FusionTupleConfig<Configs2>& other)
		: base_tuple_(boost::fusion::fold(other.base_tuple(), Configs(), assign_outer<Configs>()))
	{
	}

	virtual ~FusionTupleConfig() {}

	/** insertion  */
	template <class Key, class Value>
	void insert(const Key& j, const Value& x) {
		config_<LieConfig<Key> >().insert(j,x);
	}

	/** insert a full config at a time */
	template<class Config>
	void insertSub(const Config& config) {
		config_<Config>().insert(config);
	}

	/**
	 * Update function for whole configs - this will change existing values
	 * @param config is a config to add
	 */
	void update(const FusionTupleConfig<Configs>& config) {
		base_tuple_ = boost::fusion::accumulate(
				config.base_tuple_,	base_tuple_,
				update_helper());
	}

	/**
	 * Update function for whole subconfigs - this will change existing values
	 * @param config is a config to add
	 */
	template<class Config>
	void update(const Config& config) {
		config_<Config>().update(config);
	}

	/**
	 * Update function for single key/value pairs - will change existing values
	 * @param key is the variable identifier
	 * @param value is the variable value to update
	 */
	template<class Key, class Value>
	void update(const Key& key, const Value& value) {
		config_<LieConfig<Key> >().update(key,value);
	}

	/** check if a given element exists */
	template<class Key>
	bool exists(const Key& j) const {
		return config<LieConfig<Key> >().exists(j);
	}

	/** a variant of exists */
	template<class Key>
	boost::optional<typename Key::Value_t> exists_(const Key& j)  const {
		return config<LieConfig<Key> >().exists_(j);
	}

	/** retrieve a point */
	template<class Key>
	const typename Key::Value_t & at(const Key& j) const {
		return config<LieConfig<Key> >().at(j);
	}

	/** access operator */
	template<class Key>
	const typename Key::Value_t & operator[](const Key& j) const { return at<Key>(j); }

	/** retrieve a reference to a full subconfig */
	template<class Config>
	const Config & config() const {
		return boost::fusion::at_key<Config>(base_tuple_);
	}

	/** size of the config - sum all sizes from subconfigs */
	size_t size() const {
		return boost::fusion::accumulate(base_tuple_, 0,
				size_helper());
	}

	/** combined dimension of the subconfigs */
	size_t dim() const {
		return boost::fusion::accumulate(base_tuple_, 0,
				dim_helper());
	}

	/** number of configs in the config */
	size_t nrConfigs() const {
		return boost::fusion::size(base_tuple_);
	}

	/** erases a specific key */
	template<class Key>
	void erase(const Key& j) {
		config_<LieConfig<Key> >().erase(j);
	}

	/** clears the config */
	void clear() {
		base_tuple_ = Configs();
	}

	/** returns true if the config is empty */
	bool empty() const {
		return boost::fusion::all(base_tuple_,
				empty_helper());
	}

	/** print */
	void print(const std::string& s="") const {
		std::cout << "FusionTupleConfig " << s << ":" << std::endl;
		boost::fusion::for_each(base_tuple_, print_helper());
	}

	/** equals */
	bool equals(const FusionTupleConfig<Configs>& other, double tol=1e-9) const {
		equals_helper helper(tol);
		return boost::fusion::all(
				boost::fusion::zip(base_tuple_,other.base_tuple_), helper);
	}

	/** zero: create VectorConfig of appropriate structure */
	VectorConfig zero() const {
		return boost::fusion::accumulate(base_tuple_, VectorConfig(),
				zero_helper());
	}

	FusionTupleConfig<Configs> expmap(const VectorConfig& delta) const {
		return boost::fusion::accumulate(base_tuple_, base_tuple_,
				expmap_helper(delta));
	}

	VectorConfig logmap(const FusionTupleConfig<Configs>& cp) const {
		return boost::fusion::accumulate(boost::fusion::zip(base_tuple_, cp.base_tuple_),
				VectorConfig(), logmap_helper());
	}

	/**
	 * direct access to the underlying fusion set - don't use this normally
	 * TODO: make this a friend function or similar
	 */
	const BaseTuple & base_tuple() const { return base_tuple_; }

private:

	/** retrieve a non-const reference to a full subconfig */
	template<class Config>
	Config & config_() {
		return boost::fusion::at_key<Config>(base_tuple_);
	}


	/** Serialization function */
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version) {
		ar & BOOST_SERIALIZATION_NVP(base_tuple_);
	}

};

/** Exmap static functions */
template<class Configs>
inline FusionTupleConfig<Configs> expmap(const FusionTupleConfig<Configs>& c, const VectorConfig& delta) {
	  return c.expmap(delta);
}

/** logmap static functions */
template<class Configs>
inline VectorConfig logmap(const FusionTupleConfig<Configs>& c0, const FusionTupleConfig<Configs>& cp) {
	  return c0.logmap(cp);
}

/**
 * Easy-to-use versions of FusionTupleConfig
 * These versions provide a simpler interface more like
 * the existing TupleConfig.  Each version has a number, and
 * takes explicit template arguments for config types.
 */
template<class C1>
struct FusionTupleConfig1 : public FusionTupleConfig<boost::fusion::set<C1> > {
	typedef FusionTupleConfig<boost::fusion::set<C1> > Base;
	typedef FusionTupleConfig1<C1> This;

	typedef C1 Config1;

	FusionTupleConfig1() {}
	FusionTupleConfig1(const Base& c) : Base(c) {}
	FusionTupleConfig1(const C1& c1) : Base(boost::fusion::make_set(c1)) {}

	const Config1& first() const { return boost::fusion::at_key<C1>(this->base_tuple_); }
};

template<class C1, class C2>
struct FusionTupleConfig2 : public FusionTupleConfig<boost::fusion::set<C1, C2> > {
	typedef FusionTupleConfig<boost::fusion::set<C1, C2> > Base;
	typedef FusionTupleConfig2<C1,C2> This;

	typedef C1 Config1;
	typedef C2 Config2;

	FusionTupleConfig2() {}
	FusionTupleConfig2(const Base& c) : Base(c) {}
	FusionTupleConfig2(const C1& c1, const C2& c2) : Base(boost::fusion::make_set(c1, c2)) {}

	const Config1& first()  const { return boost::fusion::at_key<C1>(this->base_tuple_); }
	const Config2& second() const { return boost::fusion::at_key<C2>(this->base_tuple_); }
};

template<class C1, class C2, class C3>
struct FusionTupleConfig3 : public FusionTupleConfig<boost::fusion::set<C1, C2, C3> > {
	typedef FusionTupleConfig<boost::fusion::set<C1, C2, C3> > Base;

	typedef C1 Config1;
	typedef C2 Config2;
	typedef C3 Config3;

	FusionTupleConfig3() {}
	FusionTupleConfig3(const Base& c) : Base(c) {}
	FusionTupleConfig3(const C1& c1, const C2& c2, const C3& c3)
		: Base(boost::fusion::make_set(c1, c2, c3)) {}

	const Config1& first()  const { return boost::fusion::at_key<C1>(this->base_tuple_); }
	const Config2& second() const { return boost::fusion::at_key<C2>(this->base_tuple_); }
	const Config3& third()  const { return boost::fusion::at_key<C3>(this->base_tuple_); }
};


} // \namespace gtsam

