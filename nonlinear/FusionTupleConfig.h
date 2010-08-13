/**
 * @file FusionTupleConfig.h
 * @brief Experimental tuple config using boost.Fusion
 * @author Alex Cunningham
 */

#pragma once

#include <boost/fusion/container/set.hpp>
#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/sequence/intrinsic.hpp>
#include <boost/fusion/include/make_set.hpp>
#include <boost/fusion/include/as_vector.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/fusion/include/has_key.hpp>
#include <boost/fusion/include/zip.hpp>
#include <boost/fusion/algorithm/transformation.hpp>
#include <boost/fusion/algorithm/iteration.hpp>
#include <boost/fusion/algorithm/query.hpp>
#include <boost/fusion/functional/adapter/fused_function_object.hpp>

#include "Testable.h"
#include "LieConfig.h"

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
		boost::fusion::at_key<LieConfig<Key, Value> >(base_tuple_).insert(j,x);
	}

	/** retrieve a point */
	template<class Key>
	const typename Key::Value_t & at(const Key& j) const {
		return boost::fusion::at_key<LieConfig<Key, typename Key::Value_t> >(base_tuple_).at(j);
	}

	/** retrieve a full config */
	template<class Config>
	const Config & config() const {
		return boost::fusion::at_key<Config>(base_tuple_);
	}

	/** size of the config - sum all sizes from subconfigs */
	size_t size() const {
		return boost::fusion::accumulate(base_tuple_, 0, FusionTupleConfig<Configs>::size_helper());
	}

	/** number of configs in the config */
	size_t nrConfigs() const {
		return boost::fusion::size(base_tuple_);
	}

	/** returns true if the config is empty */
	bool empty() const {
		return boost::fusion::all(base_tuple_, FusionTupleConfig<Configs>::empty_helper());
	}

	/** print */
	void print(const std::string& s="") const {
		std::cout << "FusionTupleConfig " << s << ":" << std::endl;
		boost::fusion::for_each(base_tuple_, FusionTupleConfig<Configs>::print_helper());
	}

	/** equals */
	bool equals(const FusionTupleConfig<Configs>& other, double tol=1e-9) const {
		FusionTupleConfig<Configs>::equals_helper helper(tol);
		return boost::fusion::all(
				boost::fusion::zip(
						boost::fusion::as_vector(base_tuple_),
						boost::fusion::as_vector(other.base_tuple_)),
						helper);
	}

	/** direct access to the underlying fusion set - don't use this */
	const BaseTuple & base_tuple() const { return base_tuple_; }

private:

	/** helper structs to make use of fusion algorithms */
	struct size_helper
	{
	    typedef size_t result_type;

	    template<typename T>
	    size_t operator()(const T& t, const size_t& s) const
	    {
	        return s + t.size();
	    }
	};

	struct empty_helper
	{
	    template<typename T>
	    bool operator()(T t) const
	    {
	        return t.empty();
	    }
	};

	struct print_helper
	{
	    template<typename T>
	    void operator()(T t) const
	    {
	        t.print();
	    }
	};

	struct equals_helper
	{
		double tol;
		equals_helper(double t) : tol(t) {}
	    template<typename T>
	    bool operator()(T t) const
	    {
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
};


} // \namespace gtsam

