/**
 * @file FusionTupleConfig.h
 * @brief Experimental tuple config using boost.Fusion
 * @author Alex Cunningham
 */

#pragma once

#include <boost/fusion/container/set.hpp>
#include <boost/fusion/include/make_set.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/fusion/algorithm/iteration.hpp>
#include <boost/fusion/algorithm/query.hpp>

#include <LieConfig.h>

namespace gtsam {

/**
 * This config uses a real tuple to store types
 * The template parameter should be a boost fusion structure, like
 * set<Config1, Config2>
 */
template<class Configs>
class FusionTupleConfig {

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

	/** initialization by slicing a larger config */
	template<class Configs2>
	FusionTupleConfig(const FusionTupleConfig<Configs2>& other) {
		// use config subinsert and fusion::foreach
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

	/** returns true if the config is empty */
	bool empty() const {
		return boost::fusion::all(base_tuple_, FusionTupleConfig<Configs>::empty_helper());
	}

private:
	/** helper structs to make use of fusion algorithms */
	struct size_helper {
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


};


} // \namespace gtsam

