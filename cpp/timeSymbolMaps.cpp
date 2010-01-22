/*
 * timeSymbolMaps.cpp
 *
 *  Created on: Jan 20, 2010
 *      Author: richard
 */

#include <boost/unordered_map.hpp>
#include <string>
#include <boost/timer.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <map>

#include "Key.h"

using namespace std;
using namespace boost;
using namespace gtsam;

template<class T>
class SymbolMapExp {
private:
	typedef map<unsigned char, vector<T> > Map;
	typedef vector<T> Vec;

	Map values_;

public:
	typedef pair<Symbol, T> value_type;

	SymbolMapExp() {}

	T& at(const Symbol& key) {
		typename Map::iterator it = values_.find(key.chr());
		if(it != values_.end())
			return it->second.at(key.index());
		else
			throw invalid_argument("Key " + (string)key + " not present");
	}

	void set(const Symbol& key, const T& value) {
		Vec& vec(values_[key.chr()]);
		//vec.reserve(10000);
		if(key.index() >= vec.size()) {
			vec.reserve(key.index()+1);
			vec.resize(key.index());
			vec.push_back(value);
		} else
			vec[key.index()] = value;
	}
};

template<class T>
class SymbolMapBinary : public std::map<Symbol, T> {
private:
	typedef std::map<Symbol, T> Base;
public:
	SymbolMapBinary() : std::map<Symbol, T>() {}

	T& at(const Symbol& key) {
		typename Base::iterator it = Base::find(key);
		if (it == Base::end())
			throw(std::invalid_argument("SymbolMap::[] invalid key: " + (std::string)key));
		return it->second;
	}
};

struct SymbolHash : public std::unary_function<Symbol, std::size_t> {
	std::size_t operator()(Symbol const& x) const {
		std::size_t seed = 0;
		boost::hash_combine(seed, x.chr());
		boost::hash_combine(seed, x.index());
		return ((size_t(x.chr()) << 24) & x.index());
	}
};

template<class T>
class SymbolMapHash : public boost::unordered_map<Symbol, T, SymbolHash> {
public:
	SymbolMapHash() : boost::unordered_map<Symbol, T, SymbolHash>(60000) {}
};

struct Value {
	double v;
	Value() : v(0.0) {}
	Value(double vi) : v(vi) {}
	operator string() { lexical_cast<string>(v); }
	bool operator!=(const Value& vc) { return v != vc.v; }
};

#define ELEMS 3000
#define TIMEAT 300

int main(int argc, char *argv[]) {
	timer tmr;

	// pre-allocate
	cout << "Generating test data ..." << endl;
	vector<pair<Symbol, Value> > values;
	for(size_t i=0; i<ELEMS; i++) {
		values.push_back(make_pair(Symbol('a',i), (double)i));
		values.push_back(make_pair(Symbol('b',i), (double)i));
		values.push_back(make_pair(Symbol('c',i), (double)i));
	}

	// time binary map
	cout << "Timing binary map ..." << endl;
	{
		SymbolMapBinary<Value> binary;
		for(size_t i=0; i<ELEMS*3; ) {
			size_t stop = i + TIMEAT;
			tmr.restart();
			for( ; i<stop; i++)
				binary.insert(values[i]);
			double time = tmr.elapsed();
			cout << i << " values, avg " << (time/(double)TIMEAT)*1e6 << " mu-s per insert" << endl;

			tmr.restart();
			for(size_t j=0; j<i; j++)
				if(values[j].second != binary[values[j].first]) {
					cout << "Wrong value!  At key " << (string)values[j].first <<
							" expecting " << (string)values[j].second <<
							" got " << (string)binary[values[j].first] << endl;
				}
			time = tmr.elapsed();
			cout << i << " values, avg " << (time)*1e3 << " ms per lookup" << endl;
		}
	}

	// time hash map
	cout << "Timing hash map ..." << endl;
	{
		SymbolMapHash<Value> hash;
		for(size_t i=0; i<ELEMS*3; ) {
			size_t stop = i + TIMEAT;
			tmr.restart();
			for( ; i<stop; i++)
				hash.insert(values[i]);
			double time = tmr.elapsed();
			cout << i << " values, avg " << (time/(double)TIMEAT)*1e6 << " mu-s per insert" << endl;

			tmr.restart();
			for(size_t j=0; j<i; j++)
				if(values[j].second != hash[values[j].first]) {
					cout << "Wrong value!  At key " << (string)values[j].first <<
							" expecting " << (string)values[j].second <<
							" got " << (string)hash[values[j].first] << endl;
				}
			time = tmr.elapsed();
			cout << i << " values, avg " << (time/(double)i)*1e6 << " mu-s per lookup" << endl;
		}
	}

	// time experimental map
	cout << "Timing experimental map ..." << endl;
	{
		SymbolMapExp<Value> experimental;
		for(size_t i=0; i<ELEMS*3; ) {
			size_t stop = i + TIMEAT;
			tmr.restart();
			for( ; i<stop; i++)
				experimental.set(values[i].first, values[i].second);
			double time = tmr.elapsed();
			cout << i << " values, avg " << (time/(double)TIMEAT)*1e6 << " mu-s per insert" << endl;

			tmr.restart();
			for(size_t j=0; j<i; j++)
				if(values[j].second != experimental.at(values[j].first)) {
					cout << "Wrong value!  At key " << (string)values[j].first <<
							" expecting " << (string)values[j].second <<
							" got " << (string)experimental.at(values[j].first) << endl;
				}
			time = tmr.elapsed();
			cout << i << " values, avg " << (time/(double)i)*1e6 << " mu-s per lookup" << endl;
		}
	}
}
