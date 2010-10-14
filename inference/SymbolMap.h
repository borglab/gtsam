/*
 * SymbolMap.h
 *
 *  Created on: Jan 20, 2010
 *      Author: richard
 */

#pragma once

//#define GTSAM_SYMBOL_HASH
#define GTSAM_SYMBOL_BINARY
#define GTSAM_SYMBOL_SPECIAL

#include <gtsam/nonlinear/Key.h>

#include <map>
#include <boost/unordered_map.hpp>


namespace gtsam {

#ifdef GTSAM_SYMBOL_BINARY
	template<class T>
	class SymbolMap : public std::map<Symbol, T> {
	private:
		typedef std::map<Symbol, T> Base;
	public:
		SymbolMap() : std::map<Symbol, T>() {}

		const T& at(const Symbol& key) const {
			typename Base::const_iterator it = Base::find(key);
			if (it == Base::end())
				throw(std::invalid_argument("SymbolMap::[] invalid key: " + (std::string)key));
			return it->second;
		}

		T& at(const Symbol& key) {
			typename Base::iterator it = Base::find(key);
			if (it == Base::end())
				throw(std::invalid_argument("SymbolMap::[] invalid key: " + (std::string)key));
			return it->second;
		}

		//void find(void);

		//void clear() { throw std::runtime_error("Clear should not be used!"); }

	};
#endif


#ifdef GTSAM_SYMBOL_HASH
	struct SymbolHash : public std::unary_function<Symbol, std::size_t> {
		std::size_t operator()(Symbol const& x) const {
			std::size_t seed = 0;
			boost::hash_combine(seed, x.chr());
			boost::hash_combine(seed, x.index());
			return ((size_t(x.chr()) << 24) & x.index());
		}
	};

	template<class T>
	class SymbolMap : public boost::unordered_map<Symbol, T, SymbolHash> {
	public:
		SymbolMap() : boost::unordered_map<Symbol, T, SymbolHash>() {}
	};
#endif


#ifdef GTSAM_SYMBOL_SPECIAL
	template<class T>
	class FastSymbolMap {
	private:
		typedef std::vector<std::vector<T> > Map;
		typedef std::vector<T> Vec;

		Map values_;

	public:
		typedef std::pair<Symbol, T> value_type;

		FastSymbolMap() {
			values_.resize(256);
			values_[size_t('x')].reserve(10000);
			values_[size_t('l')].reserve(1000);
		}

		const T& at(const Symbol& key) const {
//			typename Map::const_iterator it = values_.find(key.chr());
//			if(it != values_.end())
//				return it->second.at(key.index());
//			else
//				throw std::invalid_argument("Key " + (std::string)key + " not present");
			return values_.at(size_t(key.chr())).at(key.index());
		}

		void insert(const value_type& val) {
			Vec& vec(values_[size_t(val.first.chr())]);
			if(val.first.index() >= vec.size()) {
				vec.reserve(val.first.index()+1);
				vec.resize(val.first.index());
				vec.push_back(val.second);
			} else
				vec[val.first.index()] = val.second;
		}

		bool empty() const {
			return false;
		}

		void erase(const Symbol& key) {

		}

		void clear() {
			throw std::runtime_error("Can't clear a FastSymbolMap");
		}

//		typedef std::pair<Symbol, T> value_type;
//
//		class iterator {
//			typename Map::iterator map_it_;
//			typename Map::iterator map_end_;
//			typename Vec::iterator vec_it_;
//		public:
//			iterator() {}
//			iterator(const iterator& it) : map_it_(it.map_it_), vec_it_(it.vec_it_) {}
//			bool operator==(const iterator& it);// { return map_it_==it.map_it_ && vec_it_==it.vec_it_; }
//			bool operator!=(const iterator& it);// { return map_it_!=it.map_it_ || vec_it_!=it.vec_it_; }
//			bool operator*();// { return *it.vec_it_; }
//			iterator& operator++(); /* {
//				if(map_it_ != map_end_ && vec_it_ == map_it_->second.end())
//					do
//						vec_it_ = (map_it_++)->second.begin();
//					while(map_it_ != map_end_ && vec_it_ == map_it_->second.end());
//				else
//					vec_it_++;
//				return *this;
//			}*/
//			iterator operator++(int); /* {
//				iterator tmp(*this);
//				++(*this);
//				return tmp;
//			}*/
//		};
//		class const_iterator {};


//		std::size_t size() const;
//		T& at(const Symbol& key);
//		const_iterator find(const Symbol& key);
//		void insert(const std::pair<Symbol, T>& p);
//    void clear() { throw std::runtime_error("Clear should not be used!"); }
//    std::size_t count() const;
//
//		const_iterator begin() const;
//		const_iterator end() const;
	};

#endif
}
