/*
 * LieConfig.cpp
 *
 *  Created on: Jan 8, 2010
 *  @Author: Richard Roberts
 */

#pragma once

#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <utility>
#include <iostream>
#include <stdexcept>

#include <gtsam/linear/VectorConfig.h>
#include <gtsam/base/Lie-inl.h>

#include <gtsam/nonlinear/LieConfig.h>

#define INSTANTIATE_LIE_CONFIG(J,T) \
  /*INSTANTIATE_LIE(T);*/ \
  template LieConfig<J,T> expmap(const LieConfig<J,T>&, const VectorConfig&); \
  template LieConfig<J,T> expmap(const LieConfig<J,T>&, const Vector&); \
  template VectorConfig logmap(const LieConfig<J,T>&, const LieConfig<J,T>&); \
  template class LieConfig<J,T>;

using namespace std;

namespace gtsam {

  template<class J, class T>
  void LieConfig<J,T>::print(const string &s) const {
       cout << "LieConfig " << s << ", size " << values_.size() << "\n";
       BOOST_FOREACH(const typename Values::value_type& v, values_) {
         gtsam::print(v.second, (string)(v.first));
       }
     }

  template<class J, class T>
  bool LieConfig<J,T>::equals(const LieConfig<J,T>& expected, double tol) const {
    if (values_.size() != expected.values_.size()) return false;
    BOOST_FOREACH(const typename Values::value_type& v, values_) {
    	if (!expected.exists(v.first)) return false;
      if(!gtsam::equal(v.second, expected[v.first], tol))
        return false;
    }
    return true;
  }

  template<class J, class T>
  const T& LieConfig<J,T>::at(const J& j) const {
    const_iterator it = values_.find(j);
    if (it == values_.end()) throw std::invalid_argument("invalid j: " + (string)j);
    else return it->second;
  }

  template<class J, class T>
  size_t LieConfig<J,T>::dim() const {
  	size_t n = 0;
  	BOOST_FOREACH(const typename Values::value_type& value, values_)
  		n += gtsam::dim(value.second);
  	return n;
  }

  template<class J, class T>
  VectorConfig LieConfig<J,T>::zero() const {
  	VectorConfig z;
  	BOOST_FOREACH(const typename Values::value_type& value, values_)
  		z.insert(value.first,gtsam::zero(gtsam::dim(value.second)));
  	return z;
  }

  template<class J, class T>
  void LieConfig<J,T>::insert(const J& name, const T& val) {
    values_.insert(make_pair(name, val));
  }

  template<class J, class T>
  void LieConfig<J,T>::insert(const LieConfig<J,T>& cfg) {
	  BOOST_FOREACH(const typename Values::value_type& v, cfg.values_)
		 insert(v.first, v.second);
  }

  template<class J, class T>
  void LieConfig<J,T>::update(const LieConfig<J,T>& cfg) {
	  BOOST_FOREACH(const typename Values::value_type& v, values_) {
	  	boost::optional<T> t = cfg.exists_(v.first);
	  	if (t) values_[v.first] = *t;
	  }
  }

  template<class J, class T>
  void LieConfig<J,T>::update(const J& j, const T& val) {
	  	values_[j] = val;
  }

  template<class J, class T>
  std::list<J> LieConfig<J,T>::keys() const {
	  std::list<J> ret;
	  BOOST_FOREACH(const typename Values::value_type& v, values_)
		  ret.push_back(v.first);
	  return ret;
  }

  template<class J, class T>
  void LieConfig<J,T>::erase(const J& j) {
    size_t dim; // unused
    erase(j, dim);
  }

  template<class J, class T>
  void LieConfig<J,T>::erase(const J& j, size_t& dim) {
    iterator it = values_.find(j);
    if (it == values_.end()) throw std::invalid_argument("invalid j: " + (string)j);
    dim = gtsam::dim(it->second);
    values_.erase(it);
  }

  // todo: insert for every element is inefficient
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const VectorConfig& delta) {
		LieConfig<J,T> newConfig;
		typedef pair<J,T> KeyValue;
		BOOST_FOREACH(const KeyValue& value, c) {
			const J& j = value.first;
			const T& pj = value.second;
			Symbol jkey = (Symbol)j;
			if (delta.contains(jkey)) {
				const Vector& dj = delta[jkey];
				newConfig.insert(j, expmap(pj,dj));
			} else
				newConfig.insert(j, pj);
		}
		return newConfig;
	}

  // todo: insert for every element is inefficient
  template<class J, class T>
  LieConfig<J,T> expmap(const LieConfig<J,T>& c, const Vector& delta) {
    if(delta.size() != dim(c)) {
    	cout << "LieConfig::dim (" << dim(c) << ") <> delta.size (" << delta.size() << ")" << endl;
      throw invalid_argument("Delta vector length does not match config dimensionality.");
    }
    LieConfig<J,T> newConfig;
    int delta_offset = 0;
		typedef pair<J,T> KeyValue;
		BOOST_FOREACH(const KeyValue& value, c) {
			const J& j = value.first;
			const T& pj = value.second;
      int cur_dim = dim(pj);
      newConfig.insert(j,expmap(pj,sub(delta, delta_offset, delta_offset+cur_dim)));
      delta_offset += cur_dim;
    }
    return newConfig;
  }

  // todo: insert for every element is inefficient
  // todo: currently only logmaps elements in both configs
  template<class J, class T>
  VectorConfig logmap(const LieConfig<J,T>& c0, const LieConfig<J,T>& cp) {
  	VectorConfig delta;
		typedef pair<J,T> KeyValue;
  	BOOST_FOREACH(const KeyValue& value, cp) {
  		if(c0.exists(value.first))
  			delta.insert(value.first,
  					logmap(c0[value.first], value.second));
  	}
  	return delta;
  }

}


